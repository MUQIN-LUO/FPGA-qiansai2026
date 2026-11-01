

// ==========================================================================
// K1 标定电平跨时钟：I_sys_clk 域 -> CSI 域
// --------------------------------------------------------------------------
// S_cal_tgl 由 50 MHz 域的 1.5 s 长按计数器翻转。isp_top 工作在 CSI 域，
// 其 I_cal_mode 必须使用本模块同步后的 O_mode，**不能直接接 S_cal_tgl**。
// O_mode 与 O_trig 同域同拍：
//   O_mode = t2（两级同步后的电平）
//   O_trig = t2 ^ t3（电平变化的单拍脉冲）
// 因此上升沿同一拍满足 (O_mode=1, O_trig=1) -> CAL 启动；
// 下降沿同一拍满足 (O_mode=0, O_trig=1) -> CAL 中止并回到自动。
// 复位：t1..t3 异步清 0；源域 S_cal_tgl 也复位为 0，上电不会产生伪触发。
// 本模块被顶层实例化，同时被 sim/tb/tb_cal_mode_cdc.v 直接验证。
// ==========================================================================
module cal_mode_cdc (
    input  wire I_csi_clk,
    input  wire I_csi_rst_n,
    input  wire I_cal_tgl,      // 源域电平（异步）
    output wire O_mode,         // 同步后电平（CSI 域）
    output wire O_trig          // 电平变化的单拍脉冲（CSI 域）
);
    reg t1, t2, t3;
    always @(posedge I_csi_clk or negedge I_csi_rst_n) begin
        if (!I_csi_rst_n) begin
            t1 <= 1'b0;
            t2 <= 1'b0;
            t3 <= 1'b0;
        end
        else begin
            t1 <= I_cal_tgl;
            t2 <= t1;
            t3 <= t2;
        end
    end
    assign O_mode = t2;
    assign O_trig = t2 ^ t3;
endmodule


module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    inout wire [3:0]  I_button,

    output wire       O_screen_pwm,
    output wire       O_led1,          // 1 Hz 心跳灯，LED1，高电平点亮
    output wire       O_buzzer,        // 无源蜂鸣器单声部方波，C12

    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
	(* keep = "true" *) wire [15:0] S_csi_data_length;
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;

	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;
    wire[1:0]   S_video_in_wp;

    // 黄色目标检测结果（CSI 域 -> 像素域）
    wire [11:0] S_det_minx, S_det_maxx, S_det_miny, S_det_maxy;
    wire [11:0] S_det_cx, S_det_cy;   // fitted/bbox centre, same result group
    wire        S_det_green;          // optional green label, same group
    wire        S_det_valid, S_det_cand;
    wire        S_det_fresh_tgl;      // one toggle per real detection pass
    wire [9:0]  S_det_cnt;
    // 圆形诊断。EVT_CIRCLE=0: {ovf,live,chord,rows,asp,size,pass,valid};
    // EVT_CIRCLE=1: {ovf,live,drop,0,0,0,pass,valid}（见 isp_evt_hud_map）。
    wire [7:0]  S_det_flags;
    // Red ball has its own result and handshake; yellow CDC remains unchanged.
    wire [11:0] S_red_minx, S_red_maxx, S_red_miny, S_red_maxy;
    wire S_red_valid, S_red_cand, S_red_update;
    wire [9:0] S_red_count;
    wire [59:0] S_red_pixel_snapshot;

    // ---- 一键标定 / 检测结果整组快照 ----
    reg  [31:0] S_k1_cnt;
    reg         S_k1_fired;
    reg         S_cal_tgl;
    wire [1:0]  S_cal_st;
    reg  [1:0]  S_cal_st_l;   // 上一拍的标定状态，用于状态变化即时上报
    // 先声明：帧率统计（更靠前的位置）和 CSI 快照都要用
    reg  [9:0] S_fps_latched;
    wire       S_isp_frm_tgl;

    // ---- 跨域快照：完整 payload + 请求/应答（不再逐位 2FF）----
    //   CSI 域把一帧的整组结果先写进 staging，等上一次事务被 ack 之后
    //   才整体搬进 payload 并翻转请求。payload 从翻转到收到 ack 之间
    //   绝不会被改写，所以对端读到的必然是同一帧的一组值。
    reg  [11:0] S_stg_minx, S_stg_maxx, S_stg_miny, S_stg_maxy;
    reg  [11:0] S_stg_cx, S_stg_cy;
    reg         S_stg_green;
    reg         S_stg_valid, S_stg_cand;
    reg  [ 9:0] S_stg_cnt;
    reg  [ 7:0] S_stg_flags;   // 圆形诊断位，与 bbox/valid/cand 同一快照组
    reg  [ 1:0] S_stg_cal;
    reg  [ 9:0] S_stg_gr, S_stg_gb;
    reg  [ 7:0] S_stg_ar, S_stg_ag, S_stg_ab;
    reg         S_stg_vld;
    reg         S_ack1, S_ack2, S_ack_tgl;
    reg  [ 9:0] S_sn_gr, S_sn_gb, S_rc_gr, S_rc_gb;
    reg  [ 7:0] S_sn_ar, S_sn_ag, S_sn_ab, S_rc_ar, S_rc_ag, S_rc_ab;
    wire        S_cal_t2, S_cal_trig;   // 由 cal_mode_cdc 在 CSI 域同步产生

    reg  [11:0] S_sn_minx, S_sn_maxx, S_sn_miny, S_sn_maxy;
    reg  [11:0] S_sn_cx, S_sn_cy;
    reg         S_sn_green;
    reg         S_sn_valid, S_sn_cand;
    reg  [9:0]  S_sn_cnt;
    reg  [7:0]  S_sn_flags;
    reg  [1:0]  S_sn_cal;
    reg         S_sn_tgl;
    reg         S_ft1, S_ft2, S_ft3;
    wire        S_frm_evt = S_ft2 ^ S_ft3;

    reg  [11:0] S_rc_minx, S_rc_maxx, S_rc_miny, S_rc_maxy;
    reg  [11:0] S_rc_cx, S_rc_cy;
    reg         S_rc_green;
    reg         S_rc_valid, S_rc_cand;
    reg  [9:0]  S_rc_cnt;
    reg  [7:0]  S_rc_flags;
    reg  [1:0]  S_rc_cal;
    reg         S_st1, S_st2, S_st3;
    wire        S_snap_evt = S_st2 ^ S_st3;
    reg  [1:0]  S_cal_st_p;

    reg  [11:0] S_det_minx_p, S_det_maxx_p, S_det_miny_p, S_det_maxy_p;
    reg  [11:0] S_det_cx_p, S_det_cy_p;
    reg         S_det_green_p;
    reg         S_det_valid_p, S_det_cand_p;
    reg  [9:0]  S_det_cnt_p;
    reg  [7:0]  S_det_flags_p;

    // 中心区域平均色（调试）
    wire [7:0]  S_avg_r, S_avg_g, S_avg_b;
    reg  [7:0]  S_avg_r_p, S_avg_g_p, S_avg_b_p;

    wire [7:0]  S_lum;          // ISP 全帧 G 均值（像素域）
    wire        S_lum_tgl;      // 每帧翻转一次的握手电平

    wire [9:0]  S_gr100, S_gb100;           // 当前白平衡增益 x100（ISP 域）
    reg  [9:0]  S_gr100_p, S_gb100_p;       // 锁存到像素域供 OSD 显示
    reg  [15:0] S_ae_pix, S_ag_pix;         // AE 曝光/增益（经 CSI 中继到像素域）
    reg  [15:0] S_ae_csi, S_ag_csi;         // CSI 域中继（24MHz→CSI→HDMI）

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;
    wire        S_csi_packet_start;
    reg         S_csi_length_active_d;
    reg [10:0]  S_mipi_line_count;
    reg         S_mipi_length_error;
    reg         S_mipi_lane_error;
    reg         S_mipi_frame_seen;
    reg         S_mipi_frame_done;
    reg         S_mipi_last_length_error;
    reg         S_mipi_last_line_error;
    reg         S_mipi_last_lane_error;
    wire[3:0]   S_mipi_diag_status_async;
    reg [3:0]   S_mipi_diag_status_sync1;
    reg [3:0]   S_mipi_diag_status_sync2;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;

    // ------------------------------------------------------------------
    // 1 Hz 心跳指示灯 + 1 秒计时脉冲：LED1（FPGA 引脚 D10，高电平点亮）
    // 系统时钟 50 MHz，半周期 = 50e6 / (2 x 1 Hz) = 25,000,000 个时钟
    // ------------------------------------------------------------------
    localparam integer LED_HALF_PERIOD = 25000000;

    reg [24:0] S_led_cnt;
    reg        S_led_1hz;
    reg        S_tick_500ms;              // 电平每 0.5 秒翻转一次（宽电平，跨域安全）

    always @(posedge I_sys_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_led_cnt <= 25'd0;
            S_led_1hz <= 1'b0;
            S_tick_500ms <= 1'b0;
        end
        else begin
            if (S_led_cnt >= LED_HALF_PERIOD - 1) begin
                S_led_cnt <= 25'd0;
                S_led_1hz <= ~S_led_1hz;
                // 用翻转而不是单周期脉冲：CSI 字节时钟(约 34.5MHz)比 50MHz 还慢，
                // 20ns 的单周期脉冲会被漏采，导致计数窗口漏清零、帧率翻倍。
                S_tick_500ms <= ~S_tick_500ms;
            end
            else begin
                S_led_cnt <= S_led_cnt + 25'd1;
            end
        end
    end

    assign O_led1 = S_led_1hz;

    // ------------------------------------------------------------------
    // 多算法切换（进阶要求 1）：板载 K1~K4 按键选择算法
    //   KEY1(D5)  -> 原图 RAW
    //   KEY2(A9)  -> 边缘 EDGE
    //   KEY3(B9)  -> 灰度 GRAY
    //   KEY4(C7)  -> 波普 POP（循环切换 4 种风格）
    //   按键内部上拉，空闲为 1，按下为 0；20 ms 消抖后取下降沿
    // ------------------------------------------------------------------
    localparam integer BTN_DEBOUNCE = 1000000;   // 50 MHz x 20 ms

    reg [3:0]  S_btn_q;          // 异步按键第 1 拍
    reg [3:0]  S_btn_q2;         // 异步按键第 2 拍（先去两级再去抖）
    reg [3:0]  S_btn_st;         // 消抖后的稳定值
    reg [3:0]  S_btn_st_d;       // 稳定值再打一拍，用于取下降沿
    reg [19:0] S_btn_cnt0;
    reg [19:0] S_btn_cnt1;
    reg [19:0] S_btn_cnt2;
    reg [19:0] S_btn_cnt3;
    reg [1:0]  S_mode;           // 0=原图 1=边缘 2=灰度

    wire [3:0] S_btn_press = S_btn_st_d & ~S_btn_st;

    always @(posedge I_sys_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_btn_q     <= 4'b1111;
            S_btn_q2    <= 4'b1111;
            S_btn_st    <= 4'b1111;
            S_btn_st_d  <= 4'b1111;
            S_btn_cnt0  <= 20'd0;
            S_btn_cnt1  <= 20'd0;
            S_btn_cnt2  <= 20'd0;
            S_btn_cnt3  <= 20'd0;
            S_mode      <= 2'd0;
        end
        else begin
            S_btn_q    <= I_button;
            S_btn_q2   <= S_btn_q;
            S_btn_st_d <= S_btn_st;

            // KEY1
            if (S_btn_q2[0] != S_btn_st[0]) begin
                if (S_btn_cnt0 >= BTN_DEBOUNCE) begin
                    S_btn_st[0] <= S_btn_q2[0];
                    S_btn_cnt0  <= 20'd0;
                end
                else S_btn_cnt0 <= S_btn_cnt0 + 20'd1;
            end
            else S_btn_cnt0 <= 20'd0;

            // KEY2
            if (S_btn_q2[1] != S_btn_st[1]) begin
                if (S_btn_cnt1 >= BTN_DEBOUNCE) begin
                    S_btn_st[1] <= S_btn_q2[1];
                    S_btn_cnt1  <= 20'd0;
                end
                else S_btn_cnt1 <= S_btn_cnt1 + 20'd1;
            end
            else S_btn_cnt1 <= 20'd0;

            // KEY3
            if (S_btn_q2[2] != S_btn_st[2]) begin
                if (S_btn_cnt2 >= BTN_DEBOUNCE) begin
                    S_btn_st[2] <= S_btn_q2[2];
                    S_btn_cnt2  <= 20'd0;
                end
                else S_btn_cnt2 <= S_btn_cnt2 + 20'd1;
            end
            else S_btn_cnt2 <= 20'd0;

            // KEY4
            if (S_btn_q2[3] != S_btn_st[3]) begin
                if (S_btn_cnt3 >= BTN_DEBOUNCE) begin
                    S_btn_st[3] <= S_btn_q2[3];
                    S_btn_cnt3  <= 20'd0;
                end
                else S_btn_cnt3 <= S_btn_cnt3 + 20'd1;
            end
            else S_btn_cnt3 <= 20'd0;

            // 直接选择；KEY4 循环
            if (S_btn_press[0])
                S_mode <= 2'd0;
            else if (S_btn_press[1])
                S_mode <= 2'd1;
            else if (S_btn_press[2])
                S_mode <= 2'd2;
            else if (S_btn_press[3])
                S_mode <= (S_mode == 2'd3) ? 2'd0 : (S_mode + 2'd1);
        end
    end

    // 模式号跨到 CSI 域（ISP 选择通路）与像素域（OSD 显示）
    reg [1:0] S_mode_csi1, S_mode_csi2;
    reg [1:0] S_mode_pix1, S_mode_pix2;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_mode_csi1 <= 2'd0;
            S_mode_csi2 <= 2'd0;
        end
        else begin
            S_mode_csi1 <= S_mode;
            S_mode_csi2 <= S_mode_csi1;
        end
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_mode_pix1 <= 2'd0;
            S_mode_pix2 <= 2'd0;
        end
        else begin
            S_mode_pix1 <= S_mode;
            S_mode_pix2 <= S_mode_pix1;
        end
    end

    // ------------------------------------------------------------------
    // 波普调色板自动轮换：0.5 s 计时脉冲计 6 次 = 每 3 秒换一套配色
    //   对应《玛丽莲·梦露》系列"同一张图、多组配色"的表现手法
    // ------------------------------------------------------------------
    reg        S_tick_d;
    reg [2:0]  S_pal_div;
    reg [1:0]  S_pal;
    reg [1:0]  S_pal_csi1, S_pal_csi2;

    wire S_tick_evt = S_tick_500ms ^ S_tick_d;

    always @(posedge I_sys_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_tick_d  <= 1'b0;
            S_pal_div <= 3'd0;
            S_pal     <= 2'd0;
        end
        else begin
            S_tick_d <= S_tick_500ms;
            if (S_tick_evt) begin
                if (S_pal_div == 3'd5) begin
                    S_pal_div <= 3'd0;
                    S_pal     <= S_pal + 2'd1;
                end
                else
                    S_pal_div <= S_pal_div + 3'd1;
            end
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_pal_csi1 <= 2'd0;
            S_pal_csi2 <= 2'd0;
        end
        else begin
            S_pal_csi1 <= S_pal;
            S_pal_csi2 <= S_pal_csi1;
        end
    end

    // ------------------------------------------------------------------
    // 处理延迟可视化（进阶 3）：以毫秒为单位显示
    //   帧缓冲深度 = 写指针领先读指针的槽数（0~3 帧）
    //   延迟 ms = 帧数 x 1000 / 实测帧率，每 0.5 s 用串行除法器算一次
    // ------------------------------------------------------------------
    reg [1:0] S_lat_frames;                 // DDR 时钟域
    reg [1:0] S_lat_sys1, S_lat_sys2;       // 同步到 50 MHz 域
    reg [9:0] S_fps_sys1, S_fps_sys2;       // 帧率同步到 50 MHz 域
    reg [9:0] S_lat_ms;                     // 换算结果
    reg [9:0] S_lat_pix1, S_lat_pix2;       // 同步到像素域

    // 写指针在帧起始先递增、再用旧值选槽，因此"刚写入的槽"是 wp-1；
    // 读槽比它落后 (wp-1)-rp 格，即真正的缓冲帧数。
    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if (!S_rst_n)
            S_lat_frames <= 2'd0;
        else
            S_lat_frames <= (S_video_in_wp - S_video_out_rp - 2'd1) & 2'b11;
    end

    // ---- 串行除法器：dividend = 帧数*1000，divisor = 帧率 ----
    reg [11:0] S_div_dend;
    reg [9:0]  S_div_dsor;
    reg [11:0] S_div_rem;
    reg [9:0]  S_div_quo;
    reg [4:0]  S_div_cnt;
    reg        S_div_run;
    reg        S_tick_d2;

    wire S_tick_evt2 = S_tick_500ms ^ S_tick_d2;

    always @(posedge I_sys_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_lat_sys1 <= 2'd0;  S_lat_sys2 <= 2'd0;
            S_fps_sys1 <= 10'd0; S_fps_sys2 <= 10'd0;
            S_tick_d2  <= 1'b0;
            S_div_dend <= 12'd0; S_div_dsor <= 10'd1;
            S_div_rem  <= 12'd0; S_div_quo  <= 10'd0;
            S_div_cnt  <= 5'd0;  S_div_run  <= 1'b0;
            S_lat_ms   <= 10'd0;
        end
        else begin
            S_lat_sys1 <= S_lat_frames;
            S_lat_sys2 <= S_lat_sys1;
            S_fps_sys1 <= S_fps_latched;
            S_fps_sys2 <= S_fps_sys1;
            S_tick_d2  <= S_tick_500ms;

            if (S_tick_evt2 && !S_div_run) begin
                // 启动一次除法：ms = frames * 1000 / fps
                S_div_dend <= {2'b00, S_lat_sys2} * 12'd1000;
                S_div_dsor <= (S_fps_sys2 == 10'd0) ? 10'd1 : S_fps_sys2;
                S_div_rem  <= 12'd0;
                S_div_quo  <= 10'd0;
                S_div_cnt  <= 5'd12;
                S_div_run  <= 1'b1;
            end
            else if (S_div_run) begin
                if (S_div_cnt != 5'd0) begin
                    if ({S_div_rem[10:0], S_div_dend[11]} >= {2'b00, S_div_dsor}) begin
                        S_div_rem <= {S_div_rem[10:0], S_div_dend[11]} - {2'b00, S_div_dsor};
                        S_div_quo <= {S_div_quo[8:0], 1'b1};
                    end
                    else begin
                        S_div_rem <= {S_div_rem[10:0], S_div_dend[11]};
                        S_div_quo <= {S_div_quo[8:0], 1'b0};
                    end
                    S_div_dend <= {S_div_dend[10:0], 1'b0};
                    S_div_cnt  <= S_div_cnt - 5'd1;
                end
                else begin
                    S_div_run <= 1'b0;
                    S_lat_ms  <= (S_div_quo > 10'd99) ? 10'd99 : S_div_quo;
                end
            end
        end
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_lat_pix1 <= 10'd0;
            S_lat_pix2 <= 10'd0;
        end
        else begin
            S_lat_pix1 <= S_lat_ms;
            S_lat_pix2 <= S_lat_pix1;
        end
    end

    // ------------------------------------------------------------------
    // 检测结果跨到像素域：先两级同步，再在每个显示帧起始整体锁存，
    // 保证画框用到的 min/max 来自同一帧，不会出现半新半旧的框
    // ------------------------------------------------------------------
    reg [11:0] dmx_s1, dmxx_s1, dmy_s1, dmyx_s1;
    reg [11:0] dmx_s2, dmxx_s2, dmy_s2, dmyx_s2;
    reg        dv_s1,   dv_s2;
    reg        dc_s1,   dc_s2;
    reg [9:0]  dn_s1,   dn_s2;
    reg [7:0]  ar_s1, ar_s2, ag_s1, ag_s2, ab_s1, ab_s2;
    reg [9:0]  gr_s1, gr_s2, gb_s1, gb_s2;

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            dmx_s1  <= 12'd0;  dmx_s2  <= 12'd0;
            dmxx_s1 <= 12'd0;  dmxx_s2 <= 12'd0;
            dmy_s1  <= 12'd0;  dmy_s2  <= 12'd0;
            dmyx_s1 <= 12'd0;  dmyx_s2 <= 12'd0;
            dv_s1   <= 1'b0;   dv_s2   <= 1'b0;
            dc_s1   <= 1'b0;   dc_s2   <= 1'b0;
            dn_s1   <= 10'd0;  dn_s2   <= 10'd0;
            ar_s1 <= 8'd0; ar_s2 <= 8'd0;
            ag_s1 <= 8'd0; ag_s2 <= 8'd0;
            ab_s1 <= 8'd0; ab_s2 <= 8'd0;
            gr_s1 <= 10'd0; gr_s2 <= 10'd0;
            gb_s1 <= 10'd0; gb_s2 <= 10'd0;
            S_gr100_p <= 10'd0; S_gb100_p <= 10'd0;
            S_ae_pix   <= 16'd0; S_ag_pix  <= 16'd0;
            S_avg_r_p <= 8'd0; S_avg_g_p <= 8'd0; S_avg_b_p <= 8'd0;
            S_det_minx_p <= 12'd0; S_det_maxx_p <= 12'd0;
            S_det_miny_p <= 12'd0; S_det_maxy_p <= 12'd0;
            S_det_cx_p   <= 12'd0; S_det_cy_p   <= 12'd0;
            S_det_green_p <= 1'b0;
            S_det_valid_p <= 1'b0;
            S_det_cand_p  <= 1'b0;
            S_det_cnt_p   <= 10'd0;
            S_det_flags_p <= 8'd0;
            S_cal_st_p    <= 2'd0;
            S_rc_minx <= 12'd0; S_rc_maxx <= 12'd0;
            S_rc_miny <= 12'd0; S_rc_maxy <= 12'd0;
            S_rc_cx   <= 12'd0; S_rc_cy   <= 12'd0;
            S_rc_green<= 1'b0;
            S_rc_valid<= 1'b0;  S_rc_cand <= 1'b0;
            S_rc_cnt <= 10'd0;  S_rc_cal  <= 2'd0;
            S_rc_flags <= 8'd0;
            S_st1 <= 1'b0; S_st2 <= 1'b0; S_st3 <= 1'b0;
            S_ack_tgl <= 1'b0;
            S_rc_gr <= 10'd0; S_rc_gb <= 10'd0;
            S_rc_ar <= 8'd0; S_rc_ag <= 8'd0; S_rc_ab <= 8'd0;
        end
        else begin
            dmx_s1  <= S_det_minx;  dmx_s2  <= dmx_s1;
            dmxx_s1 <= S_det_maxx;  dmxx_s2 <= dmxx_s1;
            dmy_s1  <= S_det_miny;  dmy_s2  <= dmy_s1;
            dmyx_s1 <= S_det_maxy;  dmyx_s2 <= dmyx_s1;
            dv_s1   <= S_det_valid; dv_s2   <= dv_s1;
            dc_s1   <= S_det_cand;  dc_s2   <= dc_s1;
            dn_s1   <= S_det_cnt;   dn_s2   <= dn_s1;
            ar_s1 <= S_avg_r; ar_s2 <= ar_s1;
            ag_s1 <= S_avg_g; ag_s2 <= ag_s1;
            ab_s1 <= S_avg_b; ab_s2 <= ab_s1;
            gr_s1 <= S_gr100; gr_s2 <= gr_s1;
            gb_s1 <= S_gb100; gb_s2 <= gb_s1;

            // 整组接收：只在 tgl 边沿整段拷入
            S_st1 <= S_sn_tgl;
            S_st2 <= S_st1;
            S_st3 <= S_st2;
            if (S_snap_evt) begin
                // 一次事务把整组一起搬走，并立刻回 ack
                S_rc_minx  <= S_sn_minx;
                S_rc_maxx  <= S_sn_maxx;
                S_rc_miny  <= S_sn_miny;
                S_rc_maxy  <= S_sn_maxy;
                S_rc_cx    <= S_sn_cx;
                S_rc_cy    <= S_sn_cy;
                S_rc_green <= S_sn_green;
                S_rc_valid <= S_sn_valid;
                S_rc_cand  <= S_sn_cand;
                S_rc_cnt   <= S_sn_cnt;
                S_rc_flags <= S_sn_flags;
                S_rc_cal   <= S_sn_cal;
                S_rc_gr    <= S_sn_gr;
                S_rc_gb    <= S_sn_gb;
                S_rc_ar    <= S_sn_ar;
                S_rc_ag    <= S_sn_ag;
                S_rc_ab    <= S_sn_ab;
                S_ack_tgl  <= ~S_ack_tgl;
            end

            if (S_hdmi_user) begin
                // 显示帧边界提交：整帧用同一份快照，不会撕裂
                S_gr100_p <= S_rc_gr;
                S_gb100_p <= S_rc_gb;
                S_avg_r_p <= S_rc_ar;
                S_avg_g_p <= S_rc_ag;
                S_avg_b_p <= S_rc_ab;
                S_det_minx_p  <= S_rc_minx;
                S_det_maxx_p  <= S_rc_maxx;
                S_det_miny_p  <= S_rc_miny;
                S_det_maxy_p  <= S_rc_maxy;
                S_det_cx_p    <= S_rc_cx;
                S_det_cy_p    <= S_rc_cy;
                S_det_green_p <= S_rc_green;
                S_det_valid_p <= S_rc_valid;
                S_det_cand_p  <= S_rc_cand;
                S_det_cnt_p   <= S_rc_cnt;
                S_det_flags_p <= S_rc_flags;    // 与 bbox/valid/cand 同一快照组
                S_cal_st_p    <= S_rc_cal;
                // AE 状态来自 CSI 域中继，显示帧边界整组锁存
                S_ae_pix      <= S_ae_csi;
                S_ag_pix      <= S_ag_csi;
            end
        end
    end

    // ---- KEY1 长按 1.5 s 触发一次白纸标定（50 MHz 域）----
    //   短按仍是 RAW（原下降沿逻辑不变）；长按只发一次，松手才允许下次。
    localparam [31:0] K1_LONG = 32'd75_000_000;   // 50 MHz x 1.5 s

    always @(posedge I_sys_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_k1_cnt   <= 32'd0;
            S_k1_fired <= 1'b0;
            S_cal_tgl  <= 1'b0;
        end
        else if (!S_btn_st[0]) begin
            if (S_k1_cnt != K1_LONG)
                S_k1_cnt <= S_k1_cnt + 32'd1;
            if ((S_k1_cnt == (K1_LONG - 32'd1)) && !S_k1_fired) begin
                S_cal_tgl  <= ~S_cal_tgl;
                S_k1_fired <= 1'b1;
            end
        end
        else begin
            S_k1_cnt   <= 32'd0;
            S_k1_fired <= 1'b0;
        end
    end

    // ---- K1 电平跨到 CSI 域：I_cal_mode 与 I_cal_trig 必须同源同拍 ----
    cal_mode_cdc u_cal_mode_cdc (
        .I_csi_clk   (S_csi_rx_clk),
        .I_csi_rst_n (S_rst_n),
        .I_cal_tgl   (S_cal_tgl),
        .O_mode      (S_cal_t2),
        .O_trig      (S_cal_trig)
    );

    // ---- CSI 域：标定请求同步 + 检测结果整组快照 ----
    //   数据先稳定、再翻 tgl；接收侧见到沿时整组已稳，比逐位两拍可靠。
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_ft1 <= 1'b0; S_ft2 <= 1'b0; S_ft3 <= 1'b0;
            S_sn_minx <= 12'd0; S_sn_maxx <= 12'd0;
            S_sn_miny <= 12'd0; S_sn_maxy <= 12'd0;
            S_sn_cx   <= 12'd0; S_sn_cy   <= 12'd0;
            S_sn_green<= 1'b0;
            S_sn_valid <= 1'b0; S_sn_cand <= 1'b0;
            S_sn_cnt <= 10'd0;  S_sn_cal <= 2'd0;  S_sn_tgl <= 1'b0;
            S_sn_flags <= 8'd0;
            S_cal_st_l <= 2'd0;
            S_stg_vld <= 1'b0;
            S_ack1 <= 1'b0;     S_ack2 <= 1'b0;
            S_stg_minx <= 12'd0; S_stg_maxx <= 12'd0;
            S_stg_miny <= 12'd0; S_stg_maxy <= 12'd0;
            S_stg_cx   <= 12'd0; S_stg_cy   <= 12'd0;
            S_stg_green<= 1'b0;
            S_stg_valid <= 1'b0; S_stg_cand <= 1'b0;
            S_stg_cnt <= 10'd0;  S_stg_cal <= 2'd0;
            S_stg_flags <= 8'd0;
            S_stg_gr <= 10'd0;   S_stg_gb <= 10'd0;
            S_stg_ar <= 8'd0;    S_stg_ag <= 8'd0;  S_stg_ab <= 8'd0;
            S_sn_gr <= 10'd0;    S_sn_gb <= 10'd0;
            S_sn_ar <= 8'd0;     S_sn_ag <= 8'd0;   S_sn_ab <= 8'd0;
        end
        else begin
            S_ft1 <= S_isp_frm_tgl;
            S_ft2 <= S_ft1;
            S_ft3 <= S_ft2;
            S_cal_st_l <= S_cal_st;
            S_ack1 <= S_ack_tgl;
            S_ack2 <= S_ack1;
            // ---- 请求/应答 ----
            //   空闲条件用【真实的请求电平】S_sn_tgl 与收到的 ack 比较，
            //   不是另造一个从不翻转的 S_sn_req（那样首包之后条件恒假，
            //   第二包永远发不出去）。
            //   优先级：本拍若既有新事件又要发送，发送搬走的是【旧】staging
            //   （非阻塞读），新事件继续留在 staging 并把 pending 保持为 1，
            //   所以后面的 S_stg_vld <= S_new_evt 必须写在发送分支之后。
            if (S_frm_evt || (S_cal_st != S_cal_st_l)) begin
                S_stg_minx  <= S_det_minx;
                S_stg_maxx  <= S_det_maxx;
                S_stg_miny  <= S_det_miny;
                S_stg_maxy  <= S_det_maxy;
                S_stg_cx    <= S_det_cx;
                S_stg_cy    <= S_det_cy;
                S_stg_green <= S_det_green;
                S_stg_valid <= S_det_valid;
                S_stg_cand  <= S_det_cand;
                S_stg_cnt   <= S_det_cnt;
                S_stg_flags <= S_det_flags;
                S_stg_cal   <= S_cal_st;
                S_stg_gr    <= S_gr100;      // awb 实际生效的增益
                S_stg_gb    <= S_gb100;
                S_stg_ar    <= S_avg_r;      // 中心 ROI 均值
                S_stg_ag    <= S_avg_g;
                S_stg_ab    <= S_avg_b;
                S_stg_vld   <= 1'b1;
            end
            // 上一次事务已被 ack 才发下一组，保证 payload 保持窗口完整
            if (S_stg_vld && (S_sn_tgl == S_ack2)) begin
                S_sn_minx  <= S_stg_minx;
                S_sn_maxx  <= S_stg_maxx;
                S_sn_miny  <= S_stg_miny;
                S_sn_maxy  <= S_stg_maxy;
                S_sn_cx    <= S_stg_cx;
                S_sn_cy    <= S_stg_cy;
                S_sn_green <= S_stg_green;
                S_sn_valid <= S_stg_valid;
                S_sn_cand  <= S_stg_cand;
                S_sn_cnt   <= S_stg_cnt;
                S_sn_flags <= S_stg_flags;
                S_sn_cal   <= S_stg_cal;
                S_sn_gr    <= S_stg_gr;
                S_sn_gb    <= S_stg_gb;
                S_sn_ar    <= S_stg_ar;
                S_sn_ag    <= S_stg_ag;
                S_sn_ab    <= S_stg_ab;
                S_sn_tgl   <= ~S_sn_tgl;
                // 同拍若又来了新事件：上面已把新值写进 staging，
                // 这里必须保持 pending=1，否则断流后的最后一个 ERR 会被吞掉。
                S_stg_vld  <= (S_frm_evt || (S_cal_st != S_cal_st_l));
            end
        end
    end

    // ------------------------------------------------------------------
    // 帧率统计：以 ISP 输出帧起始(I_tuser)计帧，每 0.5 秒锁存一次
    // 0.5 秒内帧数 x2 即为帧率；计数在 CSI 时钟域，
    // 经 toggle 握手搬到 HDMI 像素时钟域显示
    // ------------------------------------------------------------------
    reg [2:0]  S_tick_500ms_sync;
    reg [9:0]  S_fps_cnt;
    reg        S_fps_toggle;
    reg [2:0]  S_fps_tog_sync;
    reg [9:0]  S_fps_value;

    // 电平翻转跨域后按边沿检测，每个 0.5 s 恰好给出一次事件
    wire S_tick_500ms_csi = S_tick_500ms_sync[1] ^ S_tick_500ms_sync[2];

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_tick_500ms_sync <= 3'b000;
            S_fps_cnt         <= 10'd0;
            S_fps_latched     <= 10'd0;
            S_fps_toggle      <= 1'b0;
        end
        else begin
            S_tick_500ms_sync <= {S_tick_500ms_sync[1:0], S_tick_500ms};

            if (S_tick_500ms_csi) begin
                // 0.5 秒帧数 x2 = 帧率，超过 999 截顶
                if (S_fps_cnt > 10'd499)
                    S_fps_latched <= 10'd999;
                else
                    S_fps_latched <= {S_fps_cnt[8:0], 1'b0};
                S_fps_cnt    <= 10'd0;
                S_fps_toggle <= ~S_fps_toggle;
            end
            else if (S_ISP_O_tuser) begin
                if (S_fps_cnt != 10'd999)
                    S_fps_cnt <= S_fps_cnt + 10'd1;
            end
        end
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_fps_tog_sync <= 3'b000;
            S_fps_value    <= 10'd0;
        end
        else begin
            S_fps_tog_sync <= {S_fps_tog_sync[1:0], S_fps_toggle};
            if (S_fps_tog_sync[1] ^ S_fps_tog_sync[2])
                S_fps_value <= S_fps_latched;
        end
    end
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    // A RAW10 line contains 1280 * 10 / 8 = 1600 bytes.  The CSI parser
    // reports the number of 16-bit lane words, therefore the expected value
    // is 1600 / 2 = 800 = 16'h0320.  Count one long packet whenever that
    // decoded length changes from zero to non-zero.
    assign S_csi_packet_start = (S_csi_data_length != 16'd0) &&
                                !S_csi_length_active_d;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_csi_length_active_d      <= 1'b0;
            S_mipi_line_count          <= 11'd0;
            S_mipi_length_error        <= 1'b0;
            S_mipi_lane_error          <= 1'b0;
            S_mipi_frame_seen          <= 1'b0;
            S_mipi_frame_done          <= 1'b0;
            S_mipi_last_length_error   <= 1'b0;
            S_mipi_last_line_error     <= 1'b0;
            S_mipi_last_lane_error     <= 1'b0;
        end
        else begin
            S_csi_length_active_d <= (S_csi_data_length != 16'd0);

            if(S_csi_frame_start) begin
                S_mipi_line_count   <= 11'd0;
                S_mipi_length_error <= 1'b0;
                S_mipi_lane_error   <= 1'b0;
                S_mipi_frame_seen   <= 1'b1;
            end
            else begin
                if(S_csi_packet_start) begin
                    S_mipi_line_count <= S_mipi_line_count + 11'd1;
                    if(S_csi_data_length != 16'h0320)
                        S_mipi_length_error <= 1'b1;
                end

                if(S_lane_error_any)
                    S_mipi_lane_error <= 1'b1;
            end

            // Publish only a completed frame, so the HDMI-domain diagnostic
            // sees a stable result rather than counters changing every line.
            if(S_csi_frame_end) begin
                S_mipi_frame_done        <= 1'b1;
                S_mipi_last_length_error <= S_mipi_length_error;
                S_mipi_last_line_error   <= (S_mipi_line_count != 11'd720);
                S_mipi_last_lane_error   <= S_mipi_lane_error |
                                            S_lane_error_any;
            end
        end
    end

    // Diagnostic result codes consumed by hdmi_mixer DEBUG_MODE=4:
    // 0=config incomplete, 1=no HS traffic, 2=no completed CSI frame,
    // 3=lane error, 4=wrong RAW10 line length, 5=wrong lines/frame,
    // 6=the observed MIPI frame structure is correct, 7=I2C error.
    // I2C error has the highest priority because O_cfg_done only indicates
    // that the write sequencer reached its end; it does not prove that every
    // sensor register write was acknowledged successfully.
    assign S_mipi_diag_status_async =
        S_cam_i2c_error              ? 4'd7 :
        !S_cam_cfg_done              ? 4'd0 :
        !S_dbg_hs_seen               ? 4'd1 :
        !(S_mipi_frame_seen &&
          S_mipi_frame_done)         ? 4'd2 :
        S_mipi_last_lane_error       ? 4'd3 :
        S_mipi_last_length_error     ? 4'd4 :
        S_mipi_last_line_error       ? 4'd5 : 4'd6;

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_mipi_diag_status_sync1 <= 4'd0;
            S_mipi_diag_status_sync2 <= 4'd0;
        end
        else begin
            S_mipi_diag_status_sync1 <= S_mipi_diag_status_async;
            S_mipi_diag_status_sync2 <= S_mipi_diag_status_sync1;
        end
    end

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = S_mipi_diag_status_sync2;
	
	// J9 pin 37 is connected to the module's PWDNB/XSHUTDN input.
	// Hold the sensor in power-down after PLL lock, then allow its internal
	// power-up sequence to finish before the first I2C transaction.
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  // ------------------------------------------------------------------
  // AE 测光跨时钟域
  //   ISP 在像素域（约 103MHz），AE 控制器在 24MHz 域。
  //   O_lum 每帧才变一次（16.7ms），数据保持时间足够长，所以用
  //   「电平翻转握手 + 数据打两拍」：检测到翻转沿时采数据，
  //   此刻数据早已稳定，不存在采到半新半旧的问题。
  // ------------------------------------------------------------------
  reg        S_lum_t1, S_lum_t2, S_lum_t3;
  reg  [7:0] S_lum_d1, S_lum_d2;
  wire       S_lum_upd = S_lum_t2 ^ S_lum_t3;

  always @(posedge S_24m_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
      S_lum_t1 <= 1'b0; S_lum_t2 <= 1'b0; S_lum_t3 <= 1'b0;
      S_lum_d1 <= 8'd0; S_lum_d2 <= 8'd0;
    end
    else begin
      S_lum_t1 <= S_lum_tgl;
      S_lum_t2 <= S_lum_t1;
      S_lum_t3 <= S_lum_t2;
      S_lum_d1 <= S_lum;
      S_lum_d2 <= S_lum_d1;
    end
  end

  // AE 曝光/增益状态跨到 HDMI 像素域（OSD 第 6 行板测用）。
  //   源域：AE 值在每次 I2C 写完成后（O_ae_cfg_done 上升）已稳定；
  //   这里只翻转一个电平，像素域 2FF 后按边沿整组采样，避免多 bit 撕裂。
  reg S_ae_done_d, S_ae_tgl;
  always @(posedge S_24m_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
      S_ae_done_d <= 1'b0;
      S_ae_tgl    <= 1'b0;
    end
    else begin
      S_ae_done_d <= S_ae_cfg_done;
      if (S_ae_cfg_done && !S_ae_done_d)
        S_ae_tgl <= ~S_ae_tgl;
    end
  end

  // AE 状态经 CSI 域中继：24MHz 与 CSI 属不同异步时钟组，CSI 与 HDMI
  // 也不同组，因此两个跨域都不按同源同步路径做 STA（避免 24MHz→HDMI
  // 同组直采的 −0.294ns 违例）。数据在 toggle 前已稳定，边沿整组采样。
  reg [2:0]  S_ae_csync;
  always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
      S_ae_csync <= 3'd0;
      S_ae_csi   <= 16'd0;
      S_ag_csi   <= 16'd0;
    end
    else begin
      S_ae_csync <= {S_ae_csync[1:0], S_ae_tgl};
      if (S_ae_csync[2] ^ S_ae_csync[1]) begin
        S_ae_csi <= S_ae;
        S_ag_csi <= S_ag;
      end
    end
  end

  // AE 闭环控制器（替代原厂按键手动 AE，模块在 uiisp_beta/ae_set.v 内）
  ae_ctrl #(
      // 温和提亮：目标 64→80（约 36% 满量程），增益上限 4x→6x；
      // 曝光优先、死区 6、holdoff 4 帧，控制过曝/闪烁/噪声。
      // SC520GainTbl: 16=1x, 48=2x, 80=4x, 96=6x（细调仍为 1x）。
      .AE_INIT   (16'd1991),
      .AG_INIT   (16'd16  ),
      .AG_MAX    (16'd96  ),
      .AE_TARGET (8'd80   )
  ) u_ae_ctrl (
      .I_clk          (S_24m_clk      ),
      .I_rst          (~S_rst_n       ),
      .I_cam_cfg_done (S_cam_cfg_done ),
      .I_ae_cfg_done  (S_ae_cfg_done  ),
      .I_lum          (S_lum_d2       ),
      .I_lum_upd      (S_lum_upd      ),
      .O_ae_req       (S_ae_req       ),
      .O_ae           (S_ae           ),
      .O_ag           (S_ag           )
  );
  
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        ),
.O_data_length         ( S_csi_data_length )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
)
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// SC520CS原厂HD模式已经直接输出1280x720 RAW10，无需FPGA窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_ccl_src_clk   (I_sys_clk),
.I_cal_mode      (S_cal_t2),   // CDC: 必须用 CSI 域已同步电平，不能用 sys 域 S_cal_tgl
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.I_mode          (S_mode_csi2),
.I_pal           (S_pal_csi2),
.O_det_minx      (S_det_minx),
.O_det_maxx      (S_det_maxx),
.O_det_miny      (S_det_miny),
.O_det_maxy      (S_det_maxy),
.O_det_cx        (S_det_cx),
.O_det_cy        (S_det_cy),
.O_det_green     (S_det_green),
.O_det_valid     (S_det_valid),
.O_det_fresh_tgl (S_det_fresh_tgl),
.O_det_cand      (S_det_cand),
.O_det_cnt       (S_det_cnt),
.O_det_flags     (S_det_flags),
.O_red_minx      (S_red_minx),
.O_red_maxx      (S_red_maxx),
.O_red_miny      (S_red_miny),
.O_red_maxy      (S_red_maxy),
.O_red_valid     (S_red_valid),
.O_red_cand      (S_red_cand),
.O_red_count     (S_red_count),
.O_red_update    (S_red_update),
.O_rgb_r         (S_avg_r),
.O_rgb_g         (S_avg_g),
.O_rgb_b         (S_avg_b),
.O_lum           (S_lum),
.O_lum_tgl       (S_lum_tgl),
.O_gr100         (S_gr100),
.O_gb100         (S_gb100),
.I_cal_trig      (S_cal_trig),
.O_cal_st        (S_cal_st),
.O_frm_tgl       (S_isp_frm_tgl),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );

    red_ball_snapshot_cdc u_red_snapshot (
        .I_src_clk(S_csi_rx_clk), .I_src_rst_n(S_rst_n),
        .I_src_update(S_red_update),
        .I_src_data({S_red_valid,S_red_cand,S_red_count,
                     S_red_minx,S_red_maxx,S_red_miny,S_red_maxy}),
        .I_dst_clk(S_hdmi_pixel_clk), .I_dst_rst_n(S_hdmi_rst_n),
        .I_dst_frame(S_hdmi_user), .O_dst_data(S_red_pixel_snapshot)
    );


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_in_wp        ( S_video_in_wp         ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    // AE 状态在显示帧边界锁存（值来自 CSI 域中继，见 S_ae_csi）

    // ---- 两只老虎单声部无源蜂鸣器（50 MHz 域；新鲜检测 toggle 跨域）----
    // 触发/看门狗只认“真实检测通过”事件：O_det_fresh_tgl 每个核心 fresh
    // pass 翻转一次（EVT: tracker det_pass；CCL: det_result O_pass）。绿框
    // hold（4 相机帧）和跳帧都不翻转它，所以 1 s 未识别从最后一次真实检测
    // 算起，而不是绿框熄灭后再计 1 s。
    // 播放中：重新检测只清零看门狗，不重启；连续 1 s 无真实检测立即停、
    // 静音并重臂；再次检测从第一个音开始。播完目标仍在不复播，消失 1 s
    // 后再检测才重播。
    isp_two_tigers #(
        .CLK_HZ     (50_000_000),
        .TIMEOUT_MS (1000),
        .GAP_MS     (20)
    ) u_two_tigers (
        .clk     (I_sys_clk),
        .rst_n   (S_rst_n),
        .det_tgl (S_det_fresh_tgl),
        .buzzer  (O_buzzer)
    );

    // ---- 运行时长 DD:HH:MM:SS：50 MHz 固定分频 + 整组原子 CDC ----
    wire [7:0] S_rt_day, S_rt_hour, S_rt_min, S_rt_sec;
    isp_rtc_cdc #(.CLK_DIV(50_000_000)) u_rtc (
        .src_clk   (I_sys_clk),
        .src_rst_n (S_rst_n),
        .dst_clk   (S_hdmi_pixel_clk),
        .dst_rst_n (S_hdmi_rst_n),
        .day       (S_rt_day),
        .hour      (S_rt_hour),
        .minute    (S_rt_min),
        .second    (S_rt_sec)
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        // 正常画面显示（含 ISP 边缘检测结果）。
        // 需要 MIPI 链路诊断时改为 4，显示全屏彩色状态页。
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_fps_value     ( S_fps_value      ),
        .I_mode          ( S_mode_pix2      ),
        .I_lat           ( S_lat_pix2       ),
        .I_det_minx      ( S_det_minx_p     ),
        .I_det_maxx      ( S_det_maxx_p     ),
        .I_det_miny      ( S_det_miny_p     ),
        .I_det_maxy      ( S_det_maxy_p     ),
        .I_det_cx        ( S_det_cx_p       ),
        .I_det_cy        ( S_det_cy_p       ),
        .I_det_green     ( S_det_green_p    ),
        .I_det_valid     ( S_det_valid_p    ),
        .I_rt_day        ( S_rt_day         ),
        .I_rt_hour       ( S_rt_hour        ),
        .I_rt_min        ( S_rt_min         ),
        .I_rt_sec        ( S_rt_sec         ),
        .I_det_cand      ( S_det_cand_p     ),
        .I_det_cnt       ( S_det_cnt_p      ),
        .I_det_flags     ( S_det_flags_p    ),
        .I_red_valid     ( S_red_pixel_snapshot[59]    ),
        .I_red_minx      ( S_red_pixel_snapshot[47:36] ),
        .I_red_maxx      ( S_red_pixel_snapshot[35:24] ),
        .I_red_miny      ( S_red_pixel_snapshot[23:12] ),
        .I_red_maxy      ( S_red_pixel_snapshot[11:0]  ),
        .I_avg_r         ( S_avg_r_p        ),
        .I_avg_g         ( S_avg_g_p        ),
        .I_avg_b         ( S_avg_b_p        ),
        .I_gr100         ( S_gr100_p        ),
        .I_gb100         ( S_gb100_p        ),
        .I_ae            ( S_ae_pix         ),
        .I_ag            ( S_ag_pix         ),
        .I_cal_st        ( S_cal_st_p       ),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );

    
endmodule
