module hdmi_mixer #(
    parameter H_OFFSET = 0,
    parameter V_OFFSET = 0,
    parameter IMG_WIDTH = 1600,
    parameter IMG_HEIGHT = 900,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,
    input wire[9:0]   I_fps_value,      // 实时帧率，由顶层每秒锁存一次
    input wire[1:0]   I_mode,           // 当前算法：0=原图 1=边缘 2=灰度 3=波普
    input wire[9:0]   I_lat,            // 端到端延迟（毫秒）
    input wire[11:0]  I_det_minx,       // 黄色目标外接框（像素域）
    input wire[11:0]  I_det_maxx,
    input wire[11:0]  I_det_miny,
    input wire[11:0]  I_det_maxy,
    // Circle centre from the same result snapshot as the bbox (image pixels).
    input wire[11:0]  I_det_cx,
    input wire[11:0]  I_det_cy,
    input wire        I_det_green,   // tracked interior is green -> add GREEN
    input wire        I_det_valid,
    input wire [7:0]  I_rt_day,      // runtime DD:HH:MM:SS (atomic CDC group)
    input wire [7:0]  I_rt_hour,
    input wire [7:0]  I_rt_min,
    input wire [7:0]  I_rt_sec,
    input wire        I_det_cand,       // 颜色+面积通过但形状未过（调试用）
    input wire[9:0]   I_det_cnt,        // 命中像素数/16（调试显示）
    // 圆形诊断。EVT_CIRCLE=0: {ovf,live,chord,rows,asp,size,pass,valid};
    // EVT_CIRCLE=1: {ovf,live,drop,0,0,0,pass,valid} (isp_evt_hud_map)。
    input wire[7:0]   I_det_flags,
    input wire        I_red_valid,
    input wire[11:0]  I_red_minx, I_red_maxx, I_red_miny, I_red_maxy,
    input wire[7:0]   I_avg_r,          // 中心区域平均色（调试显示）
    input wire[7:0]   I_avg_g,
    input wire[7:0]   I_avg_b,
    input wire[9:0]   I_gr100,          // 当前白平衡增益 x100（如 123 = 1.23x）
    input wire[9:0]   I_gb100,
    input wire[15:0]  I_ae,             // AE 曝光（半行单位，0..1991）
    input wire[15:0]  I_ag,             // AE 增益索引（16..96，6x 上限）
    input wire[1:0]   I_cal_st,          // 一键标定状态 0/1/2/3

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg        S_window_active_1d;
    reg        S_window_active_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    wire S_window_active;
    assign S_window_active = I_video_de &&
                             (S_x >= H_OFFSET) &&
                             (S_x < (H_OFFSET + IMG_WIDTH)) &&
                             (S_y >= V_OFFSET) &&
                             (S_y < (V_OFFSET + IMG_HEIGHT));

    // Read exactly IMG_WIDTH pixels on each of IMG_HEIGHT lines.  The HDMI
    // active area may be larger than the stored frame, in which case remaining
    // pixels are emitted as black borders without consuming DDR data.
    assign O_video_rd_en = S_window_active;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            if(I_video_user)
                S_y <= 12'd0;
            else if(I_video_de && I_video_last)
                S_y <= S_y + 12'd1;

            if(!I_video_de || I_video_last)
                S_x <= 12'd0;
            else
                S_x <= S_x + 12'd1;

            // 与输出数据（两级流水）对齐的坐标
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    // ==================================================================
    // FPS OSD —— 画面左上角显示 "FPS:<帧率>"
    //   7 个字符槽：F / P / S / : / 百位 / 十位 / 个位
    //   8x8 点阵 + 四周 1 像素描边，每字符占 10x10 字体像素
    //   字符区 210 x 30 像素，占 1280x720 有效区约 0.68%
    //   背景透明：只覆盖字形和描边像素，其余像素保持原始画面
    // ==================================================================
    localparam integer OSD_SCALE = 3;                       // 字体像素放大倍数
    localparam integer OSD_CELL  = 10;                      // 每字符 10x10 字体像素
    localparam integer OSD_ADV   = OSD_CELL * OSD_SCALE;    // 字距 30
    localparam integer OSD_W     = 8 * OSD_ADV;             // 240（最长一行 LAT:33MS）
    localparam integer OSD_ROWH  = OSD_CELL * OSD_SCALE;    // 30  单行高
    localparam integer OSD_H     = 8 * OSD_ROWH;            // 240 八行（第 6 行 AE， 第 7 行圆形诊断）
    localparam integer OSD_TOP   = 24;                      // 上边距
    localparam integer OSD_X0    = 24;                      // 左边距

    wire S_osd_box = (S_x_2d >= OSD_X0) && (S_x_2d < (OSD_X0 + OSD_W)) &&
                     (S_y_2d >= OSD_TOP) && (S_y_2d < (OSD_TOP + OSD_H));

    wire [11:0] S_osd_ox = S_x_2d - OSD_X0;
    wire [11:0] S_osd_oy = S_y_2d - OSD_TOP;

    // 第 0 行帧率，第 1 行算法，第 2 行延迟，第 3 行 R/G，第 4 行 B，
    // 第 5 行白平衡增益（R123B185），第 6 行 AE 曝光/增益（E1991G96），
    // 第 7 行圆形诊断（D<hex2>C<cnt>）。
    wire [2:0]  S_osd_ridx = (S_osd_oy >= (7*OSD_ROWH)) ? 3'd7 :
                             (S_osd_oy >= (6*OSD_ROWH)) ? 3'd6 :
                             (S_osd_oy >= (5*OSD_ROWH)) ? 3'd5 :
                             (S_osd_oy >= (4*OSD_ROWH)) ? 3'd4 :
                             (S_osd_oy >= (3*OSD_ROWH)) ? 3'd3 :
                             (S_osd_oy >= (2*OSD_ROWH)) ? 3'd2 :
                             (S_osd_oy >= OSD_ROWH)     ? 3'd1 : 3'd0;
    wire [11:0] S_osd_oyr  = S_osd_oy -
                             ((S_osd_ridx == 3'd7) ? 7*OSD_ROWH :
                              (S_osd_ridx == 3'd6) ? 6*OSD_ROWH :
                              (S_osd_ridx == 3'd5) ? 5*OSD_ROWH :
                              (S_osd_ridx == 3'd4) ? 4*OSD_ROWH :
                              (S_osd_ridx == 3'd3) ? 3*OSD_ROWH :
                              (S_osd_ridx == 3'd2) ? 2*OSD_ROWH :
                              (S_osd_ridx == 3'd1) ? OSD_ROWH   : 0);

    // ---- 定位到第几个字符槽，以及槽内横坐标 ----
    reg [2:0] S_osd_slot;
    reg [6:0] S_osd_colx;
    always @(*) begin
        if      (S_osd_ox >= 12'd210) begin S_osd_slot = 3'd7; S_osd_colx = S_osd_ox - 12'd210; end
        else if (S_osd_ox >= 12'd180) begin S_osd_slot = 3'd6; S_osd_colx = S_osd_ox - 12'd180; end
        else if (S_osd_ox >= 12'd150) begin S_osd_slot = 3'd5; S_osd_colx = S_osd_ox - 12'd150; end
        else if (S_osd_ox >= 12'd120) begin S_osd_slot = 3'd4; S_osd_colx = S_osd_ox - 12'd120; end
        else if (S_osd_ox >= 12'd90 ) begin S_osd_slot = 3'd3; S_osd_colx = S_osd_ox - 12'd90;  end
        else if (S_osd_ox >= 12'd60 ) begin S_osd_slot = 3'd2; S_osd_colx = S_osd_ox - 12'd60;  end
        else if (S_osd_ox >= 12'd30 ) begin S_osd_slot = 3'd1; S_osd_colx = S_osd_ox - 12'd30;  end
        else                            begin S_osd_slot = 3'd0; S_osd_colx = S_osd_ox[6:0];     end
    end

    // ---- 帧率拆成百/十/个位（比较-减法，避免综合出除法器）----
    reg [3:0] S_dig_h;
    always @(*) begin
        if      (I_fps_value >= 10'd900) S_dig_h = 4'd9;
        else if (I_fps_value >= 10'd800) S_dig_h = 4'd8;
        else if (I_fps_value >= 10'd700) S_dig_h = 4'd7;
        else if (I_fps_value >= 10'd600) S_dig_h = 4'd6;
        else if (I_fps_value >= 10'd500) S_dig_h = 4'd5;
        else if (I_fps_value >= 10'd400) S_dig_h = 4'd4;
        else if (I_fps_value >= 10'd300) S_dig_h = 4'd3;
        else if (I_fps_value >= 10'd200) S_dig_h = 4'd2;
        else if (I_fps_value >= 10'd100) S_dig_h = 4'd1;
        else                             S_dig_h = 4'd0;
    end

    wire [9:0] S_fps_ten_in = I_fps_value - ((S_dig_h << 6) + (S_dig_h << 5) + (S_dig_h << 2));

    reg [3:0] S_dig_t;
    always @(*) begin
        if      (S_fps_ten_in >= 10'd90) S_dig_t = 4'd9;
        else if (S_fps_ten_in >= 10'd80) S_dig_t = 4'd8;
        else if (S_fps_ten_in >= 10'd70) S_dig_t = 4'd7;
        else if (S_fps_ten_in >= 10'd60) S_dig_t = 4'd6;
        else if (S_fps_ten_in >= 10'd50) S_dig_t = 4'd5;
        else if (S_fps_ten_in >= 10'd40) S_dig_t = 4'd4;
        else if (S_fps_ten_in >= 10'd30) S_dig_t = 4'd3;
        else if (S_fps_ten_in >= 10'd20) S_dig_t = 4'd2;
        else if (S_fps_ten_in >= 10'd10) S_dig_t = 4'd1;
        else                             S_dig_t = 4'd0;
    end

    wire [9:0] S_fps_one_in = S_fps_ten_in - ((S_dig_t << 3) + (S_dig_t << 1));
    wire [3:0] S_dig_o      = S_fps_one_in[3:0];

    // ---- 中心区域平均色（0~255，全量程）拆百/十/个位 ----
    //   白平衡标定需要满量程读数：显示值就是该通道的真实平均值，
    //   直接可算增益 gain = 65536 * G / 通道均值。
    function [1:0] div100_255;
        input [7:0] x;
        begin
            if      (x >= 8'd200) div100_255 = 2'd2;
            else if (x >= 8'd100) div100_255 = 2'd1;
            else                  div100_255 = 2'd0;
        end
    endfunction

    function [3:0] div10_99;
        input [6:0] x;                       // 0~99
        begin
            if      (x >= 7'd90) div10_99 = 4'd9;
            else if (x >= 7'd80) div10_99 = 4'd8;
            else if (x >= 7'd70) div10_99 = 4'd7;
            else if (x >= 7'd60) div10_99 = 4'd6;
            else if (x >= 7'd50) div10_99 = 4'd5;
            else if (x >= 7'd40) div10_99 = 4'd4;
            else if (x >= 7'd30) div10_99 = 4'd3;
            else if (x >= 7'd20) div10_99 = 4'd2;
            else if (x >= 7'd10) div10_99 = 4'd1;
            else                 div10_99 = 4'd0;
        end
    endfunction

    // 4bit -> 字形编号：0..9 -> 4..13；A=17,B=27,C=28,D=20,E=19,F=0
    function [4:0] hex_glyph;
        input [3:0] v;
        begin
            case (v)
                4'd10: hex_glyph = 5'd17;   // A
                4'd11: hex_glyph = 5'd27;   // B
                4'd12: hex_glyph = 5'd28;   // C
                4'd13: hex_glyph = 5'd20;   // D
                4'd14: hex_glyph = 5'd19;   // E
                4'd15: hex_glyph = 5'd0;    // F
                default: hex_glyph = 5'd4 + {1'b0, v};
            endcase
        end
    endfunction

    // 100 = 64 + 32 + 4
    wire [1:0] S_ar_h = div100_255(I_avg_r);
    wire [1:0] S_ag_h = div100_255(I_avg_g);
    wire [1:0] S_ab_h = div100_255(I_avg_b);

    wire [7:0] S_ar_r = I_avg_r - (({6'd0,S_ar_h} << 6) + ({6'd0,S_ar_h} << 5) + ({6'd0,S_ar_h} << 2));
    wire [7:0] S_ag_r = I_avg_g - (({6'd0,S_ag_h} << 6) + ({6'd0,S_ag_h} << 5) + ({6'd0,S_ag_h} << 2));
    wire [7:0] S_ab_r = I_avg_b - (({6'd0,S_ab_h} << 6) + ({6'd0,S_ab_h} << 5) + ({6'd0,S_ab_h} << 2));

    wire [3:0] S_ar_t = div10_99(S_ar_r[6:0]);
    wire [3:0] S_ag_t = div10_99(S_ag_r[6:0]);
    wire [3:0] S_ab_t = div10_99(S_ab_r[6:0]);

    wire [6:0] S_ar_m = S_ar_r[6:0] - ({3'd0,S_ar_t} << 3) - ({3'd0,S_ar_t} << 1);
    wire [6:0] S_ag_m = S_ag_r[6:0] - ({3'd0,S_ag_t} << 3) - ({3'd0,S_ag_t} << 1);
    wire [6:0] S_ab_m = S_ab_r[6:0] - ({3'd0,S_ab_t} << 3) - ({3'd0,S_ab_t} << 1);

    wire [3:0] S_ar_o = S_ar_m[3:0];
    wire [3:0] S_ag_o = S_ag_m[3:0];
    wire [3:0] S_ab_o = S_ab_m[3:0];

    // ---- 白平衡增益 x100（0~260）拆百/十/个位 ----
    //   量程比平均值大（增益最大 2.60x -> 260），所以百位要用 9 位比较
    function [1:0] div100_299;
        input [8:0] x;
        begin
            if      (x >= 9'd200) div100_299 = 2'd2;
            else if (x >= 9'd100) div100_299 = 2'd1;
            else                  div100_299 = 2'd0;
        end
    endfunction

    wire [1:0] S_gr_h = div100_299(I_gr100[8:0]);
    wire [1:0] S_gb_h = div100_299(I_gb100[8:0]);

    wire [8:0] S_gr_r = I_gr100[8:0] - (({7'd0,S_gr_h} << 6) + ({7'd0,S_gr_h} << 5) + ({7'd0,S_gr_h} << 2));
    wire [8:0] S_gb_r = I_gb100[8:0] - (({7'd0,S_gb_h} << 6) + ({7'd0,S_gb_h} << 5) + ({7'd0,S_gb_h} << 2));

    wire [3:0] S_gr_t = div10_99(S_gr_r[6:0]);
    wire [3:0] S_gb_t = div10_99(S_gb_r[6:0]);

    wire [6:0] S_gr_m = S_gr_r[6:0] - ({3'd0,S_gr_t} << 3) - ({3'd0,S_gr_t} << 1);
    wire [6:0] S_gb_m = S_gb_r[6:0] - ({3'd0,S_gb_t} << 3) - ({3'd0,S_gb_t} << 1);

    wire [3:0] S_gr_o = S_gr_m[3:0];
    wire [3:0] S_gb_o = S_gb_m[3:0];

    // 增益每帧才变一次，译码结果打一拍完全不影响显示，
    // 但能把这一串比较/减法从 OSD 的组合链上摘下来（原来一路串到
    // 字形查找和输出 mux，是这次时序违例的帮凶之一）
    reg [1:0] S_gr_h_q; reg [3:0] S_gr_t_q, S_gr_o_q;
    reg [1:0] S_gb_h_q; reg [3:0] S_gb_t_q, S_gb_o_q;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_gr_h_q <= 2'd0; S_gr_t_q <= 4'd0; S_gr_o_q <= 4'd0;
            S_gb_h_q <= 2'd0; S_gb_t_q <= 4'd0; S_gb_o_q <= 4'd0;
        end
        else begin
            S_gr_h_q <= S_gr_h; S_gr_t_q <= S_gr_t; S_gr_o_q <= S_gr_o;
            S_gb_h_q <= S_gb_h; S_gb_t_q <= S_gb_t; S_gb_o_q <= S_gb_o;
        end
    end

    // ---- AE 曝光/增益拆位（第 6 行板测状态：E1991G96）----
    //   曝光 0..1991：千位 0/1，余下 0..999 拆百/十/个；
    //   增益索引 16..96 显示十/个两位（当前上限 96，不会到百位）。
    function [3:0] div100_999;
        input [9:0] x;
        begin
            if      (x >= 10'd900) div100_999 = 4'd9;
            else if (x >= 10'd800) div100_999 = 4'd8;
            else if (x >= 10'd700) div100_999 = 4'd7;
            else if (x >= 10'd600) div100_999 = 4'd6;
            else if (x >= 10'd500) div100_999 = 4'd5;
            else if (x >= 10'd400) div100_999 = 4'd4;
            else if (x >= 10'd300) div100_999 = 4'd3;
            else if (x >= 10'd200) div100_999 = 4'd2;
            else if (x >= 10'd100) div100_999 = 4'd1;
            else                   div100_999 = 4'd0;
        end
    endfunction

    wire [11:0] S_ae_v    = (I_ae[11:0] > 12'd1991) ? 12'd1991 : I_ae[11:0];
    wire        S_ae_th   = (S_ae_v >= 12'd1000);
    wire [11:0] S_ae_rem  = S_ae_th ? (S_ae_v - 12'd1000) : S_ae_v;
    wire [3:0]  S_ae_h    = div100_999(S_ae_rem[9:0]);
    wire [9:0]  S_ae_rem2 = S_ae_rem[9:0] -
                            (({6'd0,S_ae_h} << 6) + ({6'd0,S_ae_h} << 5) + ({6'd0,S_ae_h} << 2));
    wire [3:0]  S_ae_t    = div10_99(S_ae_rem2[6:0]);
    wire [6:0]  S_ae_o    = S_ae_rem2[6:0] - ({3'd0,S_ae_t} << 3) - ({3'd0,S_ae_t} << 1);

    wire [7:0]  S_aeg_v   = (I_ag > 16'd255) ? 8'd255 : I_ag[7:0];
    wire [1:0]  S_aeg_h   = div100_255(S_aeg_v);
    wire [7:0]  S_aeg_r   = S_aeg_v -
                            (({6'd0,S_aeg_h} << 6) + ({6'd0,S_aeg_h} << 5) + ({6'd0,S_aeg_h} << 2));
    wire [3:0]  S_aeg_t   = div10_99(S_aeg_r[6:0]);
    wire [6:0]  S_aeg_o   = S_aeg_r[6:0] - ({3'd0,S_aeg_t} << 3) - ({3'd0,S_aeg_t} << 1);

    reg        S_ae_th_q;
    reg [3:0]  S_ae_h_q, S_ae_t_q, S_ae_o_q;
    reg [3:0]  S_aeg_t_q, S_aeg_o_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_ae_th_q <= 1'b0;
            S_ae_h_q <= 4'd0; S_ae_t_q <= 4'd0; S_ae_o_q <= 4'd0;
            S_aeg_t_q <= 4'd0; S_aeg_o_q <= 4'd0;
        end
        else begin
            S_ae_th_q <= S_ae_th;
            S_ae_h_q  <= S_ae_h;  S_ae_t_q <= S_ae_t;  S_ae_o_q <= S_ae_o;
            S_aeg_t_q <= S_aeg_t; S_aeg_o_q <= S_aeg_o;
        end
    end

    // ---- 检测像素数（0~999）拆成三位，供调试显示 ----
    reg [3:0] S_dc_h;
    always @(*) begin
        if      (I_det_cnt >= 10'd900) S_dc_h = 4'd9;
        else if (I_det_cnt >= 10'd800) S_dc_h = 4'd8;
        else if (I_det_cnt >= 10'd700) S_dc_h = 4'd7;
        else if (I_det_cnt >= 10'd600) S_dc_h = 4'd6;
        else if (I_det_cnt >= 10'd500) S_dc_h = 4'd5;
        else if (I_det_cnt >= 10'd400) S_dc_h = 4'd4;
        else if (I_det_cnt >= 10'd300) S_dc_h = 4'd3;
        else if (I_det_cnt >= 10'd200) S_dc_h = 4'd2;
        else if (I_det_cnt >= 10'd100) S_dc_h = 4'd1;
        else                           S_dc_h = 4'd0;
    end

    wire [9:0] S_dc_ten_in = I_det_cnt - ((S_dc_h << 7) + (S_dc_h << 6) + (S_dc_h << 5) + (S_dc_h << 2));

    reg [3:0] S_dc_t;
    always @(*) begin
        if      (S_dc_ten_in >= 10'd90) S_dc_t = 4'd9;
        else if (S_dc_ten_in >= 10'd80) S_dc_t = 4'd8;
        else if (S_dc_ten_in >= 10'd70) S_dc_t = 4'd7;
        else if (S_dc_ten_in >= 10'd60) S_dc_t = 4'd6;
        else if (S_dc_ten_in >= 10'd50) S_dc_t = 4'd5;
        else if (S_dc_ten_in >= 10'd40) S_dc_t = 4'd4;
        else if (S_dc_ten_in >= 10'd30) S_dc_t = 4'd3;
        else if (S_dc_ten_in >= 10'd20) S_dc_t = 4'd2;
        else if (S_dc_ten_in >= 10'd10) S_dc_t = 4'd1;
        else                            S_dc_t = 4'd0;
    end

    wire [9:0] S_dc_one_in = S_dc_ten_in - ((S_dc_t << 3) + (S_dc_t << 1));
    wire [3:0] S_dc_o     = S_dc_one_in[3:0];

    // ---- 延迟毫秒值拆成十位/个位（0~99，比较-减法，无除法器）----
    reg [3:0] S_lat_t;
    always @(*) begin
        if      (I_lat >= 10'd90) S_lat_t = 4'd9;
        else if (I_lat >= 10'd80) S_lat_t = 4'd8;
        else if (I_lat >= 10'd70) S_lat_t = 4'd7;
        else if (I_lat >= 10'd60) S_lat_t = 4'd6;
        else if (I_lat >= 10'd50) S_lat_t = 4'd5;
        else if (I_lat >= 10'd40) S_lat_t = 4'd4;
        else if (I_lat >= 10'd30) S_lat_t = 4'd3;
        else if (I_lat >= 10'd20) S_lat_t = 4'd2;
        else if (I_lat >= 10'd10) S_lat_t = 4'd1;
        else                      S_lat_t = 4'd0;
    end

    wire [9:0] S_lat_o_in = I_lat - ((S_lat_t << 3) + (S_lat_t << 1));
    wire [3:0] S_lat_o    = S_lat_o_in[3:0];

    // ---- 每个槽取哪一字模 ----
    //   0=F 1=P 2=S 3=: 4..13=0..9 15=空白
    //   16=R 17=A 18=W 19=E 20=D 21=G 22=Y 23=O 24=L 25=T
    reg [4:0] S_osd_ch;
    always @(*) begin
        case (S_osd_ridx)
            3'd1: begin
                // 第 1 行：算法名  0=RAW  1=EDGE  2=GRAY  3=POP
                case (S_osd_slot)
                    3'd0: S_osd_ch = (I_mode == 2'd0) ? 5'd16 :
                                     (I_mode == 2'd1) ? 5'd19 :
                                     (I_mode == 2'd2) ? 5'd21 : 5'd1;    // R / E / G / P
                    3'd1: S_osd_ch = (I_mode == 2'd0) ? 5'd17 :
                                     (I_mode == 2'd1) ? 5'd20 :
                                     (I_mode == 2'd2) ? 5'd16 : 5'd23;   // A / D / R / O
                    3'd2: S_osd_ch = (I_mode == 2'd0) ? 5'd18 :
                                     (I_mode == 2'd1) ? 5'd21 :
                                     (I_mode == 2'd2) ? 5'd17 : 5'd1;    // W / G / A / P
                    3'd3: S_osd_ch = (I_mode == 2'd1) ? 5'd19 :
                                     (I_mode == 2'd2) ? 5'd22 : 5'd15;   // E / Y / 空白
                    3'd4: S_osd_ch = 5'd28;                              // C
                    3'd5: S_osd_ch = 5'd4 + {3'd0, I_cal_st};            // 标定状态 0/1/2/3
                    default: S_osd_ch = 5'd15;
                endcase
            end
            3'd3: begin
                // 第 3 行：中心区域 R 均值 + G 均值（满量程 0~255，供白平衡标定）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd16;               // R
                    3'd1:    S_osd_ch = 5'd4 + S_ar_h;       // 百位
                    3'd2:    S_osd_ch = 5'd4 + S_ar_t;       // 十位
                    3'd3:    S_osd_ch = 5'd4 + S_ar_o;       // 个位
                    3'd4:    S_osd_ch = 5'd21;               // G
                    3'd5:    S_osd_ch = 5'd4 + S_ag_h;
                    3'd6:    S_osd_ch = 5'd4 + S_ag_t;
                    default: S_osd_ch = 5'd4 + S_ag_o;
                endcase
            end
            3'd4: begin
                // 第 4 行：中心区域 B 均值（满量程 0~255）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd27;               // B
                    3'd1:    S_osd_ch = 5'd4 + S_ab_h;
                    3'd2:    S_osd_ch = 5'd4 + S_ab_t;
                    3'd3:    S_osd_ch = 5'd4 + S_ab_o;
                    default: S_osd_ch = 5'd15;
                endcase
            end
            3'd5: begin
                // 第 5 行：当前白平衡增益  R123B185  =  R 1.23x、B 1.85x
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd16;               // R
                    3'd1:    S_osd_ch = 5'd4 + S_gr_h_q;
                    3'd2:    S_osd_ch = 5'd4 + S_gr_t_q;
                    3'd3:    S_osd_ch = 5'd4 + S_gr_o_q;
                    3'd4:    S_osd_ch = 5'd27;               // B
                    3'd5:    S_osd_ch = 5'd4 + S_gb_h_q;
                    3'd6:    S_osd_ch = 5'd4 + S_gb_t_q;
                    default: S_osd_ch = 5'd4 + S_gb_o_q;
                endcase
            end
            3'd6: begin
                // 第 6 行：AE 曝光/增益  E1991G96（板测状态）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd19;              // E
                    3'd1:    S_osd_ch = 5'd4 + S_ae_th_q;   // 千位 0/1
                    3'd2:    S_osd_ch = 5'd4 + S_ae_h_q;    // 百位
                    3'd3:    S_osd_ch = 5'd4 + S_ae_t_q;    // 十位
                    3'd4:    S_osd_ch = 5'd4 + S_ae_o_q;    // 个位
                    3'd5:    S_osd_ch = 5'd21;              // G
                    3'd6:    S_osd_ch = 5'd4 + S_aeg_t_q;   // 增益十位
                    default: S_osd_ch = 5'd4 + S_aeg_o_q;   // 增益个位
                endcase
            end
            3'd7: begin
                // 第 7 行：圆形诊断 D<flags hex>C<cnt>
                //   CCL : flags[7:0]={ovf,live,chord,rows,asp,size,pass,valid}
                //   EVT : flags[7:0]={ovf,live,drop,0,0,0,pass,valid}
                //   EVT 的 cnt 已按 0..999 饱和显示（999 表示 >=999）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd20;                  // D
                    3'd1:    S_osd_ch = hex_glyph(I_det_flags[7:4]);
                    3'd2:    S_osd_ch = hex_glyph(I_det_flags[3:0]);
                    3'd3:    S_osd_ch = 5'd28;                  // C
                    3'd4:    S_osd_ch = 5'd4 + S_dc_h;
                    3'd5:    S_osd_ch = 5'd4 + S_dc_t;
                    3'd6:    S_osd_ch = 5'd4 + S_dc_o;
                    default: S_osd_ch = 5'd15;
                endcase
            end
            3'd2: begin
                // 第 2 行：延迟  LAT:xxMS（毫秒，单位必须写全，避免被当成分钟）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd24;               // L
                    3'd1:    S_osd_ch = 5'd17;               // A
                    3'd2:    S_osd_ch = 5'd25;               // T
                    3'd3:    S_osd_ch = 5'd3;                // :
                    3'd4:    S_osd_ch = 5'd4 + S_lat_t;      // 十位
                    3'd5:    S_osd_ch = 5'd4 + S_lat_o;      // 个位
                    3'd6:    S_osd_ch = 5'd26;               // M
                    default: S_osd_ch = 5'd2;                // S
                endcase
            end
            default: begin
                // 第 0 行：FPS:xxx
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd0;
                    3'd1:    S_osd_ch = 5'd1;
                    3'd2:    S_osd_ch = 5'd2;
                    3'd3:    S_osd_ch = 5'd3;
                    3'd4:    S_osd_ch = (I_fps_value >= 10'd100) ? (5'd4 + S_dig_h) : 5'd15;
                    3'd5:    S_osd_ch = (I_fps_value >= 10'd10 ) ? (5'd4 + S_dig_t) : 5'd15;
                    3'd6:    S_osd_ch = 5'd4 + S_dig_o;
                    default: S_osd_ch = 5'd15;   // 第 8 格保持空白
                endcase
            end
        endcase
    end

    // 字符槽内坐标：0..9，字形本体位于 1..8，第 0 列/行和第 9 列/行留给描边
    wire [3:0] S_osd_cellx = S_osd_colx / OSD_SCALE;      // 0..9
    wire [3:0] S_osd_celly = S_osd_oyr[6:0] / OSD_SCALE;  // 0..9

    // 字形像素坐标（15 = 越界，取 0）
    wire [3:0] S_gx  = ((S_osd_cellx >= 4'd1) && (S_osd_cellx <= 4'd8)) ? (S_osd_cellx - 4'd1) : 4'd15;
    wire [3:0] S_gxm = ((S_osd_cellx >= 4'd2) && (S_osd_cellx <= 4'd9)) ? (S_osd_cellx - 4'd2) : 4'd15;
    wire [3:0] S_gxp = (S_osd_cellx <= 4'd7) ? S_osd_cellx : 4'd15;

    wire [3:0] S_gy  = ((S_osd_celly >= 4'd1) && (S_osd_celly <= 4'd8)) ? (S_osd_celly - 4'd1) : 4'd15;
    wire [3:0] S_gym = ((S_osd_celly >= 4'd2) && (S_osd_celly <= 4'd9)) ? (S_osd_celly - 4'd2) : 4'd15;
    wire [3:0] S_gyp = (S_osd_celly <= 4'd7) ? S_osd_celly : 4'd15;

    // ------------------------------------------------------------------
    // OSD 查表输入打一拍
    //   路径：x/y -> 分槽 -> ÷3 -> 字形坐标 -> glyph_row(30 选 1 大 case)
    //         -> 行内取位 -> 8 邻域描边 -> 输出 mux -> O_hdmi_data
    //   实测 13.0ns / 13.333ns，只有 +9ps 余量，且走线占 6 成，属布线敏感。
    //   这里把【查表输入】寄存一拍，让大 case 从寄存器起步，直接砍掉
    //   x/y 到字形坐标这一段组合链。
    //   OSD 是静态叠加，晚一拍只是整体右移 1 个像素，肉眼不可见；
    //   取样框 S_osd_box 同步延迟，保证"框"和"字"仍然对齐。
    // ------------------------------------------------------------------
    reg        S_osd_box_q;
    reg [4:0]  S_osd_ch_q;
    reg [3:0]  S_gx_q, S_gxm_q, S_gxp_q;
    reg [3:0]  S_gy_q, S_gym_q, S_gyp_q;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_osd_box_q <= 1'b0;
            S_osd_ch_q  <= 5'd15;
            S_gx_q  <= 4'd15; S_gxm_q <= 4'd15; S_gxp_q <= 4'd15;
            S_gy_q  <= 4'd15; S_gym_q <= 4'd15; S_gyp_q <= 4'd15;
        end
        else begin
            S_osd_box_q <= S_osd_box;
            S_osd_ch_q  <= S_osd_ch;
            S_gx_q  <= S_gx;  S_gxm_q <= S_gxm; S_gxp_q <= S_gxp;
            S_gy_q  <= S_gy;  S_gym_q <= S_gym; S_gyp_q <= S_gyp;
        end
    end

    // ---- 8x8 点阵字模，bit0 为最左像素 ----
    function [7:0] osd_font;
        input [4:0] ch;
        input [2:0] row;
        begin
            osd_font = 8'h00;
            case (ch)
                4'd0: case (row)   // F
                    3'd0: osd_font = 8'h3F; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h03; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h03; default: osd_font = 8'h00;
                endcase
                4'd1: case (row)   // P
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h63;
                    3'd4: osd_font = 8'h1F; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h03; default: osd_font = 8'h00;
                endcase
                4'd2: case (row)   // S
                    3'd0: osd_font = 8'h1E; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h1E;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h60;
                    3'd6: osd_font = 8'h1E; default: osd_font = 8'h00;
                endcase
                4'd3: case (row)   // :
                    3'd1: osd_font = 8'h0C; 3'd2: osd_font = 8'h0C;
                    3'd4: osd_font = 8'h0C; 3'd5: osd_font = 8'h0C;
                    default: osd_font = 8'h00;
                endcase
                4'd4: case (row)   // 0
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h6E; 3'd3: osd_font = 8'h76;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd5: case (row)   // 1
                    3'd0: osd_font = 8'h18; 3'd1: osd_font = 8'h1C;
                    3'd2: osd_font = 8'h18; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h7E; default: osd_font = 8'h00;
                endcase
                4'd6: case (row)   // 2
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h0C; 3'd5: osd_font = 8'h06;
                    3'd6: osd_font = 8'h7E; default: osd_font = 8'h00;
                endcase
                4'd7: case (row)   // 3
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h38;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd8: case (row)   // 4
                    3'd0: osd_font = 8'h18; 3'd1: osd_font = 8'h38;
                    3'd2: osd_font = 8'h6C; 3'd3: osd_font = 8'h33;
                    3'd4: osd_font = 8'h7E; 3'd5: osd_font = 8'h60;
                    3'd6: osd_font = 8'h60; default: osd_font = 8'h00;
                endcase
                4'd9: case (row)   // 5
                    3'd0: osd_font = 8'h7E; 3'd1: osd_font = 8'h06;
                    3'd2: osd_font = 8'h3E; 3'd3: osd_font = 8'h60;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd10: case (row)  // 6
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h06;
                    3'd2: osd_font = 8'h06; 3'd3: osd_font = 8'h3E;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd11: case (row)  // 7
                    3'd0: osd_font = 8'h7E; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h18; default: osd_font = 8'h00;
                endcase
                4'd12: case (row)  // 8
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h66; 3'd3: osd_font = 8'h3C;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd13: case (row)  // 9
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h66; 3'd3: osd_font = 8'h7C;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd16: case (row)  // R
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h1B; 3'd5: osd_font = 8'h33;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd17: case (row)  // A
                    3'd0: osd_font = 8'h1C; 3'd1: osd_font = 8'h36;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h63;
                    3'd4: osd_font = 8'h7F; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd18: case (row)  // W
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h6B;
                    3'd4: osd_font = 8'h6B; 3'd5: osd_font = 8'h36;
                    3'd6: osd_font = 8'h22; default: osd_font = 8'h00;
                endcase
                5'd19: case (row)  // E
                    3'd0: osd_font = 8'h7F; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h03; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h7F; default: osd_font = 8'h00;
                endcase
                5'd20: case (row)  // D
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h33;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h63;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h33;
                    3'd6: osd_font = 8'h1F; default: osd_font = 8'h00;
                endcase
                5'd21: case (row)  // G
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h73;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd22: case (row)  // Y
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h36; 3'd3: osd_font = 8'h1C;
                    3'd4: osd_font = 8'h08; 3'd5: osd_font = 8'h08;
                    3'd6: osd_font = 8'h08; default: osd_font = 8'h00;
                endcase
                5'd23: case (row)  // O
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h66; 3'd3: osd_font = 8'h66;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd24: case (row)  // L
                    3'd0: osd_font = 8'h03; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h03;
                    3'd4: osd_font = 8'h03; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h7E; default: osd_font = 8'h00;
                endcase
                5'd25: case (row)  // T
                    3'd0: osd_font = 8'h7E; 3'd1: osd_font = 8'h18;
                    3'd2: osd_font = 8'h18; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h18; default: osd_font = 8'h00;
                endcase
                5'd26: case (row)  // M
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h77;
                    3'd2: osd_font = 8'h7F; 3'd3: osd_font = 8'h6B;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd28: case (row)  // C
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h60;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd27: case (row)  // B
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h1F; default: osd_font = 8'h00;
                endcase
                5'd14: case (row)  // X (signed-coordinate axis label)
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h36; 3'd3: osd_font = 8'h1C;
                    3'd4: osd_font = 8'h36; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd29: case (row)  // I
                    3'd0: osd_font = 8'h7F; 3'd1: osd_font = 8'h18;
                    3'd2: osd_font = 8'h18; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h7F; default: osd_font = 8'h00;
                endcase
                5'd30: case (row)  // +
                    3'd0: osd_font = 8'h00; 3'd1: osd_font = 8'h18;
                    3'd2: osd_font = 8'h18; 3'd3: osd_font = 8'h7E;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h00; default: osd_font = 8'h00;
                endcase
                5'd31: case (row)  // -
                    3'd0: osd_font = 8'h00; 3'd1: osd_font = 8'h00;
                    3'd2: osd_font = 8'h00; 3'd3: osd_font = 8'h7E;
                    3'd4: osd_font = 8'h00; 3'd5: osd_font = 8'h00;
                    3'd6: osd_font = 8'h00; default: osd_font = 8'h00;
                endcase
                default: osd_font = 8'h00;
            endcase
        end
    endfunction

    // ---- 取某一行字模（越界返回 0）----
    function [7:0] glyph_row;
        input [4:0] ch;
        input [3:0] y;
        begin
            if (y > 4'd7) glyph_row = 8'h00;
            else          glyph_row = osd_font(ch, y[2:0]);
        end
    endfunction

    // ---- 8x8 字模（8 行全用，专供识别框旁 32px 文字）----------------
    // 与 7 行的 HUD 字模分开，保证字形高度正好 8*4 = 32 显示像素。
    function [7:0] txt_font;
        input [4:0] ch;
        input [2:0] row;
        begin
            txt_font = 8'h00;
            case (ch)
                5'd28: case (row)  // C (LSB-left: stem on the left)
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h06; 3'd3: txt_font=8'h06;
                    3'd4: txt_font=8'h06; 3'd5: txt_font=8'h06;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd29: case (row)  // I
                    3'd0: txt_font=8'h7E; 3'd1: txt_font=8'h18;
                    3'd2: txt_font=8'h18; 3'd3: txt_font=8'h18;
                    3'd4: txt_font=8'h18; 3'd5: txt_font=8'h18;
                    3'd6: txt_font=8'h18; default: txt_font=8'h7E;
                endcase
                5'd16: case (row)  // R (LSB-left, HUD shape)
                    3'd0: txt_font=8'h1F; 3'd1: txt_font=8'h63;
                    3'd2: txt_font=8'h63; 3'd3: txt_font=8'h1F;
                    3'd4: txt_font=8'h1B; 3'd5: txt_font=8'h33;
                    3'd6: txt_font=8'h63; default: txt_font=8'h63;
                endcase
                5'd24: case (row)  // L (LSB-left: stem on the left)
                    3'd0: txt_font=8'h03; 3'd1: txt_font=8'h03;
                    3'd2: txt_font=8'h03; 3'd3: txt_font=8'h03;
                    3'd4: txt_font=8'h03; 3'd5: txt_font=8'h03;
                    3'd6: txt_font=8'h03; default: txt_font=8'h7E;
                endcase
                5'd19: case (row)  // E
                    3'd0: txt_font=8'h7E; 3'd1: txt_font=8'h06;
                    3'd2: txt_font=8'h06; 3'd3: txt_font=8'h3E;
                    3'd4: txt_font=8'h06; 3'd5: txt_font=8'h06;
                    3'd6: txt_font=8'h06; default: txt_font=8'h7E;
                endcase
                5'd14: case (row)  // X
                    3'd0: txt_font=8'h66; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h3C; 3'd3: txt_font=8'h18;
                    3'd4: txt_font=8'h3C; 3'd5: txt_font=8'h66;
                    3'd6: txt_font=8'h66; default: txt_font=8'h66;
                endcase
                5'd22: case (row)  // Y
                    3'd0: txt_font=8'h66; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h66; 3'd3: txt_font=8'h3C;
                    3'd4: txt_font=8'h18; 3'd5: txt_font=8'h18;
                    3'd6: txt_font=8'h18; default: txt_font=8'h18;
                endcase
                5'd30: case (row)  // +
                    3'd0: txt_font=8'h00; 3'd1: txt_font=8'h18;
                    3'd2: txt_font=8'h18; 3'd3: txt_font=8'h7E;
                    3'd4: txt_font=8'h18; 3'd5: txt_font=8'h18;
                    3'd6: txt_font=8'h00; default: txt_font=8'h00;
                endcase
                5'd31: case (row)  // -
                    3'd0: txt_font=8'h00; 3'd1: txt_font=8'h00;
                    3'd2: txt_font=8'h00; 3'd3: txt_font=8'h7E;
                    3'd4: txt_font=8'h00; 3'd5: txt_font=8'h00;
                    3'd6: txt_font=8'h00; default: txt_font=8'h00;
                endcase
                5'd4: case (row)  // 0
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h66; 3'd3: txt_font=8'h66;
                    3'd4: txt_font=8'h66; 3'd5: txt_font=8'h66;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd5: case (row)  // 1
                    3'd0: txt_font=8'h18; 3'd1: txt_font=8'h1C;
                    3'd2: txt_font=8'h18; 3'd3: txt_font=8'h18;
                    3'd4: txt_font=8'h18; 3'd5: txt_font=8'h18;
                    3'd6: txt_font=8'h18; default: txt_font=8'h7E;
                endcase
                5'd6: case (row)  // 2
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h60; 3'd3: txt_font=8'h30;
                    3'd4: txt_font=8'h18; 3'd5: txt_font=8'h0C;
                    3'd6: txt_font=8'h06; default: txt_font=8'h7E;
                endcase
                5'd7: case (row)  // 3
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h60; 3'd3: txt_font=8'h38;
                    3'd4: txt_font=8'h60; 3'd5: txt_font=8'h60;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd8: case (row)  // 4
                    3'd0: txt_font=8'h18; 3'd1: txt_font=8'h38;
                    3'd2: txt_font=8'h6C; 3'd3: txt_font=8'h66;
                    3'd4: txt_font=8'h7E; 3'd5: txt_font=8'h60;
                    3'd6: txt_font=8'h60; default: txt_font=8'h60;
                endcase
                5'd9: case (row)  // 5
                    3'd0: txt_font=8'h7E; 3'd1: txt_font=8'h06;
                    3'd2: txt_font=8'h06; 3'd3: txt_font=8'h3E;
                    3'd4: txt_font=8'h60; 3'd5: txt_font=8'h60;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd10: case (row) // 6
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h06; 3'd3: txt_font=8'h3E;
                    3'd4: txt_font=8'h66; 3'd5: txt_font=8'h66;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd11: case (row) // 7
                    3'd0: txt_font=8'h7E; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h60; 3'd3: txt_font=8'h30;
                    3'd4: txt_font=8'h18; 3'd5: txt_font=8'h18;
                    3'd6: txt_font=8'h18; default: txt_font=8'h18;
                endcase
                5'd12: case (row) // 8
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h66; 3'd3: txt_font=8'h3C;
                    3'd4: txt_font=8'h66; 3'd5: txt_font=8'h66;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd13: case (row) // 9
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h66; 3'd3: txt_font=8'h3E;
                    3'd4: txt_font=8'h60; 3'd5: txt_font=8'h60;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd25: case (row) // G (GREEN), HUD-style open mouth
                    3'd0: txt_font=8'h3C; 3'd1: txt_font=8'h66;
                    3'd2: txt_font=8'h03; 3'd3: txt_font=8'h73;
                    3'd4: txt_font=8'h63; 3'd5: txt_font=8'h63;
                    3'd6: txt_font=8'h66; default: txt_font=8'h3C;
                endcase
                5'd26: case (row) // N (GREEN), LSB-left
                    3'd0: txt_font=8'hC3; 3'd1: txt_font=8'hE3;
                    3'd2: txt_font=8'hF3; 3'd3: txt_font=8'hDB;
                    3'd4: txt_font=8'hCF; 3'd5: txt_font=8'hC7;
                    3'd6: txt_font=8'hC3; default: txt_font=8'hC3;
                endcase
                default: txt_font = 8'h00;
            endcase
        end
    endfunction

    // 字模位序：显示按 bit=gx 取（gx 向右递增），因此字模必须是
    // LSB-在左（bit0 为最左像素）。上板 CIЯↃ⅃E 的根因是 C/R/L
    // 用了 MSB-在左的图案；E 和数字原本已是 LSB-在左。这里只把
    // C/R/L 改为 LSB-左，并保留 E/数字的原图案，不再整体反转。
    function [7:0] txt_glyph_row;
        input [4:0] ch;
        input [3:0] y;
        begin
            if (y > 4'd7) txt_glyph_row = 8'h00;
            else          txt_glyph_row = txt_font(ch, y[2:0]);
        end
    endfunction

    wire [15:0] S_row_c = {8'b0, glyph_row(S_osd_ch_q, S_gy_q )};
    wire [15:0] S_row_m = {8'b0, glyph_row(S_osd_ch_q, S_gym_q)};
    wire [15:0] S_row_p = {8'b0, glyph_row(S_osd_ch_q, S_gyp_q)};

    wire S_pix_c  = S_row_c[S_gx_q ];
    wire S_pix_l  = S_row_c[S_gxm_q];
    wire S_pix_r  = S_row_c[S_gxp_q];
    wire S_pix_u  = S_row_m[S_gx_q ];
    wire S_pix_d  = S_row_p[S_gx_q ];
    wire S_pix_ul = S_row_m[S_gxm_q];
    wire S_pix_ur = S_row_m[S_gxp_q];
    wire S_pix_dl = S_row_p[S_gxm_q];
    wire S_pix_dr = S_row_p[S_gxp_q];

    wire S_osd_green = S_pix_c;                       // 字形：亮绿
    wire S_osd_black = ~S_pix_c &&                    // 描边：8 邻域有字形则涂黑
                       (S_pix_l | S_pix_r | S_pix_u | S_pix_d |
                        S_pix_ul | S_pix_ur | S_pix_dl | S_pix_dr);

    // ==================================================================
    // 识别框旁的文字：CIRCLE X<sign><ddd> Y<sign><ddd>
    //   - 18 个字符槽，10x10 字体单元（8x8 字形 + 1 像素描边边），
    //     1:1 像素：字形高 8 显示像素，字距/行高 10
    //   - 绿色字形 + 黑色 8 邻域描边；背景透明
    //   - 坐标 = 圆心相对画面中心 (IMG_WIDTH/2, IMG_HEIGHT/2)：
    //       向右 X 正、向下 Y 正；始终显示符号，三位补零
    //   - 位置在框上方（不够则下方），水平居中并夹在有效区内，防裁切
    //   - 只在 I_det_valid=1 时显示；框失效文字消失
    //   - 全部取自 I_det_* 同一显示帧快照（wrapper 在 S_hdmi_user 整组锁存）
    //   - 坐标/位置/字形查表分级寄存，避免长组合路径（与既有 OSD 同理）
    // ==================================================================
    localparam integer TXT_SCALE = 1;                      // 8x8 glyph at 1x -> 8 px
    localparam integer TXT_CELL  = 10;                     // 8 glyph + 1 border
    localparam integer TXT_ADV   = TXT_CELL * TXT_SCALE;   // 10
    localparam integer TXT_CHARS = 24;                     // max: CIRCLE GREEN XYZ
    localparam integer TXT_W     = TXT_CHARS * TXT_ADV;    // 240 (max)
    localparam integer TXT_H     = TXT_CELL * TXT_SCALE;   // 10
    localparam integer TXT_GAP   = 4;
    localparam integer TXT_XMAX  = IMG_WIDTH  - TXT_W - 4; // 1036 (3-px pipeline + margin)
    localparam integer TXT_YMAX  = IMG_HEIGHT - TXT_H;     // 710

    // floor(x/10) for 0 <= x < 180: five restoring-division steps, no divider.
    function [4:0] txt_div10;
        input [11:0] x;
        reg [12:0] rem;
        integer i;
        begin
            rem = {1'b0, x};
            txt_div10 = 5'd0;
            for (i = 4; i >= 0; i = i - 1) begin
                if (rem >= (13'd10 << i)) begin
                    rem = rem - (13'd10 << i);
                    txt_div10[i] = 1'b1;
                end
            end
        end
    endfunction

    wire signed [12:0] S_txt_dx = $signed({1'b0, I_det_cx}) - 13'sd640;
    wire signed [12:0] S_txt_dy = $signed({1'b0, I_det_cy}) - 13'sd360;
    wire S_txt_xneg = S_txt_dx[12];
    wire S_txt_yneg = S_txt_dy[12];
    wire [12:0] S_txt_xabs = S_txt_xneg ? (~S_txt_dx + 13'd1) : S_txt_dx;
    wire [12:0] S_txt_yabs = S_txt_yneg ? (~S_txt_dy + 13'd1) : S_txt_dy;
    wire [9:0]  S_txt_xv = (S_txt_xabs > 13'd999) ? 10'd999 : S_txt_xabs[9:0];
    wire [9:0]  S_txt_yv = (S_txt_yabs > 13'd999) ? 10'd999 : S_txt_yabs[9:0];

    // ---- pipelined decimal decode -------------------------------------
    // I_det_* are frame-static here: design_top_wrapper latches the whole
    // result group at S_hdmi_user, so these registers only delay the text
    // update by a few clocks.  They never move a glyph: the pixel position is
    // still compared against S_x_2d/S_y_2d with the same glyph table.
    // They break the former I_det_cy -> div100 -> div10 -> S_txt_yo_q path
    // (HDMI_PIXEL_CLK setup, -0.365 ns in the rev-2n gate report) that the
    // runtime-clock OSD pipeline did not touch.
    reg [9:0] S_txt_xv_p, S_txt_yv_p;
    reg       S_txt_xneg_p0, S_txt_yneg_p0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_txt_xv_p <= 10'd0; S_txt_yv_p <= 10'd0;
            S_txt_xneg_p0 <= 1'b0; S_txt_yneg_p0 <= 1'b0;
        end
        else begin
            S_txt_xv_p <= S_txt_xv; S_txt_yv_p <= S_txt_yv;
            S_txt_xneg_p0 <= S_txt_xneg; S_txt_yneg_p0 <= S_txt_yneg;
        end
    end

    wire [3:0] S_txt_xh = div100_999(S_txt_xv_p);
    wire [9:0] S_txt_xr = S_txt_xv_p -
                          (({6'd0,S_txt_xh} << 6) + ({6'd0,S_txt_xh} << 5) +
                           ({6'd0,S_txt_xh} << 2));
    wire [3:0] S_txt_yh = div100_999(S_txt_yv_p);
    wire [9:0] S_txt_yr = S_txt_yv_p -
                          (({6'd0,S_txt_yh} << 6) + ({6'd0,S_txt_yh} << 5) +
                           ({6'd0,S_txt_yh} << 2));

    reg [3:0] S_txt_xh_p, S_txt_yh_p;
    reg [9:0] S_txt_xr_p, S_txt_yr_p;
    reg       S_txt_xneg_p, S_txt_yneg_p;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_txt_xh_p <= 4'd0; S_txt_yh_p <= 4'd0;
            S_txt_xr_p <= 10'd0; S_txt_yr_p <= 10'd0;
            S_txt_xneg_p <= 1'b0; S_txt_yneg_p <= 1'b0;
        end
        else begin
            S_txt_xh_p <= S_txt_xh; S_txt_yh_p <= S_txt_yh;
            S_txt_xr_p <= S_txt_xr; S_txt_yr_p <= S_txt_yr;
            S_txt_xneg_p <= S_txt_xneg_p0; S_txt_yneg_p <= S_txt_yneg_p0;
        end
    end

    wire [3:0] S_txt_xt = div10_99(S_txt_xr_p[6:0]);
    wire [3:0] S_txt_xo = S_txt_xr_p[3:0] -
                          (({1'b0,S_txt_xt} << 3) + ({1'b0,S_txt_xt} << 1));
    wire [3:0] S_txt_yt = div10_99(S_txt_yr_p[6:0]);
    wire [3:0] S_txt_yo = S_txt_yr_p[3:0] -
                          (({1'b0,S_txt_yt} << 3) + ({1'b0,S_txt_yt} << 1));

    // Dynamic width: 18 chars normally, 24 when the optional GREEN label is
    // shown.  The label stays centred over the box instead of leaving a gap.
    wire [11:0] S_txt_w_act = I_det_green ? 12'd240 : 12'd180;
    wire [11:0] S_txt_xmax  = 12'd1276 - S_txt_w_act;
    wire [12:0] S_txt_bcx = ({1'b0,I_det_minx} + {1'b0,I_det_maxx}) >> 1;
    wire signed [13:0] S_txt_x0_raw = $signed({1'b0,S_txt_bcx}) -
                                      (I_det_green ? 14'sd120 : 14'sd90);
    wire [11:0] S_txt_x0 = S_txt_x0_raw[13] ? 12'd0 :
                           (S_txt_x0_raw > {2'b0,S_txt_xmax}) ? S_txt_xmax :
                           S_txt_x0_raw[11:0];

    // Above the box if it fits, otherwise below, otherwise clamp to bottom.
    wire S_txt_above_ok = (I_det_miny >= (TXT_H + TXT_GAP));
    wire [12:0] S_txt_below = {1'b0,I_det_maxy} + TXT_GAP;
    wire S_txt_below_ok = (S_txt_below <= TXT_YMAX);
    wire [11:0] S_txt_y0 = S_txt_above_ok ? (I_det_miny - (TXT_H + TXT_GAP)) :
                           S_txt_below_ok ? S_txt_below[11:0] : TXT_YMAX;

    // ---- stage 0: decoded coordinates + placement (same display frame) ----
    reg        S_txt_on_q;
    reg        S_txt_isgreen_q;
    reg [11:0] S_txt_w_q;
    reg        S_txt_xneg_q, S_txt_yneg_q;
    reg [3:0]  S_txt_xh_q, S_txt_xt_q, S_txt_xo_q;
    reg [3:0]  S_txt_yh_q, S_txt_yt_q, S_txt_yo_q;
    reg [11:0] S_txt_x0_q, S_txt_y0_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_txt_on_q <= 1'b0;
            S_txt_isgreen_q <= 1'b0; S_txt_w_q <= 12'd180;
            S_txt_xneg_q <= 1'b0; S_txt_yneg_q <= 1'b0;
            S_txt_xh_q <= 4'd0; S_txt_xt_q <= 4'd0; S_txt_xo_q <= 4'd0;
            S_txt_yh_q <= 4'd0; S_txt_yt_q <= 4'd0; S_txt_yo_q <= 4'd0;
            S_txt_x0_q <= 12'd0; S_txt_y0_q <= 12'd0;
        end
        else begin
            S_txt_on_q    <= I_det_valid;
            S_txt_isgreen_q <= I_det_green;
            S_txt_w_q     <= S_txt_w_act;
            S_txt_xneg_q <= S_txt_xneg_p; S_txt_yneg_q <= S_txt_yneg_p;
            S_txt_xh_q   <= S_txt_xh_p;   S_txt_xt_q   <= S_txt_xt;
            S_txt_xo_q   <= S_txt_xo;
            S_txt_yh_q   <= S_txt_yh_p;   S_txt_yt_q   <= S_txt_yt;
            S_txt_yo_q   <= S_txt_yo;
            S_txt_x0_q   <= S_txt_x0;   S_txt_y0_q   <= S_txt_y0;
        end
    end

    // ---- stage A: pixel -> slot / cell -> glyph index -------------------
    wire S_txt_in_a = S_txt_on_q &&
                      (S_x_2d >= S_txt_x0_q) && (S_x_2d < (S_txt_x0_q + S_txt_w_q)) &&
                      (S_y_2d >= S_txt_y0_q) && (S_y_2d < (S_txt_y0_q + TXT_H));
    wire [11:0] S_txt_ox = S_x_2d - S_txt_x0_q;
    wire [11:0] S_txt_oy = S_y_2d - S_txt_y0_q;
    wire [4:0]  S_txt_slot = txt_div10(S_txt_ox);          // 0..17
    wire [9:0]  S_txt_cellx = S_txt_ox -
                              (({5'd0,S_txt_slot} << 3) +
                               ({5'd0,S_txt_slot} << 1));  // 0..9
    wire [9:0]  S_txt_celly = S_txt_oy[3:0];               // 0..9

    reg [4:0] S_txt_ch;
    always @(*) begin
        case (S_txt_slot)
            5'd0:  S_txt_ch = 5'd28;                    // C
            5'd1:  S_txt_ch = 5'd29;                    // I
            5'd2:  S_txt_ch = 5'd16;                    // R
            5'd3:  S_txt_ch = 5'd28;                    // C
            5'd4:  S_txt_ch = 5'd24;                    // L
            5'd5:  S_txt_ch = 5'd19;                    // E
            5'd6:  S_txt_ch = 5'd15;                    // space
            // green: 7..11 G R E E N, 12 space, 13 X, 14 sign, 15..17 Xdig
            // non-green: 7 X, 8 sign, 9..11 Xdig, 12 space, 13 Y, 14 sign, 15..17 Ydig
            5'd7:  S_txt_ch = S_txt_isgreen_q ? 5'd25 : 5'd14;
            5'd8:  S_txt_ch = S_txt_isgreen_q ? 5'd16 :
                              (S_txt_xneg_q ? 5'd31 : 5'd30);
            5'd9:  S_txt_ch = S_txt_isgreen_q ? 5'd19 :
                              (5'd4 + {1'b0,S_txt_xh_q});
            5'd10: S_txt_ch = S_txt_isgreen_q ? 5'd19 :
                              (5'd4 + {1'b0,S_txt_xt_q});
            5'd11: S_txt_ch = S_txt_isgreen_q ? 5'd26 :
                              (5'd4 + {1'b0,S_txt_xo_q});
            5'd12: S_txt_ch = 5'd15;                    // space
            5'd13: S_txt_ch = S_txt_isgreen_q ? 5'd14 : 5'd22;
            5'd14: S_txt_ch = S_txt_isgreen_q ?
                              (S_txt_xneg_q ? 5'd31 : 5'd30) :
                              (S_txt_yneg_q ? 5'd31 : 5'd30);
            5'd15: S_txt_ch = S_txt_isgreen_q ?
                              (5'd4 + {1'b0,S_txt_xh_q}) :
                              (5'd4 + {1'b0,S_txt_yh_q});
            5'd16: S_txt_ch = S_txt_isgreen_q ?
                              (5'd4 + {1'b0,S_txt_xt_q}) :
                              (5'd4 + {1'b0,S_txt_yt_q});
            5'd17: S_txt_ch = S_txt_isgreen_q ?
                              (5'd4 + {1'b0,S_txt_xo_q}) :
                              (5'd4 + {1'b0,S_txt_yo_q});
            5'd18: S_txt_ch = 5'd15;                    // space
            5'd19: S_txt_ch = S_txt_isgreen_q ? 5'd22 : 5'd15;
            5'd20: S_txt_ch = S_txt_isgreen_q ?
                              (S_txt_yneg_q ? 5'd31 : 5'd30) : 5'd15;
            5'd21: S_txt_ch = S_txt_isgreen_q ?
                              (5'd4 + {1'b0,S_txt_yh_q}) : 5'd15;
            5'd22: S_txt_ch = S_txt_isgreen_q ?
                              (5'd4 + {1'b0,S_txt_yt_q}) : 5'd15;
            5'd23: S_txt_ch = S_txt_isgreen_q ?
                              (5'd4 + {1'b0,S_txt_yo_q}) : 5'd15;
            default: S_txt_ch = 5'd15;
        endcase
    end

    reg        S_txt_box_q;
    reg [4:0]  S_txt_ch_q;
    reg [3:0]  S_txt_cellx_q, S_txt_celly_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_txt_box_q <= 1'b0; S_txt_ch_q <= 5'd15;
            S_txt_cellx_q <= 4'd0; S_txt_celly_q <= 4'd0;
        end
        else begin
            S_txt_box_q   <= S_txt_in_a;
            S_txt_ch_q    <= S_txt_ch;
            S_txt_cellx_q <= S_txt_cellx[3:0];
            S_txt_celly_q <= S_txt_celly[3:0];
        end
    end

    // ---- stage B: registered cell -> glyph pixels + 8-neighbour outline --
    wire [3:0] S_txt_gx  = ((S_txt_cellx_q >= 4'd1) && (S_txt_cellx_q <= 4'd8))
                           ? (S_txt_cellx_q - 4'd1) : 4'd15;
    wire [3:0] S_txt_gxm = ((S_txt_cellx_q >= 4'd2) && (S_txt_cellx_q <= 4'd9))
                           ? (S_txt_cellx_q - 4'd2) : 4'd15;
    wire [3:0] S_txt_gxp = (S_txt_cellx_q <= 4'd7) ? S_txt_cellx_q : 4'd15;
    wire [3:0] S_txt_gy  = ((S_txt_celly_q >= 4'd1) && (S_txt_celly_q <= 4'd8))
                           ? (S_txt_celly_q - 4'd1) : 4'd15;
    wire [3:0] S_txt_gym = ((S_txt_celly_q >= 4'd2) && (S_txt_celly_q <= 4'd9))
                           ? (S_txt_celly_q - 4'd2) : 4'd15;
    wire [3:0] S_txt_gyp = (S_txt_celly_q <= 4'd7) ? S_txt_celly_q : 4'd15;

    wire [15:0] S_txt_row_c = {8'b0, txt_glyph_row(S_txt_ch_q, S_txt_gy )};
    wire [15:0] S_txt_row_m = {8'b0, txt_glyph_row(S_txt_ch_q, S_txt_gym)};
    wire [15:0] S_txt_row_p = {8'b0, txt_glyph_row(S_txt_ch_q, S_txt_gyp)};

    wire S_txt_pix_c  = S_txt_row_c[S_txt_gx ];
    wire S_txt_pix_l  = S_txt_row_c[S_txt_gxm];
    wire S_txt_pix_r  = S_txt_row_c[S_txt_gxp];
    wire S_txt_pix_u  = S_txt_row_m[S_txt_gx ];
    wire S_txt_pix_d  = S_txt_row_p[S_txt_gx ];
    wire S_txt_pix_ul = S_txt_row_m[S_txt_gxm];
    wire S_txt_pix_ur = S_txt_row_m[S_txt_gxp];
    wire S_txt_pix_dl = S_txt_row_p[S_txt_gxm];
    wire S_txt_pix_dr = S_txt_row_p[S_txt_gxp];

    wire S_txt_green = S_txt_box_q && S_txt_pix_c;
    wire S_txt_black = S_txt_box_q && ~S_txt_pix_c &&
                       (S_txt_pix_l | S_txt_pix_r | S_txt_pix_u | S_txt_pix_d |
                        S_txt_pix_ul | S_txt_pix_ur | S_txt_pix_dl | S_txt_pix_dr);

    reg S_txt_green_q, S_txt_black_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_txt_green_q <= 1'b0; S_txt_black_q <= 1'b0;
        end
        else begin
            S_txt_green_q <= S_txt_green;
            S_txt_black_q <= S_txt_black;
        end
    end

    // ==================================================================
    // 运行时长 DD:HH:MM:SS（右下角，左上 HUD 同字体/绿色描边）
    //   11 个 30x30 单元，固定位置；字段来自 50 MHz 域整组原子 CDC。
    //   ":" 用 HUD 字模 glyph 3；数字 4..13。字段在 1 Hz 更新。
    // ==================================================================
    localparam integer RT_SCALE = 3;
    localparam integer RT_CELL  = 10;
    localparam integer RT_ADV   = RT_CELL * RT_SCALE;       // 30
    localparam integer RT_CHARS = 11;
    localparam integer RT_W     = RT_CHARS * RT_ADV;        // 330
    localparam integer RT_H     = RT_CELL * RT_SCALE;       // 30
    localparam integer RT_X0    = IMG_WIDTH  - RT_W - 24;   // 926
    localparam integer RT_Y0    = IMG_HEIGHT - RT_H - 24;   // 666

    function [4:0] rt_div30;
        input [11:0] x;
        reg [12:0] rem;
        integer i;
        begin
            rem = {1'b0, x};
            rt_div30 = 5'd0;
            for (i = 4; i >= 0; i = i - 1) begin
                if (rem >= (13'd30 << i)) begin
                    rem = rem - (13'd30 << i);
                    rt_div30[i] = 1'b1;
                end
            end
        end
    endfunction

    function [3:0] rt_div3;
        input [11:0] x;
        reg [12:0] rem;
        integer i;
        begin
            rem = {1'b0, x};
            rt_div3 = 4'd0;
            for (i = 3; i >= 0; i = i - 1) begin
                if (rem >= (13'd3 << i)) begin
                    rem = rem - (13'd3 << i);
                    rt_div3[i] = 1'b1;
                end
            end
        end
    endfunction

    wire [3:0] S_rt_day_t = div10_99(I_rt_day[6:0]);
    wire [6:0] S_rt_day_r = I_rt_day[6:0] -
        (({3'd0,S_rt_day_t} << 3) + ({3'd0,S_rt_day_t} << 1));
    wire [3:0] S_rt_day_o = S_rt_day_r[3:0];
    wire [3:0] S_rt_hour_t = div10_99(I_rt_hour[6:0]);
    wire [6:0] S_rt_hour_r = I_rt_hour[6:0] -
        (({3'd0,S_rt_hour_t} << 3) + ({3'd0,S_rt_hour_t} << 1));
    wire [3:0] S_rt_hour_o = S_rt_hour_r[3:0];
    wire [3:0] S_rt_min_t = div10_99(I_rt_min[6:0]);
    wire [6:0] S_rt_min_r = I_rt_min[6:0] -
        (({3'd0,S_rt_min_t} << 3) + ({3'd0,S_rt_min_t} << 1));
    wire [3:0] S_rt_min_o = S_rt_min_r[3:0];
    wire [3:0] S_rt_sec_t = div10_99(I_rt_sec[6:0]);
    wire [6:0] S_rt_sec_r = I_rt_sec[6:0] -
        (({3'd0,S_rt_sec_t} << 3) + ({3'd0,S_rt_sec_t} << 1));
    wire [3:0] S_rt_sec_o = S_rt_sec_r[3:0];

    // ---- runtime OSD coordinate pipeline --------------------------------
    // S_x/S_y are exactly two pixels ahead of S_x_2d/S_y_2d (the data and
    // coordinate paths are both two-stage).  Dividing by 30 and then by 3 in
    // one clock was the HDMI_PIXEL_CLK critical path.  Computing from S_x/S_y
    // and using two register stages gives each divider a full clock period,
    // while the q stage still sees exactly the coordinate that S_x_2d/S_y_2d
    // held one clock earlier: the rendered overlay is pixel-identical to the
    // original single-stage version (verified against the rev-2m RTL).
    wire S_rt_in_lead = (S_x >= RT_X0) && (S_x < (RT_X0 + RT_W)) &&
                        (S_y >= RT_Y0) && (S_y < (RT_Y0 + RT_H));
    wire [11:0] S_rt_ox_lead = S_x - RT_X0;
    wire [11:0] S_rt_oy_lead = S_y - RT_Y0;
    wire [4:0]  S_rt_slot_lead = rt_div30(S_rt_ox_lead);    // 0..10
    wire [11:0] S_rt_rem_lead = S_rt_ox_lead -
                              (({5'd0,S_rt_slot_lead} << 5) -
                               ({5'd0,S_rt_slot_lead} << 1));
    wire [9:0]  S_rt_celly_lead = rt_div3(S_rt_oy_lead);    // 0..9

    reg        S_rt_box_d1;
    reg [4:0]  S_rt_slot_d1;
    reg [11:0] S_rt_rem_d1;
    reg [9:0]  S_rt_celly_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_rt_box_d1 <= 1'b0; S_rt_slot_d1 <= 5'd0;
            S_rt_rem_d1 <= 12'd0; S_rt_celly_d1 <= 10'd0;
        end
        else begin
            S_rt_box_d1   <= S_rt_in_lead;
            S_rt_slot_d1  <= S_rt_slot_lead;
            S_rt_rem_d1   <= S_rt_rem_lead;
            S_rt_celly_d1 <= S_rt_celly_lead;
        end
    end

    wire [9:0] S_rt_cellx_lead2 = rt_div3(S_rt_rem_d1);     // 0..9

    reg        S_rt_box_d2;
    reg [4:0]  S_rt_slot_d2;
    reg [9:0]  S_rt_cellx_d2, S_rt_celly_d2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_rt_box_d2 <= 1'b0; S_rt_slot_d2 <= 5'd0;
            S_rt_cellx_d2 <= 10'd0; S_rt_celly_d2 <= 10'd0;
        end
        else begin
            S_rt_box_d2   <= S_rt_box_d1;
            S_rt_slot_d2  <= S_rt_slot_d1;
            S_rt_cellx_d2 <= S_rt_cellx_lead2;
            S_rt_celly_d2 <= S_rt_celly_d1;
        end
    end

    reg [4:0] S_rt_ch;
    always @(*) begin
        case (S_rt_slot_d2)
            5'd0:  S_rt_ch = 5'd4 + {1'b0,S_rt_day_t};
            5'd1:  S_rt_ch = 5'd4 + {1'b0,S_rt_day_o};
            5'd2:  S_rt_ch = 5'd3;                          // :
            5'd3:  S_rt_ch = 5'd4 + {1'b0,S_rt_hour_t};
            5'd4:  S_rt_ch = 5'd4 + {1'b0,S_rt_hour_o};
            5'd5:  S_rt_ch = 5'd3;
            5'd6:  S_rt_ch = 5'd4 + {1'b0,S_rt_min_t};
            5'd7:  S_rt_ch = 5'd4 + {1'b0,S_rt_min_o};
            5'd8:  S_rt_ch = 5'd3;
            5'd9:  S_rt_ch = 5'd4 + {1'b0,S_rt_sec_t};
            5'd10: S_rt_ch = 5'd4 + {1'b0,S_rt_sec_o};
            default: S_rt_ch = 5'd15;
        endcase
    end

    reg        S_rt_box_q;
    reg [4:0]  S_rt_ch_q;
    reg [3:0]  S_rt_cellx_q, S_rt_celly_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_rt_box_q <= 1'b0; S_rt_ch_q <= 5'd15;
            S_rt_cellx_q <= 4'd0; S_rt_celly_q <= 4'd0;
        end
        else begin
            S_rt_box_q   <= S_rt_box_d2;
            S_rt_ch_q    <= S_rt_ch;
            S_rt_cellx_q <= S_rt_cellx_d2[3:0];
            S_rt_celly_q <= S_rt_celly_d2[3:0];
        end
    end

    wire [3:0] S_rt_gx  = ((S_rt_cellx_q >= 4'd1) && (S_rt_cellx_q <= 4'd8))
                          ? (S_rt_cellx_q - 4'd1) : 4'd15;
    wire [3:0] S_rt_gxm = ((S_rt_cellx_q >= 4'd2) && (S_rt_cellx_q <= 4'd9))
                          ? (S_rt_cellx_q - 4'd2) : 4'd15;
    wire [3:0] S_rt_gxp = (S_rt_cellx_q <= 4'd7) ? S_rt_cellx_q : 4'd15;
    wire [3:0] S_rt_gy  = ((S_rt_celly_q >= 4'd1) && (S_rt_celly_q <= 4'd8))
                          ? (S_rt_celly_q - 4'd1) : 4'd15;
    wire [3:0] S_rt_gym = ((S_rt_celly_q >= 4'd2) && (S_rt_celly_q <= 4'd9))
                          ? (S_rt_celly_q - 4'd2) : 4'd15;
    wire [3:0] S_rt_gyp = (S_rt_celly_q <= 4'd7) ? S_rt_celly_q : 4'd15;

    wire [15:0] S_rt_row_c = {8'b0, glyph_row(S_rt_ch_q, S_rt_gy )};
    wire [15:0] S_rt_row_m = {8'b0, glyph_row(S_rt_ch_q, S_rt_gym)};
    wire [15:0] S_rt_row_p = {8'b0, glyph_row(S_rt_ch_q, S_rt_gyp)};

    wire S_rt_pix_c  = S_rt_row_c[S_rt_gx ];
    wire S_rt_pix_l  = S_rt_row_c[S_rt_gxm];
    wire S_rt_pix_r  = S_rt_row_c[S_rt_gxp];
    wire S_rt_pix_u  = S_rt_row_m[S_rt_gx ];
    wire S_rt_pix_d  = S_rt_row_p[S_rt_gx ];
    wire S_rt_pix_ul = S_rt_row_m[S_rt_gxm];
    wire S_rt_pix_ur = S_rt_row_m[S_rt_gxp];
    wire S_rt_pix_dl = S_rt_row_p[S_rt_gxm];
    wire S_rt_pix_dr = S_rt_row_p[S_rt_gxp];

    wire S_rt_green = S_rt_box_q && S_rt_pix_c;
    wire S_rt_black = S_rt_box_q && ~S_rt_pix_c &&
                      (S_rt_pix_l | S_rt_pix_r | S_rt_pix_u | S_rt_pix_d |
                       S_rt_pix_ul | S_rt_pix_ur | S_rt_pix_dl | S_rt_pix_dr);

    reg S_rt_green_q, S_rt_black_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_rt_green_q <= 1'b0; S_rt_black_q <= 1'b0;
        end
        else begin
            S_rt_green_q <= S_rt_green;
            S_rt_black_q <= S_rt_black;
        end
    end

    // ==================================================================
    // 安路 Logo（基础要求 2.3(4)）：右上角 160x160 位图
    //   ROM 像素 25bit = {不透明标志[24], RGB[23:0]}
    //   透明像素(bit24=0)不覆盖画面，因此只有图案本身出现
    // ==================================================================
    localparam integer LOGO_W = 160;
    localparam integer LOGO_H = 160;
    localparam integer LOGO_X = IMG_WIDTH - LOGO_W;   // 1280 - 160 = 1120
    localparam integer LOGO_Y = 0;

    wire S_logo_box = (S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
                      (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H));

    // 地址换算改成移位相加，不要乘法。
    //   LOGO_W = 160 = 128 + 32，写成 (y<<7)+(y<<5)+x 后只剩一个加法器。
    //   原来用 *160 综合出了 DSPMULT，路径变成
    //     S_y_2d -> DSP乘法 -> 加法 -> ROM地址 -> ROM读 -> 输出mux -> 寄存器
    //   实测 15.057ns / 13.333ns 预算（-2.112ns），其中 8.4ns 是走线。
    //   移位是纯连线，能省下 DSP 那一级和它前后的大扇出网络。
    wire [14:0] S_logo_relx = S_x_2d - LOGO_X;                       // x 相对 Logo 左边界
    wire [14:0] S_logo_addr = (({3'd0,S_y_2d} << 7) + ({3'd0,S_y_2d} << 5)) + S_logo_relx;
    wire [24:0] S_logo_pixel;

    // 【地址打一拍】实测该加法器输出一根网 fanout=551、单根走线 4.0ns，
    //   占掉 13.3ns 预算的三分之一，是 OSD/Logo 合成路径走线占比 71% 的主因。
    //   地址是"每像素一个值"的组合量，借一拍即可让它只驱动寄存器。
    //   Logo 是静态叠加，晚一拍只是整体右移 1 像素，肉眼不可见；
    //   S_logo_box 同步延迟，保证"框"与"像素"仍然配对。
    reg  [14:0] S_logo_addr_q;
    reg         S_logo_box_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_logo_addr_q <= 15'd0;
            S_logo_box_q  <= 1'b0;
        end
        else begin
            S_logo_addr_q <= S_logo_addr;
            S_logo_box_q  <= S_logo_box;
        end
    end

    anlogic_logo_rom u_anlogic_logo_rom (
        .I_addr  ( S_logo_addr_q ),
        .O_pixel ( S_logo_pixel  )
    );

    // ==================================================================
    // 黄色目标外接框：绿色 3 像素边框
    // ==================================================================
    localparam integer BOX_T = 3;

    // 绿色 = 颜色/面积/形状全部通过；橙色 = 颜色和面积过了但形状没通过，
    // 用于现场判断是"颜色判据漏检"还是"形状判据太严"
    wire S_in_box = (I_det_valid || I_det_cand) &&
                    (S_x_2d >= I_det_minx) && (S_x_2d <= I_det_maxx) &&
                    (S_y_2d >= I_det_miny) && (S_y_2d <= I_det_maxy);

    wire S_box_edge = S_in_box &&
                      ((S_x_2d <  I_det_minx + BOX_T) ||
                       (S_x_2d + BOX_T > I_det_maxx) ||
                       (S_y_2d <  I_det_miny + BOX_T) ||
                       (S_y_2d + BOX_T > I_det_maxy));

    // ==================================================================
    // 中心取样框（标定辅助）：1 像素红色细框
    //   位置与 ISP 里 O_rgb_r/g/b 的取样区域完全一致：
    //     isp_top.v  S_in_center: x 608~671, y 328~391（64x64）
    //   把被测物体（白纸 / 黄卷尺）对进这个框，OSD 第 3、4 行的读数
    //   就是它本身的 R/G/B。演示时把 SHOW_CB 置 0 即可隐藏。
    // ==================================================================
    // Registered one stage before pixel output to keep HDMI timing short.
    // Cyan identifies the new red ball; yellow retains green/orange and priority.
    wire S_red_inside = I_red_valid &&
        (S_x_1d >= I_red_minx) && (S_x_1d <= I_red_maxx) &&
        (S_y_1d >= I_red_miny) && (S_y_1d <= I_red_maxy);
    wire S_red_edge = S_red_inside &&
        ((S_x_1d < I_red_minx + BOX_T) || (S_x_1d + BOX_T > I_red_maxx) ||
         (S_y_1d < I_red_miny + BOX_T) || (S_y_1d + BOX_T > I_red_maxy));
    reg S_red_edge_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) S_red_edge_q <= 1'b0;
        else S_red_edge_q <= S_red_edge;
    end

    localparam integer CB_X0 = 608;
    localparam integer CB_Y0 = 328;
    localparam integer CB_W  = 64;
    localparam integer CB_H  = 64;
    localparam integer CB_T  = 1;                 // 线宽：1 像素
    localparam         SHOW_CB = 1'b1;

    wire S_cb_edge = (S_x_2d >= CB_X0) && (S_x_2d < CB_X0 + CB_W) &&
                     (S_y_2d >= CB_Y0) && (S_y_2d < CB_Y0 + CB_H) &&
                     ((S_x_2d <  CB_X0 + CB_T) ||
                      (S_x_2d >= CB_X0 + CB_W - CB_T) ||
                      (S_y_2d <  CB_Y0 + CB_T) ||
                      (S_y_2d >= CB_Y0 + CB_H - CB_T));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_window_active_1d <= 1'b0;
            S_window_active_2d <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_window_active_1d <= S_window_active;
            S_window_active_2d <= S_window_active_1d;
            O_hdmi_de        <= S_video_de_2d;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 4) begin
                // Full-screen MIPI receive diagnostic.  The status is
                // generated and synchronized in design_top_wrapper.
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'd0: O_hdmi_data <= 24'h000080; // blue: sensor config incomplete
                        4'd1: O_hdmi_data <= 24'h000000; // black: no MIPI HS traffic
                        4'd2: O_hdmi_data <= 24'h00ffff; // cyan: HS present, no complete frame
                        4'd3: O_hdmi_data <= 24'hff0000; // red: D-PHY lane error
                        4'd4: O_hdmi_data <= 24'hff00ff; // magenta: RAW10 line length error
                        4'd5: O_hdmi_data <= 24'hffff00; // yellow: not 720 lines per frame
                        4'd6: O_hdmi_data <= 24'h00ff00; // green: MIPI frame structure passed
                        4'd7: O_hdmi_data <= 24'hff8000; // orange: sensor I2C transaction error
                        default: O_hdmi_data <= 24'hffffff;
                    endcase
                    // 诊断页也叠加 FPS，便于同时确认帧率
                    if(S_osd_box_q) begin
                        if(S_osd_green)
                            O_hdmi_data <= 24'h00ff00;
                        else if(S_osd_black)
                            O_hdmi_data <= 24'h000000;
                    end
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_video_de_2d) begin
                    O_hdmi_data <= S_window_active_2d ? S_video_data_2d : 24'd0;
                    // 识别框旁文字：绿色字形 + 黑色描边（最高优先级，防被
                    // 其它叠加层截断；背景透明，非字形像素继续走下面的链）
                    if(S_txt_green_q)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_txt_black_q)
                        O_hdmi_data <= 24'h000000;
                    // 黄色目标外接框：绿=完全通过，橙=形状未通过
                    else if(S_box_edge)
                        O_hdmi_data <= I_det_valid ? 24'h00ff00 : 24'hff8000;
                    else if(S_red_edge_q)
                        O_hdmi_data <= 24'h00ffff;
                    // 中心取样框：红色细框（标定用，和被检测框重叠时让检测框优先）
                    else if(SHOW_CB && S_cb_edge)
                        O_hdmi_data <= 24'hff0000;
                    // 安路 Logo：右上角，仅非透明像素覆盖
                    else if(S_logo_box_q && S_logo_pixel[24])
                        O_hdmi_data <= S_logo_pixel[23:0];
                    // FPS OSD：背景透明，仅字形（亮绿）和描边（黑）覆盖画面
                    else if(S_osd_box_q) begin
                        if(S_osd_green)
                            O_hdmi_data <= 24'h00ff00;
                        else if(S_osd_black)
                            O_hdmi_data <= 24'h000000;
                    end
                    // 运行时长 OSD：右下角，同字体绿色+黑描边（最低优先级；
                    // 框/文字/Logo 覆盖处让位，其余像素正常显示）
                    else if(S_rt_box_q) begin
                        if(S_rt_green_q)
                            O_hdmi_data <= 24'h00ff00;
                        else if(S_rt_black_q)
                            O_hdmi_data <= 24'h000000;
                    end
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
        end
    end

endmodule
