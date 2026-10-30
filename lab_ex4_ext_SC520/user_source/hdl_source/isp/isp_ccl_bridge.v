// ==========================================================================
// CCL 跨时钟桥：掩膜流 FIFO + 坐标重建 + CCL + 结果 CDC + 丢拍恢复
// --------------------------------------------------------------------------
// 设计与回归测试**实例化同一份本模块**，测试台不再手写镜像逻辑。
//
// 时钟：I_mipi_clk = MIPI 字节时钟（103.52 MHz）；
//       I_ccl_clk  = 50 MHz 系统时钟 /2（25 MHz，真正异步）。
// 复位：I_rst_n 为 MIPI 域异步复位；CCL 域用同步释放的 ccl_rst_n。
//
// 丢拍恢复：
//   err   —— 贴下一个成功写入拍，标记当前帧损坏；
//   resync—— 粘滞到下一个成功写入的 tuser；
//   坏帧后的 tuser：本拍组合清 CCL + 暂存一拍，下一拍以 clean 状态送入；
//   frame_open —— 只有干净 tuser 能开帧，非 tuser 拍遇 err/resync 即关帧。
//
// 事件 CDC：C_evt → toggle（CCL 域）→ 3FF（MIPI 域），边沿用第 2/3 级；
//           结果数据保持到下一帧，在稳定边沿后采样。
// ==========================================================================
module isp_ccl_bridge #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [11:0]  MIN_W_PX   = 12'd24,
    parameter [11:0]  MAX_W_PX   = 12'd720,
    parameter [23:0]  MIN_CELLS  = 24'd6
)(
    // ---------------- MIPI 域 ----------------
    input  wire        I_mipi_clk,
    input  wire        I_rst_n,
    input  wire        I_fg_valid,
    input  wire [3:0]  I_fg_mask,
    input  wire        I_fg_tuser,
    input  wire        I_fg_tlast,
    input  wire        I_det_mask,      // 标定屏蔽（MIPI 域电平）
    // ---------------- CCL 域时钟 ----------------
    input  wire        I_ccl_clk,
    // ---------------- 结果（MIPI 域，已 CDC 采样） ----------------
    output reg         O_evt,
    output reg  [11:0] O_w,
    output reg  [11:0] O_h,
    output reg  [23:0] O_cnt,
    output reg  [20:0] O_chord,
    output reg  [10:0] O_rows,
    output reg  [11:0] O_minx,
    output reg  [11:0] O_maxx,
    output reg  [11:0] O_miny,
    output reg  [11:0] O_maxy,
    output reg         O_pass,
    output reg         O_cand,
    output reg         O_ok_size,
    output reg         O_ok_asp,
    output reg         O_ok_rows,
    output reg         O_ok_chord,
    output reg  [3:0]  O_lab,
    output reg         O_ovf,
    // ---------------- 调试（测试/观测用） ----------------
    output wire        O_dbg_full,
    output wire        O_dbg_drop
);

    // ---------------- CCL 域复位：异步置位、同步释放 ----------------
    reg [1:0] ccl_rst_sync;
    always @(posedge I_ccl_clk or negedge I_rst_n) begin
        if (!I_rst_n) ccl_rst_sync <= 2'd0;
        else          ccl_rst_sync <= {ccl_rst_sync[0], 1'b1};
    end
    wire ccl_rst_n = ccl_rst_sync[1];

    // ---------------- 掩膜 FIFO（MIPI → CCL） ----------------
    reg         S_mf_err_pending;
    reg         S_mf_resync;
    wire        S_mf_drop = I_fg_valid & O_dbg_full;
    wire        S_mf_err_w = S_mf_err_pending & ~(I_fg_tuser & S_mf_resync);
    wire [9:0]  S_mf_wdata = {S_mf_resync, S_mf_err_w,
                              I_fg_mask, I_fg_tuser, I_fg_tlast, 2'b00};
    wire        S_mf_empty, S_mf_valid;
    wire [9:0]  S_mf_rdata;
    wire        S_ccl_busy;
    wire        S_ccl_rd_en;

    always @(posedge I_mipi_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_mf_err_pending <= 1'b0;
            S_mf_resync      <= 1'b0;
        end
        else if (S_mf_drop) begin
            S_mf_err_pending <= 1'b1;
            S_mf_resync      <= 1'b1;
        end
        else if (I_fg_valid && !O_dbg_full) begin
            if (S_mf_err_pending) S_mf_err_pending <= 1'b0;
            if (I_fg_tuser)       S_mf_resync      <= 1'b0;
        end
    end

    isp_mask_fifo #(.AW(9)) u_mask_fifo (
        .I_wr_clk  (I_mipi_clk),
        .I_wr_rst_n(I_rst_n),
        .I_wr_en   (I_fg_valid),
        .I_wr_data (S_mf_wdata),
        .O_full    (O_dbg_full),
        .I_rd_clk  (I_ccl_clk),
        .I_rd_rst_n(ccl_rst_n),
        .I_rd_en   (S_ccl_rd_en),
        .O_rd_data (S_mf_rdata),
        .O_rd_valid(S_mf_valid),
        .O_empty   (S_mf_empty)
    );

    // 调试：丢拍粘滞
    reg dbg_drop_r;
    always @(posedge I_mipi_clk or negedge I_rst_n) begin
        if (!I_rst_n) dbg_drop_r <= 1'b0;
        else if (S_mf_drop) dbg_drop_r <= 1'b1;
    end
    assign O_dbg_drop = dbg_drop_r;

    // ---------------- 读侧：坏帧、无丢拍恢复、帧门控 ----------------
    wire        S_mf_resync_r = S_mf_rdata[9];
    wire        S_mf_err      = S_mf_rdata[8];
    wire        S_mf_tuser    = S_mf_rdata[3];
    wire        S_mf_tlast    = S_mf_rdata[2];

    reg         S_frame_bad;
    reg         S_frame_open;
    wire        S_tuser_bad = S_mf_valid && S_mf_tuser &&
                              (S_frame_bad || S_mf_resync_r) && !S_ccl_busy;
    assign      S_ccl_rd_en = ~S_mf_empty & ~S_ccl_busy & ~S_tuser_bad;


    // tuser 拍暂存一拍，下一拍以 clean 状态送入
    reg         S_hold_valid;
    reg [9:0]   S_hold_data;
    always @(posedge I_ccl_clk or negedge ccl_rst_n) begin
        if (!ccl_rst_n) begin
            S_hold_valid <= 1'b0;
            S_hold_data  <= 10'd0;
        end
        else begin
            S_hold_valid <= S_tuser_bad;
            if (S_tuser_bad) S_hold_data <= S_mf_rdata;
        end
    end

    wire        S_ccl_in_valid = S_hold_valid | (S_mf_valid && !S_tuser_bad);
    wire [9:0]  S_ccl_in_data  = S_hold_valid ? S_hold_data : S_mf_rdata;
    wire [3:0]  S_ccl_hit      = S_ccl_in_data[7:4];
    wire        S_ccl_tuser    = S_ccl_in_data[3];
    wire        S_ccl_tlast    = S_ccl_in_data[2];

    // x/y 重建
    reg [11:0] ccl_xcnt, ccl_ycnt;
    wire [11:0] S_ccl_x = S_ccl_tuser ? 12'd0 : ccl_xcnt;
    wire [11:0] S_ccl_y = S_ccl_tuser ? 12'd0 : ccl_ycnt;
    always @(posedge I_ccl_clk or negedge ccl_rst_n) begin
        if (!ccl_rst_n) begin
            ccl_xcnt <= 12'd0;
            ccl_ycnt <= 12'd0;
        end
        else if (S_ccl_in_valid) begin
            if (S_ccl_tuser) begin
                ccl_xcnt <= 12'd4;
                ccl_ycnt <= 12'd0;
            end
            else if (ccl_xcnt >= 12'd1276) begin
                ccl_xcnt <= 12'd0;
                ccl_ycnt <= ccl_ycnt + 12'd1;
            end
            else begin
                ccl_xcnt <= ccl_xcnt + 12'd4;
            end
        end
    end

    // 标定屏蔽同步
    reg [1:0] ccl_clr_sync;
    always @(posedge I_ccl_clk or negedge ccl_rst_n) begin
        if (!ccl_rst_n) ccl_clr_sync <= 2'd0;
        else            ccl_clr_sync <= {ccl_clr_sync[0], I_det_mask};
    end
    wire S_ccl_clr = ccl_clr_sync[1];

    // ---------------- CCL ----------------
    wire        C_evt, C_pass, C_cand, C_ok_size, C_ok_asp, C_ok_rows, C_ok_chord, C_ovf;
    wire [11:0] C_w, C_h, C_minx, C_maxx, C_miny, C_maxy;
    wire [23:0] C_cnt;
    wire [20:0] C_chord;
    wire [10:0] C_rows;
    wire [3:0]  C_lab;

    isp_ccl_circle #(
        .IMG_WIDTH (IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .MIN_W_PX  (MIN_W_PX),
        .MAX_W_PX  (MAX_W_PX),
        .MIN_CELLS (MIN_CELLS)
    ) u_circle_detect (
        .I_clk    (I_ccl_clk),
        .I_rst_n  (ccl_rst_n),
        .I_clr    (S_ccl_clr | S_tuser_bad),
        .I_valid  (S_ccl_in_valid),
        .I_hit    (S_ccl_hit),
        .I_x      (S_ccl_x),
        .I_y      (S_ccl_y),
        .O_evt    (C_evt),
        .O_w      (C_w),
        .O_h      (C_h),
        .O_cnt    (C_cnt),
        .O_chord  (C_chord),
        .O_rows   (C_rows),
        .O_minx   (C_minx),
        .O_maxx   (C_maxx),
        .O_miny   (C_miny),
        .O_maxy   (C_maxy),
        .O_pass   (C_pass),
        .O_cand   (C_cand),
        .O_ok_size(C_ok_size),
        .O_ok_asp (C_ok_asp),
        .O_ok_rows(C_ok_rows),
        .O_ok_chord(C_ok_chord),
        .O_lab    (C_lab),
        .O_ovf    (C_ovf),
        .O_busy   (S_ccl_busy)
    );

    // ---------------- 坏帧标志 / 帧门控（CCL 实例之后，使用 C_evt） ----------------
    always @(posedge I_ccl_clk or negedge ccl_rst_n) begin
        if (!ccl_rst_n) begin
            S_frame_bad  <= 1'b0;
            S_frame_open <= 1'b0;
        end
        else begin
            if (S_mf_valid) begin
                if (S_mf_tuser) begin
                    S_frame_bad  <= S_mf_err;
                    S_frame_open <= ~S_mf_err;          // resync tuser（err=0）开新帧
                end
                else if (S_mf_err || S_mf_resync_r) begin
                    S_frame_bad  <= 1'b1;
                    S_frame_open <= 1'b0;
                end
            end
            if (S_ccl_clr)                S_frame_open <= 1'b0;
            if (C_evt && S_frame_open)    S_frame_open <= 1'b0;
        end
    end

    // ---------------- 事件 CDC：3FF + 边沿 ----------------
    reg C_evt_tgl;
    reg C_ovf_hold;
    always @(posedge I_ccl_clk or negedge ccl_rst_n) begin
        if (!ccl_rst_n) begin
            C_evt_tgl  <= 1'b0;
            C_ovf_hold <= 1'b0;
        end
        else if (C_evt && S_frame_open) begin
            C_evt_tgl  <= ~C_evt_tgl;
            C_ovf_hold <= C_ovf;
        end
    end

    reg [2:0] S_evt_sync;
    always @(posedge I_mipi_clk or negedge I_rst_n) begin
        if (!I_rst_n) S_evt_sync <= 3'd0;
        else          S_evt_sync <= {S_evt_sync[1:0], C_evt_tgl};
    end
    wire S_evt_pulse = S_evt_sync[2] ^ S_evt_sync[1];

    always @(posedge I_mipi_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_evt <= 1'b0;
            O_w <= 12'd0; O_h <= 12'd0; O_cnt <= 24'd0; O_chord <= 21'd0; O_rows <= 11'd0;
            O_minx <= 12'd0; O_maxx <= 12'd0; O_miny <= 12'd0; O_maxy <= 12'd0;
            O_pass <= 1'b0; O_cand <= 1'b0;
            O_ok_size <= 1'b0; O_ok_asp <= 1'b0; O_ok_rows <= 1'b0; O_ok_chord <= 1'b0;
            O_lab <= 4'd0; O_ovf <= 1'b0;
        end
        else begin
            O_evt <= S_evt_pulse;
            if (S_evt_pulse) begin
                O_w <= C_w; O_h <= C_h; O_cnt <= C_cnt; O_chord <= C_chord; O_rows <= C_rows;
                O_minx <= C_minx; O_maxx <= C_maxx; O_miny <= C_miny; O_maxy <= C_maxy;
                O_pass <= C_pass; O_cand <= C_cand;
                O_ok_size <= C_ok_size; O_ok_asp <= C_ok_asp;
                O_ok_rows <= C_ok_rows; O_ok_chord <= C_ok_chord;
                O_lab <= C_lab; O_ovf <= C_ovf_hold;
            end
        end
    end

endmodule
