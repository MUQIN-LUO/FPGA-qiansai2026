// ==========================================================================
// 前景掩膜（重构版）：背景色差 ∪ 亮度梯度
// --------------------------------------------------------------------------
// 为什么不再用"绝对色度"：
//   旧判据 foreground = (max(R,G,B)-min(R,G,B) >= 20) ∪ (梯度 >= 48)。
//   只要背景本身有颜色（蓝布、木桌、草地），色度就远超门限，整片背景都成
//   前景 -> 前景铺满画面 -> 外接框接近满幅 -> 圆形判据必然失败
//   （实测蓝布 chroma≈80，整帧被判前景，仿真复现 rho=0.995）。
//
// 新判据：
//   foreground = ( |R-Rbg| + |G-Gbg| + |B-Bbg| >= BG_TH ) ∪ ( 亮度梯度 >= EDGE_TH )
//   Rbg/Gbg/Bbg 来自帧级背景估计（画面四周环带的慢速均值）。
//   只要目标颜色与**背景**不同就能分离，与目标本身是什么颜色无关：
//   红/黄/蓝/绿/黑/白圆都能出来；背景是大面积纯色也不影响。
//
// 两级流水（避免把 3 减法 + 3 取绝对值 + 2 加法 + 比较压在 103 MHz 一拍里）：
//   S1: 每像素算三通道绝对差之和，寄存
//   S2: 与 BG_TH 比较、并与 Sobel 边缘掩膜取并
// ==========================================================================
module isp_fg_mask #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [9:0]   BG_TH      = 10'd60      // 三通道绝对差之和门限
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire [7:0]  I_bg_r,
    input  wire [7:0]  I_bg_g,
    input  wire [7:0]  I_bg_b,
    // edge_detect_96 输出（比 I_* 晚 1 拍）
    input  wire        I_edge_valid,
    input  wire        I_edge_tlast,
    input  wire        I_edge_tuser,
    input  wire [95:0] I_edge_tdata,
    output wire [3:0]  O_mask,
    output wire        O_valid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [11:0] O_x,
    output wire [11:0] O_y
);

    function [9:0] cdist;
        input [23:0] p;
        input [7:0]  br, bgc, bb;
        reg   [7:0]  dr, dg, db;
        begin
            dr = (p[23:16] >= br)  ? (p[23:16] - br)  : (br - p[23:16]);
            dg = (p[15:8]  >= bgc) ? (p[15:8]  - bgc) : (bgc - p[15:8]);
            db = (p[7:0]   >= bb)  ? (p[7:0]   - bb)  : (bb - p[7:0]);
            cdist = {2'd0, dr} + {2'd0, dg} + {2'd0, db};
        end
    endfunction

    wire [9:0] d0 = cdist(I_tdata[95:72], I_bg_r, I_bg_g, I_bg_b);
    wire [9:0] d1 = cdist(I_tdata[71:48], I_bg_r, I_bg_g, I_bg_b);
    wire [9:0] d2 = cdist(I_tdata[47:24], I_bg_r, I_bg_g, I_bg_b);
    wire [9:0] d3 = cdist(I_tdata[23:0],  I_bg_r, I_bg_g, I_bg_b);

    reg [3:0]  S1_hit;
    reg [11:0] S1_x, S1_y;
    reg [11:0] S1_xc, S1_yc;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_hit <= 4'd0;
            S1_x  <= 12'd0; S1_y  <= 12'd0;
            S1_xc <= 12'd0; S1_yc <= 12'd0;
        end
        else if (I_tvalid) begin
            S1_hit <= { (d3 >= BG_TH), (d2 >= BG_TH),
                        (d1 >= BG_TH), (d0 >= BG_TH) };
            if (I_tuser) begin
                S1_x  <= 12'd0;
                S1_y  <= 12'd0;
                S1_xc <= 12'd4;
                S1_yc <= 12'd0;
            end
            else begin
                S1_x <= S1_xc;
                S1_y <= S1_yc;
                if (S1_xc >= (IMG_WIDTH - 4)) begin
                    S1_xc <= 12'd0;
                    S1_yc <= (S1_yc >= (IMG_HEIGHT - 1)) ? 12'd0 : S1_yc + 12'd1;
                end
                else begin
                    S1_xc <= S1_xc + 12'd4;
                end
            end
        end
    end

    wire [3:0] S_grad_hit = { |I_edge_tdata[23:0],   |I_edge_tdata[47:24],
                              |I_edge_tdata[71:48],  |I_edge_tdata[95:72] };

    assign O_mask  = S1_hit | S_grad_hit;
    assign O_valid = I_edge_valid;
    assign O_tlast = I_edge_tlast;
    assign O_tuser = I_edge_tuser;
    assign O_x     = S1_x;
    assign O_y     = S1_y;

endmodule


// ==========================================================================
// 背景色估计：画面四周环带像素的平均色，帧级更新
// --------------------------------------------------------------------------
// 环带像素数 NB_PIX = 2*W*BORDER + 2*(H-2*BORDER)*BORDER
//   1280x720、BORDER=8 -> 31744，取 2^15 = 32768 作除数，偏大 3.2%，
//   对"背景色"这种只需要粗略值的场合完全够用；AVG_SHIFT 可调。
// ==========================================================================
module isp_bg_est #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [11:0]  BORDER     = 12'd8,
    parameter integer AVG_SHIFT  = 15         // 2^15 = 32768 ≈ 环带像素数
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tvalid,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    output reg  [7:0]  O_bg_r,
    output reg  [7:0]  O_bg_g,
    output reg  [7:0]  O_bg_b
);

    reg [11:0] cx, cy;
    wire [11:0] cx_n = (cx >= (IMG_WIDTH - 4)) ? 12'd0 : cx + 12'd4;
    wire [11:0] cy_n = (cy >= (IMG_HEIGHT - 1)) ? 12'd0 : cy + 12'd1;

    wire S_frm_end = I_tvalid && (cx >= (IMG_WIDTH - 4)) && (cy >= (IMG_HEIGHT - 1));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            cx <= 12'd0;
            cy <= 12'd0;
        end
        else if (I_tvalid) begin
            if (I_tuser) begin
                cx <= 12'd4;
                cy <= 12'd0;
            end
            else begin
                cx <= cx_n;
                cy <= (cx >= (IMG_WIDTH - 4)) ? cy_n : cy;
            end
        end
    end

    // 外圈环带
    wire S_border = (cy < BORDER) || (cy >= (IMG_HEIGHT - BORDER)) ||
                    (cx < BORDER) || (cx >= (IMG_WIDTH - BORDER));

    wire [9:0] S_sum_r = I_tdata[95:88] + I_tdata[71:64] + I_tdata[47:40] + I_tdata[23:16];
    wire [9:0] S_sum_g = I_tdata[87:80] + I_tdata[63:56] + I_tdata[39:32] + I_tdata[15:8];
    wire [9:0] S_sum_b = I_tdata[79:72] + I_tdata[55:48] + I_tdata[31:24] + I_tdata[7:0];

    reg [27:0] S_acc_r, S_acc_g, S_acc_b;
    reg        S_first;            // 首帧直接采用均值，之后慢速跟踪

    wire [7:0] S_mean_r = (S_acc_r >> AVG_SHIFT > 28'd255) ? 8'd255 : (S_acc_r >> AVG_SHIFT);
    wire [7:0] S_mean_g = (S_acc_g >> AVG_SHIFT > 28'd255) ? 8'd255 : (S_acc_g >> AVG_SHIFT);
    wire [7:0] S_mean_b = (S_acc_b >> AVG_SHIFT > 28'd255) ? 8'd255 : (S_acc_b >> AVG_SHIFT);

    // 慢速平滑：向新均值靠 1/4，避免目标扫过边缘时背景跳变
    function [7:0] iir4;
        input [7:0] old;
        input [7:0] tgt;
        begin
            iir4 = (tgt > old) ? (old + ((tgt - old) >> 2))
                               : (old - ((old - tgt) >> 2));
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_acc_r <= 28'd0;
            S_acc_g <= 28'd0;
            S_acc_b <= 28'd0;
            O_bg_r  <= 8'd128;
            O_bg_g  <= 8'd128;
            O_bg_b  <= 8'd128;
            S_first <= 1'b0;
        end
        else if (S_frm_end) begin
            S_acc_r <= 28'd0;
            S_acc_g <= 28'd0;
            S_acc_b <= 28'd0;
            if (!S_first) begin
                // 首帧直接采用环带均值（上电默认灰只是占位）
                O_bg_r  <= S_mean_r;
                O_bg_g  <= S_mean_g;
                O_bg_b  <= S_mean_b;
                S_first <= 1'b1;
            end
            else begin
                O_bg_r  <= iir4(O_bg_r, S_mean_r);
                O_bg_g  <= iir4(O_bg_g, S_mean_g);
                O_bg_b  <= iir4(O_bg_b, S_mean_b);
            end
        end
        else if (I_tvalid && S_border) begin
            S_acc_r <= S_acc_r + {18'd0, S_sum_r};
            S_acc_g <= S_acc_g + {18'd0, S_sum_g};
            S_acc_b <= S_acc_b + {18'd0, S_sum_b};
        end
    end

endmodule
