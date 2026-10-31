// ==========================================================================
// 颜色无关的圆形检测前端（替代按颜色取候选的做法）
// --------------------------------------------------------------------------
// 设计要点（对应赛题改造需求 1）：
//   1) 候选区域生成【不经过任何目标颜色阈值】。前景掩膜由两路判据取并集：
//        · 背景色差  |R-Rbg|+|G-Gbg|+|B-Bbg| >= BG_TH
//          Rbg/Gbg/Bbg 为画面四周环带的帧级均值。判的是"与背景不同"，
//          与目标本身是什么颜色无关。旧版用绝对色度，彩底会整帧成前景。
//        · 亮度梯度  Sobel(|gx|+|gy|) >= EDGE_TH （直接复用 edge_detect_96）
//          用来覆盖灰、白、黑这类与背景亮度差不足的圆：只要与背景有明暗
//          台阶，边缘就会进入前景。
//      两路都是尺度无关的量，与具体色相无关。目标的颜色只用于 CCL 的
//      "同色才合并"判据（见 isp_ccl_circle.v 的 COLOR_AWARE）。
//   2) 圆形判据落在【轮廓特征】上，不是宽高比：
//        · 主判据 chord_ratio = Σ(每行最左到最右前景像素的跨度) / (w*h)
//            完整圆      ≈ π/4 ≈ 0.785
//            实心/空心方块 ≈ 1.0      ← 正方形被这一条直接排除
//            三角形/菱形  ≈ 0.5       ← 被下限排除
//          该量与"轮廓是否闭合"无关：实心圆与空心圆环得到同一个比值，
//          所以灰白圆（只有边缘）与彩色圆（整片色度）用同一套阈值。
//        · 宽高比、有效行连续性、尺寸门限作为辅助条件。
//   3) 逐行跨度来自 isp_span_acc，取 MIN_HIT=1（轮廓可能只有 1 像素宽，
//      不能用原来的"每拍至少 2 命中"腐蚀）。
// ==========================================================================

// --------------------------------------------------------------------------
// 前景掩膜：背景色差 ∪ 亮度梯度（本轮改为背景自适应）
// --------------------------------------------------------------------------
// 为什么不再用"绝对色度"：
//   旧判据 foreground = (max(R,G,B)-min(R,G,B) >= 20) ∪ (梯度 >= 48)。
//   只要背景本身有颜色（蓝布、木桌、草地），色度就远超门限，整片背景都成
//   前景 -> 前景铺满画面 -> 外接框接近满幅 -> 圆形判据必然失败。
//
// 新判据：
//   foreground = ( |R-Rbg| + |G-Gbg| + |B-Bbg| >= BG_TH ) ∪ (亮度梯度 >= EDGE_TH)
//   Rbg/Gbg/Bbg 来自帧级背景估计（画面四周环带的慢速均值）。
//   只要目标颜色与**背景**不同就能分离，与目标本身是什么颜色无关。
//
// 颜色一致性 CCL 还需要"这一拍 4 个像素是什么颜色"：
//   O_rgb = 4 像素平均值量化到 5:5:5，与 O_mask 同拍。
//   只用它做连通域的同色合并判据，不参与前景判定。
//
// 两级流水（避免把 3 减法 + 3 取绝对值 + 2 加法 + 比较压在 103 MHz 一拍里）：
//   S1: 每像素算三通道绝对差之和、并寄存 5:5:5 颜色
//   S2: 与 BG_TH 比较、并与 Sobel 边缘掩膜取并
// --------------------------------------------------------------------------
module isp_fg_mask #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [9:0]   BG_TH      = 10'd60      // 三通道绝对差之和门限
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    // 像素流：必须与 edge_detect_96 实例的输入同源同拍
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    // 帧级背景色（isp_bg_est）
    input  wire [7:0]  I_bg_r,
    input  wire [7:0]  I_bg_g,
    input  wire [7:0]  I_bg_b,
    // edge_detect_96 的输出（比 I_* 晚 1 拍）
    input  wire        I_edge_valid,
    input  wire        I_edge_tlast,
    input  wire        I_edge_tuser,
    input  wire [95:0] I_edge_tdata,
    // 前景掩膜（与 I_edge_* 同拍）。bit0 对应最左像素。
    output wire [3:0]  O_mask,
    // 该拍 4 像素平均色 5:5:5（与 O_mask 同拍），供 CCL 做颜色一致性
    output wire [14:0] O_rgb,
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

    // 4 像素平均色（sum/4/8 = sum>>5），与掩膜同拍寄存
    wire [9:0] S_sum_r = I_tdata[95:88] + I_tdata[71:64] + I_tdata[47:40] + I_tdata[23:16];
    wire [9:0] S_sum_g = I_tdata[87:80] + I_tdata[63:56] + I_tdata[39:32] + I_tdata[15:8];
    wire [9:0] S_sum_b = I_tdata[79:72] + I_tdata[55:48] + I_tdata[31:24] + I_tdata[7:0];

    reg [3:0]  S1_hit;
    reg [14:0] S1_rgb;
    reg [11:0] S1_x, S1_y;
    reg [11:0] S1_xc, S1_yc;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_hit <= 4'd0;
            S1_rgb <= 15'd0;
            S1_x  <= 12'd0; S1_y  <= 12'd0;
            S1_xc <= 12'd0; S1_yc <= 12'd0;
        end
        else if (I_tvalid) begin
            S1_hit <= { (d3 >= BG_TH), (d2 >= BG_TH),
                        (d1 >= BG_TH), (d0 >= BG_TH) };
            S1_rgb <= { S_sum_r[9:5], S_sum_g[9:5], S_sum_b[9:5] };
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

    // ★ 对齐修正：颜色差分支 S1 比 Sobel/edge 输出早一拍。
    //   旧实现直接把 S1_hit/S1_rgb 与 edge 的 grad/valid/tuser 拼在一起，
    //   导致 mask 与 RGB 颜色相对 CCL 坐标早 4 像素（实测圆 bbox 左移一格），
    //   颜色一致性 CCL 也会用"下一拍"的 RGB 去判当前格。
    //   这里把 S1 再打一拍（S2），与 edge 输出同拍；O_valid/tuser/tlast
    //   仍来自 edge，mask/RGB/x/y 全部对齐到同一拍。
    reg [3:0]  S2_hit;
    reg [14:0] S2_rgb;
    reg [11:0] S2_x, S2_y;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S2_hit <= 4'd0;
            S2_rgb <= 15'd0;
            S2_x   <= 12'd0;
            S2_y   <= 12'd0;
        end
        else begin
            // 每拍无条件寄存 S1：S2 与 edge 的 O_tdata 对应同一个输入拍。
            // 若用 I_edge_valid 门控，首有效拍（tuser 当拍）的 S1 会被漏采，
            // 实测首拍 marker 的 S1 已在，但 O_rgb 仍是背景（帧首 RGB 错位）。
            // 输出有效性仍由 O_valid=I_edge_valid 门控，空拍不会外泄。
            S2_hit <= S1_hit;
            S2_rgb <= S1_rgb;
            S2_x   <= S1_x;
            S2_y   <= S1_y;
        end
    end

    assign O_mask  = S2_hit | S_grad_hit;
    assign O_rgb   = S2_rgb;
    assign O_valid = I_edge_valid;
    assign O_tlast = I_edge_tlast;
    assign O_tuser = I_edge_tuser;
    assign O_x     = S2_x;
    assign O_y     = S2_y;

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

    // 场景切换时用"半步"加速收敛，稳态仍走 1/4 慢跟踪：
    //   |Δ| <= 32（光照/目标扫过边缘）保持慢速；
    //   |Δ| >  32（换背景）走半步，3 帧内收敛，避免换场景后长时间误判。
    //   例：灰(110,110,110) -> 蓝(40,80,160)，逐帧误差 150 -> 75 -> 37 < BG_TH(60)。
    function [7:0] track_bg;
        input [7:0] old;
        input [7:0] tgt;
        reg [7:0] d;
        reg [8:0] mid;
        begin
            d   = (tgt > old) ? (tgt - old) : (old - tgt);
            mid = ({1'b0, old} + {1'b0, tgt}) >> 1;
            track_bg = (d > 8'd32) ? mid[7:0] : iir4(old, tgt);
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
                O_bg_r  <= track_bg(O_bg_r, S_mean_r);
                O_bg_g  <= track_bg(O_bg_g, S_mean_g);
                O_bg_b  <= track_bg(O_bg_b, S_mean_b);
            end
        end
        else if (I_tvalid && S_border) begin
            S_acc_r <= S_acc_r + {18'd0, S_sum_r};
            S_acc_g <= S_acc_g + {18'd0, S_sum_g};
            S_acc_b <= S_acc_b + {18'd0, S_sum_b};
        end
    end

endmodule


// --------------------------------------------------------------------------
// 圆形评估：单目标，逐帧给出一个候选框 + 圆形判定
// --------------------------------------------------------------------------
module isp_circle_detect #(
    parameter [11:0] MIN_W    = 12'd24,     // 最小外接边长（像素）
    parameter [11:0] MAX_W    = 12'd720,    // 最大外接边长
    parameter [23:0] MIN_CNT  = 24'd16,     // 前景像素数噪声地板（空心轮廓也够）
    // 弦长/外接框面积 的允许区间：5/8 = 0.625 ~ 7/8 = 0.875
    //   圆 ≈ 0.785 落在区间内；方块/矩形 ≈ 1.0 被上界排除；
    //   三角形/菱形 ≈ 0.5 被下界排除。
    parameter [11:0] ROW_RATIO_NUM = 12'd9, // 有效行连续性 >= 9/10
    parameter [11:0] ROW_RATIO_DEN = 12'd10
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_clr,
    input  wire        I_valid,
    input  wire [3:0]  I_hit,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    // 帧尾单拍快照
    output reg         O_evt,
    output reg  [11:0] O_w,
    output reg  [11:0] O_h,
    output reg  [23:0] O_cnt,
    output reg  [20:0] O_span,
    output reg  [10:0] O_rows,
    output reg  [11:0] O_minx,
    output reg  [11:0] O_maxx,
    output reg  [11:0] O_miny,
    output reg  [11:0] O_maxy,
    // 判定结果（寄存，与 O_evt 同拍有效）
    output reg         O_pass,
    output reg         O_cand,
    // 逐项诊断（供 OSD / 调试）
    output reg         O_ok_size,
    output reg         O_ok_asp,
    output reg         O_ok_rows,
    output reg         O_ok_chord
);

    // 逐拍统计：MIN_HIT=1，轮廓只有 1 像素宽时也要算进跨度
    wire [23:0] S_cnt;
    wire [20:0] S_span;
    wire [10:0] S_rows;
    wire [11:0] S_w, S_h, S_minx, S_maxx, S_miny, S_maxy;
    wire        S_acc_evt;

    isp_span_acc #(
        .MIN_HIT (3'd1)
    ) u_shape_acc (
        .I_clk  (I_clk),
        .I_rst_n(I_rst_n),
        .I_valid(I_valid),
        .I_clr  (I_clr),
        .I_x    (I_x),
        .I_y    (I_y),
        .I_h    (I_hit),
        .O_w    (S_w),
        .O_h    (S_h),
        .O_cnt  (S_cnt),
        .O_span (S_span),
        .O_rows (S_rows),
        .O_minx (S_minx),
        .O_maxx (S_maxx),
        .O_miny (S_miny),
        .O_maxy (S_maxy),
        .O_evt  (S_acc_evt)
    );

    // ------------------------------------------------------------------
    // 三级流水：帧尾指标 -> 面积乘法 -> 判据。
    // 面积乘法单独占一级，避免"乘法 + 多级移位加法 + 比较"挤在一拍里
    // （否则 TD 会把乘法拆成 LUT 加法树，在 103 MHz 的像素时钟上直接违例）。
    // ------------------------------------------------------------------
    // ---- 第 1 级：锁存原始指标 ----
    reg  [11:0] S1_w, S1_h;
    reg  [11:0] S1_minx, S1_maxx, S1_miny, S1_maxy;
    reg  [23:0] S1_cnt;
    reg  [20:0] S1_span;
    reg  [10:0] S1_rows;
    reg         S1_evt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_evt  <= 1'b0;
            S1_w    <= 12'd0;  S1_h    <= 12'd0;
            S1_cnt  <= 24'd0;  S1_span <= 21'd0;  S1_rows <= 11'd0;
            S1_minx <= 12'd0;  S1_maxx <= 12'd0;
            S1_miny <= 12'd0;  S1_maxy <= 12'd0;
        end
        else begin
            S1_evt <= S_acc_evt;
            if (S_acc_evt) begin
                S1_w    <= S_w;    S1_h    <= S_h;
                S1_cnt  <= S_cnt;  S1_span <= S_span;  S1_rows <= S_rows;
                S1_minx <= S_minx; S1_maxx <= S_maxx;
                S1_miny <= S_miny; S1_maxy <= S_maxy;
            end
        end
    end

    // ---- 第 2 级：面积乘法（单独一级）----
    reg  [23:0] S2_area;
    reg         S2_evt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S2_evt  <= 1'b0;
            S2_area <= 24'd0;
        end
        else begin
            S2_evt <= S1_evt;
            if (S1_evt) S2_area <= S1_w * S1_h;
        end
    end

    // ---- 判据（输入全部已寄存）----
    wire O_ok_size_i = (S1_w >= MIN_W) && (S1_h >= MIN_W) &&
                       (S1_w <= MAX_W) && (S1_h <= MAX_W) &&
                       (S1_cnt >= MIN_CNT);

    // 宽高比 0.80 ~ 1.25（圆在弱透视、轻微遮挡下仍接近 1:1；长条被排除）
    wire O_ok_asp_i  = (({4'd0,S1_w} * 12'd5) >= ({4'd0,S1_h} * 12'd4)) &&
                       (({4'd0,S1_h} * 12'd5) >= ({4'd0,S1_w} * 12'd4));

    wire O_ok_rows_i = (({1'd0,S1_rows} * 12'd10) >= (S1_h * ROW_RATIO_NUM)) &&
                       (({4'd0,S1_h}    * ROW_RATIO_DEN) >= ({1'd0,S1_rows} * 12'd10));

    // 弦长总和 / 外接框面积，用 8 倍定点避免除法：5*A <= 8*S <= 7*A
    wire [27:0] S_span8 = {7'd0, S1_span} << 3;
    wire [27:0] S_area5 = ({4'd0, S2_area} << 2) + {4'd0, S2_area};
    wire [27:0] S_area7 = ({4'd0, S2_area} << 2) + ({4'd0, S2_area} << 1) + {4'd0, S2_area};
    wire O_ok_chord_i = (S_span8 >= S_area5) && (S_span8 <= S_area7);

    wire O_pass_i = O_ok_size_i && O_ok_asp_i && O_ok_rows_i && O_ok_chord_i;
    // 候选：外形像目标、尺寸也对，但轮廓不像圆（供调参观察，不点亮绿框）
    wire O_cand_i = O_ok_size_i && O_ok_asp_i && O_ok_rows_i && !O_ok_chord_i;

    // ---- 第 3 级：输出寄存（与 O_evt 对齐）----
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_evt  <= 1'b0;
            O_w    <= 12'd0;  O_h    <= 12'd0;
            O_cnt  <= 24'd0;  O_span <= 21'd0;  O_rows <= 11'd0;
            O_minx <= 12'd0;  O_maxx <= 12'd0;
            O_miny <= 12'd0;  O_maxy <= 12'd0;
            O_pass <= 1'b0;   O_cand <= 1'b0;
            O_ok_size <= 1'b0; O_ok_asp <= 1'b0;
            O_ok_rows <= 1'b0; O_ok_chord <= 1'b0;
        end
        else begin
            O_evt <= S2_evt;
            if (S2_evt) begin
                O_w    <= S1_w;    O_h    <= S1_h;
                O_cnt  <= S1_cnt;  O_span <= S1_span;  O_rows <= S1_rows;
                O_minx <= S1_minx; O_maxx <= S1_maxx;
                O_miny <= S1_miny; O_maxy <= S1_maxy;
                O_pass <= O_pass_i; O_cand <= O_cand_i;
                O_ok_size <= O_ok_size_i; O_ok_asp <= O_ok_asp_i;
                O_ok_rows <= O_ok_rows_i; O_ok_chord <= O_ok_chord_i;
            end
        end
    end

endmodule
