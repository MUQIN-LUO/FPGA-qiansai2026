// ==========================================================================
// 颜色无关的圆形检测前端（替代按颜色取候选的做法）
// --------------------------------------------------------------------------
// 设计要点（对应赛题改造需求 1）：
//   1) 候选区域生成【不经过任何颜色阈值】。前景掩膜由两路"与颜色无关"的
//      判据取并集：
//        · 色度  chroma = max(R,G,B) - min(R,G,B) >= CHROMA_TH
//          只判"这个像素有彩色"，不区分红/黄/蓝/绿，任何一种饱和色都能进
//          候选，因此不再需要红、黄两套阈值。
//        · 亮度梯度  Sobel(|gx|+|gy|) >= EDGE_TH （直接复用 edge_detect_96）
//          用来覆盖灰、白、黑这类"没有色度"的圆：只要与背景有明暗台阶，
//          边缘就会进入前景。
//      两路都是尺度无关的量，与具体色相无关。
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
// 前景掩膜：色度 ∪ 亮度梯度
// --------------------------------------------------------------------------
module isp_fg_mask #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [7:0]   CHROMA_TH  = 8'd20
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    // 像素流：必须与 edge_detect_96 实例的输入同源同拍
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    // edge_detect_96 的输出（比 I_* 晚 1 拍）
    input  wire        I_edge_valid,
    input  wire        I_edge_tlast,
    input  wire        I_edge_tuser,
    input  wire [95:0] I_edge_tdata,
    // 前景掩膜（与 I_edge_* 同拍）。bit0 对应最左像素。
    output wire [3:0]  O_mask,
    output wire        O_valid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [11:0] O_x,
    output wire [11:0] O_y
);

    // 色度 = max(R,G,B) - min(R,G,B)。纯组合，8 位。
    function [7:0] chroma_of;
        input [23:0] p;
        reg [7:0] mx, mn;
        begin
            mx = (p[23:16] > p[15:8]) ? p[23:16] : p[15:8];
            mx = (mx > p[7:0]) ? mx : p[7:0];
            mn = (p[23:16] < p[15:8]) ? p[23:16] : p[15:8];
            mn = (mn < p[7:0]) ? mn : p[7:0];
            chroma_of = mx - mn;
        end
    endfunction

    // 每拍 4 像素；[95:72] 是最左（x+0），[23:0] 是最右（x+3）
    wire [7:0] c0 = chroma_of(I_tdata[95:72]);
    wire [7:0] c1 = chroma_of(I_tdata[71:48]);
    wire [7:0] c2 = chroma_of(I_tdata[47:24]);
    wire [7:0] c3 = chroma_of(I_tdata[23:0]);

    wire [3:0] S_chroma_hit = { (c3 >= CHROMA_TH), (c2 >= CHROMA_TH),
                                (c1 >= CHROMA_TH), (c0 >= CHROMA_TH) };

    // 亮度梯度掩膜：edge_detect_96 输出 0xFFFFFF / 0x000000
    wire [3:0] S_grad_hit = { |I_edge_tdata[23:0],   |I_edge_tdata[47:24],
                              |I_edge_tdata[71:48],  |I_edge_tdata[95:72] };

    reg  [3:0]  S_chroma_q;
    reg  [11:0] S_xq, S_yq;
    reg  [11:0] S_xc, S_yc;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_chroma_q <= 4'd0;
            S_xq <= 12'd0; S_yq <= 12'd0;
            S_xc <= 12'd0; S_yc <= 12'd0;
        end
        else if (I_tvalid) begin
            // 色度打一拍，正好与 edge_detect_96 的寄存输出对齐
            S_chroma_q <= S_chroma_hit;
            S_xq <= S_xc;
            S_yq <= S_yc;
            if (I_tuser) begin
                S_xc <= 12'd4;
                S_yc <= 12'd0;
            end
            else if (S_xc >= (IMG_WIDTH - 4)) begin
                S_xc <= 12'd0;
                S_yc <= (S_yc >= (IMG_HEIGHT - 1)) ? 12'd0 : S_yc + 12'd1;
            end
            else begin
                S_xc <= S_xc + 12'd4;
            end
        end
    end

    assign O_mask  = S_chroma_q | S_grad_hit;
    assign O_valid = I_edge_valid;
    assign O_tlast = I_edge_tlast;
    assign O_tuser = I_edge_tuser;
    assign O_x     = S_xq;
    assign O_y     = S_yq;

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
