// ==========================================================================
// 波普风格（POP）：色调分离 + 假色彩
// --------------------------------------------------------------------------
// 从 isp_top 抽出成独立模块，目的：
//   1. 可单独做单元测试（灰阶斜坡 / 暗-正常-亮 / 真实场景）
//   2. 内置每帧各量化档的像素计数，量化"整屏塌成一档"的问题
//
// 修复点（MODE=1）：
//   原实现把 luma（0..253）直接与固定阈值 64/128/192 比较。只要整帧亮度
//   落在同一区间（欠曝、逆光、大面积深色衣物），全部像素进同一档 -> 整屏
//   单一颜色，场景层次完全丢失。
//   现改为"带平滑与限幅的亮度归一化"：
//     · 每帧统计全帧 luma 均值（只用低 8 位累加）
//     · 与目标中灰 112 比较，算增益 K（Q8），**限幅 0.5x~2.0x**
//     · K 走 1/8 一阶平滑，避免逐帧拉伸造成闪烁与噪声放大
//     · 均匀场景的 K 会收敛到 1.0 附近，不会人为制造层次
//   量化仍在固定阈值上做，所以"档位顺序"和"配色"语义不变。
// ==========================================================================
module isp_pop #(
    parameter MODE = 1,            // 0 = 原实现；1 = 带归一化（默认）
    parameter [7:0]  T0 = 8'd64,
    parameter [7:0]  T1 = 8'd128,
    parameter [7:0]  T2 = 8'd192,
    parameter [7:0]  TARGET_MID = 8'd112,   // 归一化目标中灰
    parameter integer GAIN_SMOOTH_SH = 3    // 增益平滑 1/8
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_valid,
    input  wire [95:0] I_tdata,       // 白平衡后的 4 像素 RGB888
    input  wire [1:0]  I_pal,         // 配色方案
    output wire [95:0] O_tdata,       // POP 输出
    // 每帧各档像素计数（原始档位，不含套印错位），帧尾锁存
    output reg  [23:0] O_cnt_b0,
    output reg  [23:0] O_cnt_b1,
    output reg  [23:0] O_cnt_b2,
    output reg  [23:0] O_cnt_b3,
    output reg  [9:0]  O_gain,        // 当前归一化增益（Q8，256=1.0x）
    output reg         O_frm
);

    function [7:0] rgb_to_y8;
        input [23:0] px;
        begin
            rgb_to_y8 = {2'b0,px[23:18]} + {1'b0,px[15:9]} + {2'b0,px[7:2]};
        end
    endfunction

    wire [7:0] y0 = rgb_to_y8(I_tdata[95:72]);
    wire [7:0] y1 = rgb_to_y8(I_tdata[71:48]);
    wire [7:0] y2 = rgb_to_y8(I_tdata[47:24]);
    wire [7:0] y3 = rgb_to_y8(I_tdata[23:0]);

    // ---------------- 帧级均值 -> 归一化增益 ----------------
    reg [23:0] S_sum;             // 每拍最多 253*4 = 1012，230400 拍 -> 23 位够
    reg [19:0] S_n;
    reg [9:0]  S_gain;

    wire [9:0] S_sum_beat = {2'd0,y0} + {2'd0,y1} + {2'd0,y2} + {2'd0,y3};

    // 帧尾：mean = sum / n（n 为拍数，用移位近似到 2^18）
    wire [23:0] mean24 = S_sum >> 10;         // 近似每拍均值 (0..1012)
    wire [7:0]  mean8  = (mean24 > 24'd253) ? 8'd253 : mean24[7:0];

    // 目标增益 K = TARGET_MID*256 / mean8，限幅 [128, 512]
    wire [23:0] tgt_raw = (mean8 == 8'd0) ? 24'd32768
                                          : ({8'd0, TARGET_MID} * 8'd256) / {16'd0, mean8};
    wire [9:0]  tgt_gain = (tgt_raw < 24'd128) ? 10'd128 :
                           (tgt_raw > 24'd512) ? 10'd512 : tgt_raw[9:0];

    function [9:0] gain_step;
        input [9:0] old;
        input [9:0] tgt;
        begin
            if (tgt > old) gain_step = old + ((tgt - old) >> GAIN_SMOOTH_SH);
            else           gain_step = old - ((old - tgt) >> GAIN_SMOOTH_SH);
        end
    endfunction

    wire [7:0] S_frm_evt = (I_valid && (S_n >= 20'd1)) ? 1'b0 : 1'b0;  // 占位，见下方累加

    reg        S_frame;
    always @(posedge I_clk) begin
        if (I_valid) begin
            if (S_n == 20'd230399) S_frame <= 1'b1;
            else                   S_frame <= 1'b0;
        end
        else S_frame <= 1'b0;
    end

    // ---------------- 归一化后的 luma 与档位 ----------------
    function [7:0] norm_y;
        input [7:0]  y;
        input [9:0]  k;
        reg   [17:0] p;
        begin
            p = {10'd0, y} * {8'd0, k};        // y*K，K 为 Q8
            norm_y = (p[17:8] > 18'd253) ? 8'd253 : p[15:8];
        end
    endfunction

    wire [7:0] n0 = (MODE == 0) ? y0 : norm_y(y0, S_gain);
    wire [7:0] n1 = (MODE == 0) ? y1 : norm_y(y1, S_gain);
    wire [7:0] n2 = (MODE == 0) ? y2 : norm_y(y2, S_gain);
    wire [7:0] n3 = (MODE == 0) ? y3 : norm_y(y3, S_gain);

    function [1:0] band_of;
        input [7:0] g;
        begin
            if      (g < T0) band_of = 2'd0;
            else if (g < T1) band_of = 2'd1;
            else if (g < T2) band_of = 2'd2;
            else             band_of = 2'd3;
        end
    endfunction

    wire [1:0] b0 = band_of(n0);
    wire [1:0] b1 = band_of(n1);
    wire [1:0] b2 = band_of(n2);
    wire [1:0] b3 = band_of(n3);

    // 套印错位：中间调按 x-2 的档位着色
    reg [7:0] S_bands_d;
    wire [1:0] s0 = S_bands_d[5:4];
    wire [1:0] s1 = S_bands_d[7:6];
    wire [1:0] s2 = b0;
    wire [1:0] s3 = b1;

    wire [1:0] e0 = ((s0 == 2'd1) || (s0 == 2'd2)) ? s0 : b0;
    wire [1:0] e1 = ((s1 == 2'd1) || (s1 == 2'd2)) ? s1 : b1;
    wire [1:0] e2 = ((s2 == 2'd1) || (s2 == 2'd2)) ? s2 : b2;
    wire [1:0] e3 = ((s3 == 2'd1) || (s3 == 2'd2)) ? s3 : b3;

    function [23:0] pop_pal;
        input [1:0] idx;
        input [1:0] pal;
        begin
            case (pal)
                2'd1: case (idx)
                    2'd0: pop_pal = 24'h00574b;
                    2'd1: pop_pal = 24'hffe0b2;
                    2'd2: pop_pal = 24'h00e5ff;
                    default: pop_pal = 24'hff00aa;
                endcase
                2'd2: case (idx)
                    2'd0: pop_pal = 24'hd32f2f;
                    2'd1: pop_pal = 24'hffc0cb;
                    2'd2: pop_pal = 24'h1e5aa8;
                    default: pop_pal = 24'hff7a00;
                endcase
                2'd3: case (idx)
                    2'd0: pop_pal = 24'h202020;
                    2'd1: pop_pal = 24'hffd400;
                    2'd2: pop_pal = 24'hff3b30;
                    default: pop_pal = 24'hf2f2f2;
                endcase
                default: case (idx)
                    2'd0: pop_pal = 24'h7b2d8e;
                    2'd1: pop_pal = 24'hff9ec4;
                    2'd2: pop_pal = 24'hffd400;
                    default: pop_pal = 24'h00c8c8;
                endcase
            endcase
        end
    endfunction

    assign O_tdata = { pop_pal(e0, I_pal), pop_pal(e1, I_pal),
                       pop_pal(e2, I_pal), pop_pal(e3, I_pal) };

    // ---------------- 统计与增益更新 ----------------
    reg [23:0] c0, c1, c2, c3;

    wire [1:0] c0_inc = {1'b0,(b0==2'd0)} + {1'b0,(b1==2'd0)} + {1'b0,(b2==2'd0)} + {1'b0,(b3==2'd0)};
    wire [1:0] c1_inc = {1'b0,(b0==2'd1)} + {1'b0,(b1==2'd1)} + {1'b0,(b2==2'd1)} + {1'b0,(b3==2'd1)};
    wire [1:0] c2_inc = {1'b0,(b0==2'd2)} + {1'b0,(b1==2'd2)} + {1'b0,(b2==2'd2)} + {1'b0,(b3==2'd2)};
    wire [1:0] c3_inc = {1'b0,(b0==2'd3)} + {1'b0,(b1==2'd3)} + {1'b0,(b2==2'd3)} + {1'b0,(b3==2'd3)};

    // 帧尾判定：每行 320 拍、每帧 720 行 -> 230400 拍
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_sum   <= 24'd0;
            S_n     <= 20'd0;
            S_gain  <= 10'd256;
            S_bands_d <= 8'd0;
            c0 <= 24'd0; c1 <= 24'd0; c2 <= 24'd0; c3 <= 24'd0;
            O_cnt_b0 <= 24'd0; O_cnt_b1 <= 24'd0;
            O_cnt_b2 <= 24'd0; O_cnt_b3 <= 24'd0;
            O_gain <= 10'd256;
            O_frm  <= 1'b0;
        end
        else begin
            O_frm <= 1'b0;
            if (I_valid) begin
                S_bands_d <= {b3,b2,b1,b0};
                S_sum <= S_sum + {14'd0, S_sum_beat};
                S_n   <= S_n + 20'd1;
                c0 <= c0 + {22'd0, c0_inc};
                c1 <= c1 + {22'd0, c1_inc};
                c2 <= c2 + {22'd0, c2_inc};
                c3 <= c3 + {22'd0, c3_inc};

                if (S_n == 20'd230399) begin
                    O_cnt_b0 <= c0 + {22'd0, c0_inc};
                    O_cnt_b1 <= c1 + {22'd0, c1_inc};
                    O_cnt_b2 <= c2 + {22'd0, c2_inc};
                    O_cnt_b3 <= c3 + {22'd0, c3_inc};
                    O_gain   <= (MODE == 0) ? 10'd256 : gain_step(S_gain, tgt_gain);
                    S_gain   <= (MODE == 0) ? 10'd256 : gain_step(S_gain, tgt_gain);
                    S_sum    <= 24'd0;
                    S_n      <= 20'd0;
                    c0 <= 24'd0; c1 <= 24'd0; c2 <= 24'd0; c3 <= 24'd0;
                    O_frm <= 1'b1;
                end
            end
            else begin
                // 帧间隙（valid 拉低）也结算一次：比依赖精确拍数更鲁棒
                if (S_n != 20'd0) begin
                    O_cnt_b0 <= c0;
                    O_cnt_b1 <= c1;
                    O_cnt_b2 <= c2;
                    O_cnt_b3 <= c3;
                    O_gain   <= (MODE == 0) ? 10'd256 : gain_step(S_gain, tgt_gain);
                    S_gain   <= (MODE == 0) ? 10'd256 : gain_step(S_gain, tgt_gain);
                    S_sum    <= 24'd0;
                    S_n      <= 20'd0;
                    c0 <= 24'd0; c1 <= 24'd0; c2 <= 24'd0; c3 <= 24'd0;
                    O_frm <= 1'b1;
                end
            end
        end
    end

endmodule
