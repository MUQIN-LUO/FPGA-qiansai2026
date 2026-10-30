// ==========================================================================
// 灰度世界自动白平衡（Gray-World AWB）
// --------------------------------------------------------------------------
// 位置：黑电平校正 / demosaic / 色度降噪 之后，awb 乘法器与 gamma 之前。
//       统计点与 AE 测光点相同（都在 AWB 之前的同一路像素流上）。
//
// 算法：
//   1) 逐帧累加 R/G/B。只统计"未削顶、非纯黑"的像素（用 G 通道做门控，
//      8 个比较器，不引入乘法器），避免大面积过曝/死黑把均值拉偏。
//   2) 帧尾：gR = meanG / meanR、gB = meanG / meanB（Q16）。
//      G 通道按定义归一化到 1.0（绿通道做锚点）。这样：
//        · 整体亮度【不】被白平衡改变，显示器亮度完全由 AE 决定；
//        · AE 测光在 AWB 之前，两个闭环之间没有互相激励的通路。
//   3) 除零保护：分母过小 / 可用样本不足 -> 本帧不动，保持旧增益。
//   4) 增益限幅：GAIN_MIN ~ GAIN_MAX（默认 0.5x ~ 3.0x）。
//   5) 帧间平滑：一阶 EMA，new = old + (target - old)/4，避免画面闪。
//   6) 整组更新：R、B 两个增益在同一拍一起生效（O_commit 单拍脉冲），
//      下游在帧边界整组锁存，不会出现"同帧新旧混用"。
//
// 除法实现：恢复余数法，先 4 拍求整数部分（上限 4），再 8 拍求 8 位小数，
//           得 Q8 比值，再左移 8 位成 Q16。每帧只跑 2 次，不在像素通路上。
// ==========================================================================
module isp_gray_world #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    // 可用像素数下限：低于这个数认为画面几乎全黑/全白，不做估计
    parameter [27:0]  MIN_SAMPLES = 28'd20000,
    // 分母下限（>>12 之后），防止除零与极小分母
    parameter [15:0]  MIN_DEN     = 16'd64,
    parameter [19:0]  GAIN_MIN    = 20'd32768,    // 0.5x
    parameter [19:0]  GAIN_MAX    = 20'd196608,   // 3.0x
    parameter [19:0]  GAIN_INIT_R = 20'd76824,    // 1.1722x（与原来一致）
    parameter [19:0]  GAIN_INIT_B = 20'd138731,   // 2.1169x
    parameter [1:0]   SMOOTH_SH   = 2'd2          // EMA 步长 1/4
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_valid,
    input  wire [95:0] I_tdata,       // AWB 之前的 4 像素 RGB888
    output reg  [19:0] O_gain_r,      // Q16
    output reg  [19:0] O_gain_b,      // Q16
    output reg         O_commit       // 单拍：本帧整组更新完成
);

    // ---------------- 像素级统计 ----------------
    wire [7:0] p0r = I_tdata[95:88], p0g = I_tdata[87:80], p0b = I_tdata[79:72];
    wire [7:0] p1r = I_tdata[71:64], p1g = I_tdata[63:56], p1b = I_tdata[55:48];
    wire [7:0] p2r = I_tdata[47:40], p2g = I_tdata[39:32], p2b = I_tdata[31:24];
    wire [7:0] p3r = I_tdata[23:16], p3g = I_tdata[15:8],  p3b = I_tdata[7:0];

    // 门控：G 不削顶、不纯黑
    wire u0 = (p0g > 8'd4) && (p0g < 8'd250);
    wire u1 = (p1g > 8'd4) && (p1g < 8'd250);
    wire u2 = (p2g > 8'd4) && (p2g < 8'd250);
    wire u3 = (p3g > 8'd4) && (p3g < 8'd250);

    wire [9:0] beat_r = (u0 ? {2'd0,p0r} : 10'd0) + (u1 ? {2'd0,p1r} : 10'd0)
                      + (u2 ? {2'd0,p2r} : 10'd0) + (u3 ? {2'd0,p3r} : 10'd0);
    wire [9:0] beat_g = (u0 ? {2'd0,p0g} : 10'd0) + (u1 ? {2'd0,p1g} : 10'd0)
                      + (u2 ? {2'd0,p2g} : 10'd0) + (u3 ? {2'd0,p3g} : 10'd0);
    wire [9:0] beat_b = (u0 ? {2'd0,p0b} : 10'd0) + (u1 ? {2'd0,p1b} : 10'd0)
                      + (u2 ? {2'd0,p2b} : 10'd0) + (u3 ? {2'd0,p3b} : 10'd0);
    wire [2:0] beat_n = {2'b00,u0} + {2'b00,u1} + {2'b00,u2} + {2'b00,u3};

    // 行/帧坐标（与 isp_span_acc 同一套约定，不依赖 tlast 的语义）
    reg [11:0] S_x, S_y;
    wire       S_row_end = (S_x >= (IMG_WIDTH - 4));
    wire       S_frm_end = I_valid && S_row_end && (S_y >= (IMG_HEIGHT - 1));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if (I_valid) begin
            if (S_row_end) begin
                S_x <= 12'd0;
                S_y <= (S_y >= (IMG_HEIGHT - 1)) ? 12'd0 : S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd4;
            end
        end
    end

    reg [27:0] S_sum_r, S_sum_g, S_sum_b;
    reg [27:0] S_nsmp;

    // ---------------- 帧尾除法 FSM ----------------
    localparam [1:0] ST_IDLE = 2'd0, ST_DIV = 2'd1, ST_APPLY = 2'd2;

    reg [1:0]  S_st;
    reg        S_pass;        // 0 = 算 R，1 = 算 B
    reg [3:0]  S_ccnt;        // 0..11：前 4 拍整数，后 8 拍小数
    reg [16:0] S_rem;
    reg [15:0] S_den;
    reg [2:0]  S_int;
    reg [7:0]  S_frac;
    reg [15:0] S_den_b;       // B 的分母先存着
    reg [19:0] S_tgt_r, S_tgt_b;

    // 整数段（S_ccnt 0..3）：纯减法，不上移
    wire        S_is_int  = (S_ccnt < 4'd4);
    wire        S_ge_i    = (S_rem >= {1'b0, S_den});
    wire [16:0] S_rem_i_n = S_ge_i ? (S_rem - {1'b0, S_den}) : S_rem;
    // 小数段（S_ccnt 4..11）：左移一位再试减
    wire [16:0] S_rem_d   = {S_rem[15:0], 1'b0};
    wire        S_ge_d    = (S_rem_d >= {1'b0, S_den});
    wire [16:0] S_rem_d_n = S_ge_d ? (S_rem_d - {1'b0, S_den}) : S_rem_d;

    wire [16:0] S_rem_nx = S_is_int ? S_rem_i_n : S_rem_d_n;
    wire [2:0]  S_int_nx = (S_is_int && S_ge_i && (S_int != 3'd7)) ? (S_int + 3'd1) : S_int;
    wire [7:0]  S_frac_nx = S_is_int ? S_frac : {S_frac[6:0], S_ge_d};

    // Q8 比值 -> Q16 增益，限幅
    function [19:0] q8_to_gain;
        input [2:0] i;
        input [7:0] f;
        reg   [19:0] g;
        begin
            g = {1'b0, i, f, 8'd0};           // (i + f/256) << 16
            if (g < GAIN_MIN)      q8_to_gain = GAIN_MIN;
            else if (g > GAIN_MAX) q8_to_gain = GAIN_MAX;
            else                   q8_to_gain = g;
        end
    endfunction

    // EMA：向目标靠 1/4
    function [19:0] ema_step;
        input [19:0] old;
        input [19:0] tgt;
        begin
            if (tgt > old) ema_step = old + ((tgt - old) >> SMOOTH_SH);
            else           ema_step = old - ((old - tgt) >> SMOOTH_SH);
        end
    endfunction

    wire [15:0] S_num   = S_sum_g[27:12];
    wire [15:0] S_den_r = S_sum_r[27:12];
    wire [15:0] S_den_bq= S_sum_b[27:12];
    wire        S_enough = (S_nsmp >= MIN_SAMPLES) &&
                           (S_den_r >= MIN_DEN) && (S_den_bq >= MIN_DEN) &&
                           (S_num   >= MIN_DEN);

    // 帧尾必须先把算子锁存下来：累加器在同一拍被清零，
    // 除法器随后 24 拍用的是这些锁存值，不能直接读 S_sum_*。
    reg [15:0] S_num_q, S_den_r_q, S_den_b_q;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_sum_r <= 28'd0; S_sum_g <= 28'd0; S_sum_b <= 28'd0;
            S_nsmp  <= 28'd0;
            S_st    <= ST_IDLE;
            S_pass  <= 1'b0;
            S_ccnt  <= 4'd0;
            S_rem   <= 17'd0;
            S_den   <= 16'd1;
            S_int   <= 3'd0;
            S_frac  <= 8'd0;
            S_den_b <= 16'd1;
            S_tgt_r <= GAIN_INIT_R;
            S_tgt_b <= GAIN_INIT_B;
            S_num_q <= 16'd0;
            S_den_r_q <= 16'd1;
            S_den_b_q <= 16'd1;
            O_gain_r <= GAIN_INIT_R;
            O_gain_b <= GAIN_INIT_B;
            O_commit <= 1'b0;
        end
        else begin
            O_commit <= 1'b0;

            // ---- 累加 ----
            if (I_valid && (S_st == ST_IDLE)) begin
                S_sum_r <= S_sum_r + {18'd0, beat_r};
                S_sum_g <= S_sum_g + {18'd0, beat_g};
                S_sum_b <= S_sum_b + {18'd0, beat_b};
                S_nsmp  <= S_nsmp + {25'd0, beat_n};
            end

            case (S_st)
                ST_IDLE: begin
                    if (S_frm_end) begin
                        S_sum_r <= 28'd0;
                        S_sum_g <= 28'd0;
                        S_sum_b <= 28'd0;
                        S_nsmp  <= 28'd0;
                        if (S_enough) begin
                            // 先把算子锁存下来（累加器在这一拍同时被清零）
                            S_num_q   <= S_num;
                            S_den_r_q <= S_den_r;
                            S_den_b_q <= S_den_bq;
                            // 启动 R：num = ΣG>>12，den = ΣR>>12
                            S_rem   <= {1'b0, S_num};
                            S_den   <= S_den_r;
                            S_den_b <= S_den_bq;
                            S_int   <= 3'd0;
                            S_frac  <= 8'd0;
                            S_ccnt  <= 4'd0;
                            S_pass  <= 1'b0;
                            S_st    <= ST_DIV;
                        end
                        // 样本不足：不动，保持旧增益（无 commit）
                    end
                end

                ST_DIV: begin
                    S_rem  <= S_rem_nx;
                    S_int  <= S_int_nx;
                    S_frac <= S_frac_nx;
                    S_ccnt <= S_ccnt + 4'd1;
                    if (S_ccnt == 4'd11) begin
                        if (!S_pass) begin
                            S_tgt_r <= q8_to_gain(S_int_nx, S_frac_nx);
                            // 接着算 B（用帧尾锁存的算子，不能用已清零的累加器）
                            S_rem   <= {1'b0, S_num_q};
                            S_den   <= S_den_b_q;
                            S_int   <= 3'd0;
                            S_frac  <= 8'd0;
                            S_ccnt  <= 4'd0;
                            S_pass  <= 1'b1;
                        end
                        else begin
                            S_tgt_b <= q8_to_gain(S_int_nx, S_frac_nx);
                            S_st    <= ST_APPLY;
                        end
                    end
                end

                ST_APPLY: begin
                    // 整组更新：两个增益同一拍提交
                    O_gain_r <= ema_step(O_gain_r, S_tgt_r);
                    O_gain_b <= ema_step(O_gain_b, S_tgt_b);
                    O_commit <= 1'b1;
                    S_st     <= ST_IDLE;
                end

                default: S_st <= ST_IDLE;
            endcase
        end
    end

endmodule
