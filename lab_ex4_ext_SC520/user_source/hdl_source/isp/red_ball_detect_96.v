// Independent red-ball detector, RGB888 x4, after WB and before Gamma.
// Existing yellow detector and its thresholds are not modified.
module red_ball_detect_96 #(
    parameter [23:0] MIN_PIX      = 24'd64,
    // 绝对亮度地板：只用来挡暗处噪声，不承担"红"的判据。
    // 原值 48 是拍脑袋按合成场景定的，实测暗红球球面均值只有 R=36~43（见下），
    // 48 会把整颗球挡在门外。降到 28 后仍远高于暗场噪声底。
    parameter [7:0]  MIN_RED      = 8'd28,
    // 相对色差的绝对附加项：R 必须比对应通道高出 "通道值/2^CHROMA_SHIFT + 本项"。
    parameter [9:0]  RED_MARGIN   = 10'd6,
    // 相对色差的分母指数：0 = 关闭相对项（退化为原绝对判据 R>=G+16 / R>=B+16）
    parameter [2:0]  CHROMA_SHIFT = 3'd2
)(
    input wire I_clk, I_rst_n, I_enable, I_valid,
    input wire [95:0] I_data,
    input wire [11:0] I_x, I_y,
    output reg [11:0] O_minx, O_maxx, O_miny, O_maxy,
    output reg O_valid, O_cand, O_update,
    output reg [9:0] O_count
);
    // Include red and moderately pink red; reject grey, purple and orange.
    // Widen before arithmetic. No channel multiply, division or LUT/DSP.
    //
    // 2026-10-28 改动（暗红球漏检）：原判据 R>=G+16 / R>=B+16 是**绝对**色差，
    // 而球面明暗会让整颗球的 R/G/B 一起下降 —— 绝对差不变、相对差不变，
    // 但绝对判据会随亮度一起失效。实测暗红球球面 ROI 均值：
    //   手持（暗）R036 G019 B019、浅背景（亮）R043 G022 B021，
    //   两组都是"只有 R>=48 不过，R>=G+16 / R>=B+16 只剩 1~6 的余量"。
    // 故改为**相对**色差：R >= G + G/4 + K（B 同理），K 为绝对的噪声余量。
    // 球面明暗按比例缩放，相对色差不变，因此同一套阈值覆盖亮暗两端。
    // CHROMA_SHIFT=0 时相对项消失，可精确复现旧判据用于 A/B 对比。
    function automatic red_hit;
        input [23:0] p;
        reg [9:0] r, g, b, gb, lo, chroma, need_g, need_b;
        begin
            r = {2'b0,p[23:16]}; g = {2'b0,p[15:8]}; b = {2'b0,p[7:0]};
            gb = (g >= b) ? g-b : b-g;
            lo = (g <= b) ? g : b;
            chroma = (r >= lo) ? r-lo : 10'd0;
            // G/4 用移位；SHIFT=0 时置 0，等价于旧判据
            need_g = g + ((CHROMA_SHIFT == 3'd0) ? 10'd0 : (g >> CHROMA_SHIFT))
                       + RED_MARGIN;
            need_b = b + ((CHROMA_SHIFT == 3'd0) ? 10'd0 : (b >> CHROMA_SHIFT))
                       + RED_MARGIN;
            red_hit = (r >= {2'b0,MIN_RED}) &&
                      (r >= need_g) && (r >= need_b) &&
                      ((r << 1) >= (g + g + g)) &&
                      ((r << 1) >= (b + b + b)) &&
                      ((gb + gb + gb) <= chroma);
        end
    endfunction

    wire first_pixel = I_valid && I_x == 12'd0 && I_y == 12'd0;
    // The 3x3 demosaic has invalid edge interpolation samples. In the real
    // ISP regression these form red points at x=0/1278 and y=0, stretching a
    // centred ball to width 1279. Exclude only this four-pixel red border;
    // still consume every valid beat so frame/row accounting is unchanged.
    wire colour_region = (I_x >= 12'd4) && (I_x < 12'd1276) &&
                         (I_y >= 12'd4) && (I_y < 12'd716);
    reg armed;
    reg valid_q;
    reg [3:0] hits_q;
    reg [11:0] x_q, y_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            armed <= 1'b0; valid_q <= 1'b0;
            hits_q <= 4'd0; x_q <= 12'd0; y_q <= 12'd0;
        end else begin
            if (!I_enable) armed <= 1'b0;
            else if (first_pixel) armed <= 1'b1;
            valid_q <= I_valid && I_enable && (armed || first_pixel);
            if (I_valid) begin
                // Lane [95:72] is x+0, [23:0] is x+3, as in the yellow path.
                hits_q <= colour_region ?
                          {red_hit(I_data[23:0]), red_hit(I_data[47:24]),
                           red_hit(I_data[71:48]), red_hit(I_data[95:72])} : 4'd0;
                x_q <= I_x; y_q <= I_y;
            end
        end
    end

    wire [11:0] width_s, height_s, minx_s, maxx_s, miny_s, maxy_s;
    wire [23:0] count_s;
    wire [20:0] span_s;
    wire [10:0] rows_s;
    wire frame_s;
    // A separate instance: the original yellow instance is untouched.
    isp_span_acc u_red_span (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_valid(valid_q),
        .I_clr(!I_enable || !armed), .I_x(x_q), .I_y(y_q), .I_h(hits_q),
        .O_w(width_s), .O_h(height_s), .O_cnt(count_s), .O_span(span_s),
        .O_rows(rows_s), .O_minx(minx_s), .O_maxx(maxx_s),
        .O_miny(miny_s), .O_maxy(maxy_s), .O_evt(frame_s)
    );

    reg [1:0] state;
    reg [3:0] cycles;
    reg [11:0] mul_a, w_s, h_s;
    reg [23:0] mul_b, area, n_s;
    reg [20:0] sp_s;
    reg [10:0] nr_s;
    reg [11:0] bx0, bx1, by0, by1;
    reg [2:0] confidence;
    reg enable_d;
    wire [31:0] area32 = {8'd0,area};
    wire [31:0] span32 = {11'd0,sp_s};
    wire [31:0] count32 = {8'd0,n_s};
    wire [15:0] w16 = {4'd0,w_s}, h16 = {4'd0,h_s};
    wire [15:0] rows16 = {5'd0,nr_s};
    wire [31:0] span100 = (span32 << 6) + (span32 << 5) + (span32 << 2);
    wire [31:0] count100 = (count32 << 6) + (count32 << 5) + (count32 << 2);
    wire pass_shape = (n_s >= MIN_PIX) && (w_s >= 12'd8) && (h_s >= 12'd8) &&
        (((w16 << 3)-w16) >= ((h16 << 2)+h16)) &&
        (((h16 << 3)-h16) >= ((w16 << 2)+w16)) &&
        (((rows16 << 2)+rows16) >= (h16 << 2)) &&
        (count100 >= ((area32 << 5)+(area32 << 1)+area32)) &&
        (span100 >= ((area32 << 6)-(area32 << 2))) &&
        (span100 <= ((area32 << 6)+(area32 << 5)-(area32 << 2)));
    wire [2:0] next_conf = pass_shape ?
        ((confidence == 3'd7) ? 3'd7 : confidence+3'd1) :
        ((confidence == 3'd0) ? 3'd0 : confidence-3'd1);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state <= 0; cycles <= 0; mul_a <= 0; mul_b <= 0; area <= 0;
            w_s <= 0; h_s <= 0; n_s <= 0; sp_s <= 0; nr_s <= 0;
            bx0 <= 0; bx1 <= 0; by0 <= 0; by1 <= 0;
            confidence <= 0; enable_d <= 0;
            O_minx <= 0; O_maxx <= 0; O_miny <= 0; O_maxy <= 0;
            O_valid <= 0; O_cand <= 0; O_update <= 0; O_count <= 0;
        end else begin
            enable_d <= I_enable;
            O_update <= 1'b0;
            if (!I_enable) begin
                state <= 0; confidence <= 0;
                O_valid <= 0; O_cand <= 0; O_count <= 0;
                O_minx <= 0; O_maxx <= 0; O_miny <= 0; O_maxy <= 0;
                if (enable_d) O_update <= 1'b1;
            end else begin
                case (state)
                    0: if (frame_s) begin
                        w_s <= width_s; h_s <= height_s; n_s <= count_s;
                        sp_s <= span_s; nr_s <= rows_s;
                        bx0 <= minx_s; bx1 <= maxx_s; by0 <= miny_s; by1 <= maxy_s;
                        // Twelve-cycle shift/add area, using no extra DSP.
                        mul_a <= width_s; mul_b <= {12'd0,height_s};
                        area <= 0; cycles <= 12; state <= 1;
                    end
                    1: begin
                        if (mul_a[0]) area <= area + mul_b;
                        mul_a <= {1'b0,mul_a[11:1]}; mul_b <= mul_b << 1;
                        cycles <= cycles - 1'b1;
                        if (cycles == 1) state <= 2;
                    end
                    2: begin
                        confidence <= next_conf;
                        if (!O_valid && next_conf >= 3) O_valid <= 1;
                        else if (O_valid && next_conf < 2) O_valid <= 0;
                        O_cand <= n_s >= MIN_PIX;
                        O_count <= n_s > 24'd15984 ? 10'd999 : n_s[13:4];
                        // A miss never moves an already displayed stable box.
                        if (pass_shape) begin
                            O_minx <= bx0; O_maxx <= bx1;
                            O_miny <= by0; O_maxy <= by1;
                        end
                        O_update <= 1; state <= 0;
                    end
                    default: state <= 0;
                endcase
            end
        end
    end
endmodule

// Whole-message handshake; latest staging value survives update/send collision.
// Output changes only at the HDMI frame boundary. Independent of yellow CDC.
module red_ball_snapshot_cdc (
    input wire I_src_clk, I_src_rst_n, I_src_update,
    input wire [59:0] I_src_data,
    input wire I_dst_clk, I_dst_rst_n, I_dst_frame,
    output reg [59:0] O_dst_data
);
    reg [59:0] stage, payload, received;
    reg pending, request, ack;
    (* async_reg = "true" *) reg ack1, ack2;
    (* async_reg = "true" *) reg req1, req2;
    reg req3;
    always @(posedge I_src_clk or negedge I_src_rst_n) begin
        if (!I_src_rst_n) begin
            stage <= 0; payload <= 0; pending <= 0;
            request <= 0; ack1 <= 0; ack2 <= 0;
        end else begin
            ack1 <= ack; ack2 <= ack1;
            if (I_src_update) begin stage <= I_src_data; pending <= 1; end
            if (pending && request == ack2) begin
                payload <= stage; request <= ~request;
                pending <= I_src_update;
            end
        end
    end
    always @(posedge I_dst_clk or negedge I_dst_rst_n) begin
        if (!I_dst_rst_n) begin
            req1 <= 0; req2 <= 0; req3 <= 0; ack <= 0;
            received <= 0; O_dst_data <= 0;
        end else begin
            req1 <= request; req2 <= req1; req3 <= req2;
            if (req2 != req3) begin received <= payload; ack <= req2; end
            if (I_dst_frame) O_dst_data <= received;
        end
    end
endmodule
