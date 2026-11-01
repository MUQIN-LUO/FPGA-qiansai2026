// ==========================================================================
// Ball-interior colour sampler + green classifier (CSI domain).
// --------------------------------------------------------------------------
// The circle detector is colour-agnostic.  This block only decides whether the
// tracked ball looks green, for the optional "GREEN" label.
//
// Input is the existing post-AWB / pre-gamma 8-bit RGB beat sums:
//   S_beatq_r/g/b = sum of the 4 pixels in the 4-pixel beat (10-bit),
//   S_xq/S_yq     = beat coordinate.  The HUD centre ROI uses the same data.
//
// A 4x4 grid of beats (16 beats = 64 pixels) is sampled inside the tracked
// circle at offsets +/-r/2 and +/-r/4 in x and y (all inside r).  The grid is
// latched at the frame start from the tracker's last fresh box, so the colour
// follows the ball instead of a fixed centre ROI.
//
// Green decision (preliminary; board tuning is still required):
//   G > R, G >= B+2, (G-R)*8 >= R+G+B, G >= 6
// This is relative/low-brightness friendly and rejects neutral grey:
//   (12,23,17)/(22,45,35)/(15,30,22) pass; (10,10,10)/(10,13,10) fail.
// A 0..3 counter gives 2-sample hysteresis (2 green samples to set,
// 2 non-green to clear).
// ==========================================================================
module isp_green_sample #(
    parameter integer IMG_W = 1280,
    parameter integer IMG_H = 720
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        beat_valid,   // S_det_vq
    input  wire [11:0] beat_x, beat_y,
    input  wire [9:0]  beat_r, beat_g, beat_b,
    input  wire [11:0] trk_cx, trk_cy, trk_r,
    input  wire        trk_ok,
    input  wire        id_chg,       // tracker: target identity changed
    output reg         green
);
    localparam [11:0] LAST_X = IMG_W - 4;
    localparam [11:0] LAST_Y = IMG_H - 1;

    wire fstart = beat_valid && (beat_x == 12'd0) && (beat_y == 12'd0);
    wire fend   = beat_valid && (beat_x == LAST_X) && (beat_y == LAST_Y);

    reg [11:0] tx0, tx1, tx2, tx3;
    reg [11:0] ty0, ty1, ty2, ty3;
    reg        grid_ok;
    reg [15:0] sum_r, sum_g, sum_b;
    reg [4:0]  cnt;
    reg [1:0]  green_cnt;

    wire [11:0] off1 = trk_r >> 1;
    wire [11:0] off2 = trk_r >> 2;
    wire signed [12:0] sx0 = $signed({1'b0,trk_cx}) - $signed({1'b0,off1});
    wire signed [12:0] sx1 = $signed({1'b0,trk_cx}) - $signed({1'b0,off2});
    wire signed [12:0] sx2 = $signed({1'b0,trk_cx}) + $signed({1'b0,off2});
    wire signed [12:0] sx3 = $signed({1'b0,trk_cx}) + $signed({1'b0,off1});
    wire signed [12:0] sy0 = $signed({1'b0,trk_cy}) - $signed({1'b0,off1});
    wire signed [12:0] sy1 = $signed({1'b0,trk_cy}) - $signed({1'b0,off2});
    wire signed [12:0] sy2 = $signed({1'b0,trk_cy}) + $signed({1'b0,off2});
    wire signed [12:0] sy3 = $signed({1'b0,trk_cy}) + $signed({1'b0,off1});
    wire x_ok = (sx0[12]==1'b0) && (sx0 <= 13'd1276) &&
                (sx1[12]==1'b0) && (sx1 <= 13'd1276) &&
                (sx2[12]==1'b0) && (sx2 <= 13'd1276) &&
                (sx3[12]==1'b0) && (sx3 <= 13'd1276);
    wire y_ok = (sy0[12]==1'b0) && (sy0 <= 13'd719) &&
                (sy1[12]==1'b0) && (sy1 <= 13'd719) &&
                (sy2[12]==1'b0) && (sy2 <= 13'd719) &&
                (sy3[12]==1'b0) && (sy3 <= 13'd719);

    wire hit_x = (beat_x == tx0) || (beat_x == tx1) ||
                 (beat_x == tx2) || (beat_x == tx3);
    wire hit_y = (beat_y == ty0) || (beat_y == ty1) ||
                 (beat_y == ty2) || (beat_y == ty3);

    wire [9:0] avg_r = sum_r >> 6;
    wire [9:0] avg_g = sum_g >> 6;
    wire [9:0] avg_b = sum_b >> 6;
    wire [9:0] sum3  = avg_r + avg_g + avg_b;
    wire green_now = (avg_g > avg_r) &&
                     (avg_g >= (avg_b + 10'd2)) &&
                     ((({1'b0,avg_g} - {1'b0,avg_r}) << 3) >= sum3) &&
                     (avg_g >= 10'd6);
    wire [1:0] next_cnt = green_now ?
                          ((green_cnt == 2'd3) ? 2'd3 : green_cnt + 2'd1) :
                          ((green_cnt == 2'd0) ? 2'd0 : green_cnt - 2'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx0<=12'd0; tx1<=12'd0; tx2<=12'd0; tx3<=12'd0;
            ty0<=12'd0; ty1<=12'd0; ty2<=12'd0; ty3<=12'd0;
            grid_ok<=1'b0; sum_r<=16'd0; sum_g<=16'd0; sum_b<=16'd0;
            cnt<=5'd0; green_cnt<=2'd0; green<=1'b0;
        end
        else if (id_chg) begin
            // target identity changed: discard any in-flight/old colour
            green<=1'b0; green_cnt<=2'd0; grid_ok<=1'b0;
            sum_r<=16'd0; sum_g<=16'd0; sum_b<=16'd0; cnt<=5'd0;
        end
        else begin
            if (fstart) begin
                sum_r<=16'd0; sum_g<=16'd0; sum_b<=16'd0; cnt<=5'd0;
                grid_ok <= trk_ok && (trk_r > 12'd8) && x_ok && y_ok;
                tx0 <= {sx0[11:2], 2'b00};
                tx1 <= {sx1[11:2], 2'b00};
                tx2 <= {sx2[11:2], 2'b00};
                tx3 <= {sx3[11:2], 2'b00};
                ty0 <= sy0[11:0];
                ty1 <= sy1[11:0];
                ty2 <= sy2[11:0];
                ty3 <= sy3[11:0];
                if (!trk_ok) begin
                    green_cnt<=2'd0; green<=1'b0;
                end
            end
            else if (beat_valid && !fend && grid_ok && hit_x && hit_y) begin
                sum_r <= sum_r + {6'd0, beat_r};
                sum_g <= sum_g + {6'd0, beat_g};
                sum_b <= sum_b + {6'd0, beat_b};
                cnt   <= cnt + 5'd1;
            end
            else if (fend) begin
                if (grid_ok && (cnt == 5'd16)) begin
                    green_cnt <= next_cnt;
                    // 2 green samples to set, 2 non-green to clear
                    if (!green && (next_cnt >= 2'd2))      green <= 1'b1;
                    else if (green && (next_cnt == 2'd0)) green <= 1'b0;
                end
                else begin
                    // edge/partial grid: no valid sample, never keep old GREEN
                    green_cnt <= 2'd0;
                    green     <= 1'b0;
                end
                grid_ok <= 1'b0;
            end
        end
    end
endmodule
