// ==========================================================================
// Display / foreground Sobel core fed by the shared 4-pixel luma stream.
// --------------------------------------------------------------------------
// This is the original edge_detect_96 datapath with the RGB->Y conversion
// removed.  The algorithm is intentionally UNCHANGED:
//   * two horizontal samples per 4-pixel beat, each the average of a
//     neighbouring pair ((Y0+Y1)>>1 and (Y2+Y3)>>1);
//   * one vertical line buffer (the previous row), not a full 2D window;
//   * 3x3 Sobel on those pair samples, |Gx|+|Gy| >= EDGE_TH;
//   * two pipeline stages (window capture, then Sobel/compare).
// Therefore the display EDGE pixel pattern is bit-identical to the baseline.
// Do not describe this as full-resolution per-pixel Sobel.
//
// I_y4 = {Y3,Y2,Y1,Y0}, Y0 is the least-significant (right-most) pixel,
// matching isp_rgb2y96.  The caller supplies aligned valid/tuser/tlast.
// ==========================================================================
module edge_detect_y96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH    = 11'd80
)(
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input      [31:0]   I_y4,
    input               I_tvalid,
    input               I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    output              O_tready
);

assign O_tready = I_tready;

wire [7:0] gray0 = I_y4[31:24];
wire [7:0] gray1 = I_y4[23:16];
wire [7:0] gray2 = I_y4[15:8];
wire [7:0] gray3 = I_y4[7:0];

localparam IMG_WIDTH_GRP = (IMG_WIDTH >> 2);

// Use two grayscale samples per 4-pixel input beat.  Each sample represents
// a neighbouring pair of pixels, providing 640 horizontal edge samples while
// keeping the two line buffers small enough for this device.
wire [8:0] pair_sum0 = {1'b0,gray0} + {1'b0,gray1};
wire [8:0] pair_sum1 = {1'b0,gray2} + {1'b0,gray3};
wire [7:0] pair0 = pair_sum0[8:1];
wire [7:0] pair1 = pair_sum1[8:1];

(* ram_style = "block" *) reg [15:0] line1_mem [0:IMG_WIDTH_GRP-1];
(* ram_style = "block" *) reg [15:0] line2_mem [0:IMG_WIDTH_GRP-1];

reg [10:0] x_grp;
reg [10:0] y_pos;

wire [15:0] top_word = line2_mem[x_grp];
wire [15:0] mid_word = line1_mem[x_grp];
wire [15:0] bot_word = {pair0,pair1};

wire [7:0] top0 = top_word[15:8];
wire [7:0] top1 = top_word[7:0];
wire [7:0] mid0 = mid_word[15:8];
wire [7:0] mid1 = mid_word[7:0];

reg [7:0] top_prev2, top_prev1;
reg [7:0] mid_prev2, mid_prev1;
reg [7:0] bot_prev2, bot_prev1;

function [13:0] sobel_mag;
    input [7:0] a00, a01, a02;
    input [7:0] a10, a11, a12;
    input [7:0] a20, a21, a22;
    reg signed [12:0] gx_f;
    reg signed [12:0] gy_f;
    reg [12:0] abs_gx_f;
    reg [12:0] abs_gy_f;
    begin
        gx_f = -$signed({1'b0,a00}) + $signed({1'b0,a02})
             - ($signed({1'b0,a10}) <<< 1) + ($signed({1'b0,a12}) <<< 1)
             - $signed({1'b0,a20}) + $signed({1'b0,a22});
        gy_f =  $signed({1'b0,a00}) + ($signed({1'b0,a01}) <<< 1) + $signed({1'b0,a02})
             - $signed({1'b0,a20}) - ($signed({1'b0,a21}) <<< 1) - $signed({1'b0,a22});
        abs_gx_f = gx_f[12] ? (~gx_f + 1'b1) : gx_f;
        abs_gy_f = gy_f[12] ? (~gy_f + 1'b1) : gy_f;
        sobel_mag = abs_gx_f + abs_gy_f;
    end
endfunction

// Two Sobel windows are evaluated per input beat.  Each result is duplicated
// to its corresponding pair of output pixels.
//
// 流水化：第 1 级捕获 3x3 窗口（RAM 读 + 行内滑动），第 2 级做 Sobel 与比较。
// 总 latency 2 拍（相对 I_tvalid）；顶层在它前面再放 1 拍共享灰度寄存器，
// 因此显示 EDGE 相对 AWB 输出仍是原来的 3 拍。
wire window_valid = (y_pos >= 11'd2) && (x_grp != 11'd0) && !I_tuser;

reg [7:0] w_t2, w_t1, w_t0, w_tp1;
reg [7:0] w_m2, w_m1, w_m0, w_mp1;
reg [7:0] w_b2, w_b1, w_p0, w_p1;
reg       w_winvalid, w_valid;
reg       O_tlast_s1, O_tuser_s1, O_tvalid_s1;

wire [13:0] mag0 = sobel_mag(w_t2,w_t1,w_t0,
                             w_m2,w_m1,w_m0,
                             w_b2,w_b1,w_p0);
wire [13:0] mag1 = sobel_mag(w_t1,w_t0,w_tp1,
                             w_m1,w_m0,w_mp1,
                             w_b1,w_p0,w_p1);

wire edge0 = w_winvalid && (mag0 >= EDGE_TH);
wire edge1 = w_winvalid && (mag1 >= EDGE_TH);

wire [23:0] out0 = edge0 ? 24'hffffff : 24'h000000;
wire [23:0] out1 = edge1 ? 24'hffffff : 24'h000000;

always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_tlast  <= 1'b0;
        O_tuser  <= 1'b0;
        O_tdata  <= 96'd0;
        O_tvalid <= 1'b0;
        O_tlast_s1 <= 1'b0; O_tuser_s1 <= 1'b0; O_tvalid_s1 <= 1'b0;
        x_grp    <= 11'd0;
        y_pos    <= 11'd0;
        top_prev2 <= 8'd0; top_prev1 <= 8'd0;
        mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
        bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        w_t2<=8'd0; w_t1<=8'd0; w_t0<=8'd0; w_tp1<=8'd0;
        w_m2<=8'd0; w_m1<=8'd0; w_m0<=8'd0; w_mp1<=8'd0;
        w_b2<=8'd0; w_b1<=8'd0; w_p0<=8'd0; w_p1<=8'd0;
        w_winvalid<=1'b0; w_valid<=1'b0;
    end else begin
        // ---- 第 1 级：控制与数据都进一级寄存器 ----
        O_tlast_s1  <= I_tlast;
        O_tuser_s1  <= I_tuser;
        O_tvalid_s1 <= I_tvalid;
        w_valid     <= I_tvalid;

        // ---- 第 2 级：输出 ----
        O_tlast  <= O_tlast_s1;
        O_tuser  <= O_tuser_s1;
        O_tvalid <= O_tvalid_s1;
        if (w_valid) O_tdata <= {out0,out0,out1,out1};
        else         O_tdata <= 96'd0;

        if(I_tvalid) begin
            // 窗口捕获（与下面的行缓存读写同拍）
            w_t2 <= top_prev2; w_t1 <= top_prev1; w_t0 <= top0; w_tp1 <= top1;
            w_m2 <= mid_prev2; w_m1 <= mid_prev1; w_m0 <= mid0; w_mp1 <= mid1;
            w_b2 <= bot_prev2; w_b1 <= bot_prev1; w_p0 <= pair0; w_p1 <= pair1;
            w_winvalid <= window_valid;

            line2_mem[x_grp] <= mid_word;
            line1_mem[x_grp] <= bot_word;

            top_prev2 <= top0; top_prev1 <= top1;
            mid_prev2 <= mid0; mid_prev1 <= mid1;
            bot_prev2 <= pair0; bot_prev1 <= pair1;

            if(I_tuser) begin
                x_grp <= 11'd1;
                y_pos <= 11'd0;
            end else if(x_grp == IMG_WIDTH_GRP - 1) begin
                x_grp <= 11'd0;
                if(y_pos == IMG_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;
                top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
            end else begin
                x_grp <= x_grp + 1'b1;
            end
        end
    end
end

endmodule
