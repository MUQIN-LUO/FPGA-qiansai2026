#!/usr/bin/env python3
# Generate the phase-2 grayscale->gradient-event front end.
import os

recip = [0 if m == 0 else int(round(65536.0 / m)) for m in range(256)]
cases = "\n".join(
    f"                8'd{m}: recip8 = 17'd{recip[m]};" for m in range(256)
)

code = r'''// ==========================================================================
// Phase-2 research front end: O_dg_y4 (4xY8/beat) -> 4x4-block gradient events.
// --------------------------------------------------------------------------
// Real signed 1x1 Sobel on the shared full-resolution luma beat:
//   * uses the two middle pixels x+1,x+2 of each beat with a full 3x3 window;
//     NOT the display edge's horizontal 2-pixel average;
//   * one event per 4x4 block (strongest of 4 rows x 2 pixels);
//   * Q8 unit gradient (ux,uy) and a 16-bin sector;
//   * event cap is enforced in the detector core (EVMAX).
// NOT placed/routed or put on board.  Line buffers use block RAM and one
// registered read beat of latency.
// ==========================================================================
module isp_grad_event96 #(
    parameter integer IMG_W   = 1280,
    parameter integer IMG_H   = 720,
    parameter [10:0]  EDGE_TH = 11'd48
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] I_y4,          // {Y3,Y2,Y1,Y0}, Y3=x+0, Y0=x+3
    input  wire        I_valid,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire [11:0] I_x,           // x of x+0 (0,4,...,1276)
    input  wire [11:0] I_y,           // row 0..719
    output reg         O_ev_valid,
    output reg  [11:0] O_ev_x,
    output reg  [11:0] O_ev_y,
    output reg  signed [9:0] O_ev_ux,
    output reg  signed [9:0] O_ev_uy,
    output reg  [3:0]  O_ev_sector,
    output reg         O_frame_start,
    output reg         O_frame_end
);
    localparam integer BW = 320;         // 4-pixel beats per row

    // 65536/mag8; reciprocal for Q8 direction normalization (no divider).
    function [16:0] recip8;
        input [7:0] m;
        begin
            case (m)
__CASES__
                default: recip8 = 17'd0;
            endcase
        end
    endfunction

    // 16-bin sector from the signed gradient (coverage only; any monotonic
    // mapping that covers 16 bins for a full circle is sufficient).
    function [3:0] sector16;
        input signed [12:0] gx;
        input signed [12:0] gy;
        reg [12:0] ax, ay, minor, major;
        reg [2:0] oct;
        reg sub;
        begin
            ax = gx[12] ? (~gx + 13'd1) : gx;
            ay = gy[12] ? (~gy + 13'd1) : gy;
            if (ax >= ay) begin minor=ay; major=ax; end
            else          begin minor=ax; major=ay; end
            sub = (((minor << 3) + (minor << 2)) >= ((major << 2) + major));
            if (ax >= ay) begin
                if (!gx[12] && !gy[12]) oct=3'd0;
                else if ( gx[12] && !gy[12]) oct=3'd3;
                else if ( gx[12] &&  gy[12]) oct=3'd4;
                else                         oct=3'd7;
            end
            else begin
                if (!gx[12] && !gy[12]) oct=3'd1;
                else if ( gx[12] && !gy[12]) oct=3'd2;
                else if ( gx[12] &&  gy[12]) oct=3'd5;
                else                         oct=3'd6;
            end
            sector16 = {oct, sub};
        end
    endfunction

    function signed [9:0] clip10;
        input signed [23:0] v;
        begin
            if (v > 24'sd255) clip10 = 10'sd255;
            else if (v < -24'sd256) clip10 = -10'sd256;
            else clip10 = v[9:0];
        end
    endfunction

    // ---------------- line buffers (one registered read beat) -------------
    (* ram_style = "block" *) reg [31:0] lb1 [0:BW-1];
    (* ram_style = "block" *) reg [31:0] lb2 [0:BW-1];

    reg [31:0] top_q, mid_q, bot_q;
    reg [10:0] idx_q;
    reg [11:0] x_q, y_q;
    reg        v_q, tlast_q;
    wire [10:0] idx = I_x[10:2];
    wire [31:0] top_w = lb2[idx];
    wire [31:0] mid_w = lb1[idx];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            top_q<=0; mid_q<=0; bot_q<=0; idx_q<=0; x_q<=0; y_q<=0;
            v_q<=0; tlast_q<=0;
        end
        else begin
            v_q <= I_valid;
            if (I_valid) begin
                top_q <= top_w; mid_q <= mid_w; bot_q <= I_y4;
                lb1[idx] <= I_y4;
                lb2[idx] <= lb1[idx];
                idx_q <= idx; x_q <= I_x; y_q <= I_y;
                tlast_q <= I_tlast;
            end
            else tlast_q <= 0;
        end
    end

    // ---------------- 4x4 block accumulators ------------------------------
    (* ram_style = "block" *) reg [10:0] blk_mag  [0:BW-1];
    (* ram_style = "block" *) reg signed [12:0] blk_gx [0:BW-1];
    (* ram_style = "block" *) reg signed [12:0] blk_gy [0:BW-1];
    reg        blk_subx [0:BW-1];
    reg [1:0]  blk_suby [0:BW-1];
    reg        blk_v    [0:BW-1];
    integer k;

    // Sobel for p1 (x+1) and p2 (x+2) on the middle row (y-1).
    wire [7:0] t0=top_q[31:24], t1=top_q[23:16], t2=top_q[15:8], t3=top_q[7:0];
    wire [7:0] m0=mid_q[31:24], m1=mid_q[23:16], m2=mid_q[15:8], m3=mid_q[7:0];
    wire [7:0] b0=bot_q[31:24], b1=bot_q[23:16], b2=bot_q[15:8], b3=bot_q[7:0];

    function signed [12:0] sob_gx;
        input [7:0] a00,a01,a02,a10,a11,a12,a20,a21,a22;
        begin
            sob_gx = -$signed({1'b0,a00}) + $signed({1'b0,a02})
                     - ($signed({1'b0,a10}) <<< 1) + ($signed({1'b0,a12}) <<< 1)
                     - $signed({1'b0,a20}) + $signed({1'b0,a22});
        end
    endfunction
    function signed [12:0] sob_gy;
        input [7:0] a00,a01,a02,a10,a11,a12,a20,a21,a22;
        begin
            sob_gy =  $signed({1'b0,a20}) + ($signed({1'b0,a21}) <<< 1) + $signed({1'b0,a22})
                    - $signed({1'b0,a00}) - ($signed({1'b0,a01}) <<< 1) - $signed({1'b0,a02});
        end
    endfunction
    function [10:0] abs13;
        input signed [12:0] a;
        begin abs13 = a[12] ? (~a + 13'd1) : a; end
    endfunction

    wire signed [12:0] gx1 = sob_gx(t0,t1,t2,m0,m1,m2,b0,b1,b2);
    wire signed [12:0] gy1 = sob_gy(t0,t1,t2,m0,m1,m2,b0,b1,b2);
    wire signed [12:0] gx2 = sob_gx(t1,t2,t3,m1,m2,m3,b1,b2,b3);
    wire signed [12:0] gy2 = sob_gy(t1,t2,t3,m1,m2,m3,b1,b2,b3);
    wire [10:0] mag1 = abs13(gx1) + abs13(gy1);
    wire [10:0] mag2 = abs13(gx2) + abs13(gy2);
    wire use2 = (mag2 > mag1);
    wire [10:0] sel_mag = use2 ? mag2 : mag1;
    wire signed [12:0] sel_gx = use2 ? gx2 : gx1;
    wire signed [12:0] sel_gy = use2 ? gy2 : gy1;
    wire sel_subx = use2;
    wire [8:0] bx = x_q[10:2];
    wire [1:0] suby = y_q[1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k=0;k<BW;k=k+1) begin
                blk_mag[k]<=0; blk_gx[k]<=0; blk_gy[k]<=0;
                blk_subx[k]<=0; blk_suby[k]<=0; blk_v[k]<=0;
            end
        end
        else if (v_q && (y_q >= 12'd2) && (sel_mag >= EDGE_TH)) begin
            if (sel_mag > blk_mag[bx]) begin
                blk_mag[bx] <= sel_mag;
                blk_gx[bx]  <= sel_gx;
                blk_gy[bx]  <= sel_gy;
                blk_subx[bx]<= sel_subx;
                blk_suby[bx]<= suby;
                blk_v[bx]   <= 1'b1;
            end
        end
    end

    // ---------------- emission FSM ---------------------------------------
    localparam [1:0] E_IDLE=0, E_EMIT=1, E_END=2;
    reg [1:0]  est;
    reg [8:0]  ebx;
    reg [11:0] egroup_y;
    reg        elast;
    reg [7:0]  emag8;
    reg [16:0] erecip;
    reg signed [23:0] ux_raw, uy_raw;

    always @(*) begin
        emag8 = (blk_mag[ebx] >> 3);
        if (emag8 == 0) emag8 = 8'd1;
        erecip = recip8(emag8);
        ux_raw = $signed(blk_gx[ebx]) * $signed({1'b0,erecip});
        uy_raw = $signed(blk_gy[ebx]) * $signed({1'b0,erecip});
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            est<=E_IDLE; ebx<=0; egroup_y<=0; elast<=0;
            O_ev_valid<=0; O_ev_x<=0; O_ev_y<=0; O_ev_ux<=0; O_ev_uy<=0;
            O_ev_sector<=0; O_frame_start<=0; O_frame_end<=0;
        end
        else begin
            O_ev_valid <= 1'b0;
            O_frame_start <= I_tuser;
            O_frame_end <= 1'b0;
            case (est)
            E_IDLE: begin
                if (tlast_q && (y_q > 0) &&
                    (((y_q[1:0] == 2'd3)) || (y_q == 12'd718))) begin
                    egroup_y <= {2'd0, y_q[10:2]};
                    elast <= (y_q == 12'd718);
                    ebx <= 0; est <= E_EMIT;
                end
            end
            E_EMIT: begin
                if (blk_v[ebx]) begin
                    O_ev_x <= ({2'd0, ebx} << 2) + 12'd1 + {11'd0, blk_subx[ebx]};
                    O_ev_y <= (egroup_y << 2) + {10'd0, blk_suby[ebx]};
                    O_ev_ux <= clip10(ux_raw >>> 11);
                    O_ev_uy <= clip10(uy_raw >>> 11);
                    O_ev_sector <= sector16(blk_gx[ebx], blk_gy[ebx]);
                    O_ev_valid <= 1'b1;
                    blk_v[ebx] <= 1'b0;
                    blk_mag[ebx] <= 0;
                end
                if (ebx == BW-1) begin
                    if (elast) est <= E_END;
                    else begin ebx<=0; est<=E_IDLE; end
                end
                else ebx <= ebx + 1'b1;
            end
            E_END: begin
                O_frame_end <= 1'b1;
                est <= E_IDLE;
            end
            endcase
        end
    end
endmodule
'''

code = code.replace("__CASES__", cases)
out = os.path.join(os.path.dirname(__file__), "..", "..",
                   "user_source", "hdl_source", "isp", "isp_grad_event96.v")
out = os.path.abspath(out)
with open(out, "w", encoding="utf-8") as f:
    f.write(code)
print("wrote", out, len(code))
