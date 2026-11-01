// ==========================================================================
// Position/radius cross-frame ball tracker (CSI domain, after the event core).
// --------------------------------------------------------------------------
// The circle core is colour-agnostic and only reports one best candidate per
// PROCESSED frame (O_evt = S_DONE, O_flags[0] = fresh pass).  This block:
//   - associates fresh detections by position and radius
//       position gate  : |dcx| <= 1.5*r+16 and |dcy| <= 1.5*r+16
//       radius gate    : |dr| <= 0.20*r + 2 (approx. via shifts)
//   - requires STABLE_N consecutive associated fresh detections to declare a
//     stable track (default 10).  A candidate that misses is dropped at once;
//     only a stable track gets a bounded hold.
//   - after a stable track misses, keeps the LAST FRESH box/centre/radius for
//     HOLD_FRAMES real camera frames (4: ~67 ms at 60 fps, ~133 ms at 30 fps),
//     counted from cam_frame_start so the hold follows the actual camera rate.
//     A 200 ms CSI-clock timeout is OR-ed in so a stalled camera can never
//     hold the box forever.
//     The hold does not count as a detection and does not refresh itself.
//   - limited re-lock: a fresh detection outside the normal gate but inside a
//     2.5*r+32 / 40% radius-relaxed gate is accepted at most RELAX_MAX times
//     before the track is considered lost.
//   - after the hold expires the tracker goes IDLE; the next fresh detection
//     starts a new candidate ANYWHERE (global recovery) and must reconfirm
//     STABLE_N times.  A far background spot arriving during a hold is
//     ignored, so it cannot steal the lock.
//
// This is a tracking layer only: the underlying radius/validity gates are
// unchanged.  The hold output is display-only; det_pass stays the core's
// "this frame actually detected" bit.
// ==========================================================================
module isp_ball_track #(
    parameter integer IMG_W     = 1280,
    parameter integer IMG_H     = 720,
    parameter integer MS_DIV         = 103520, // CSI cycles per ms (103.52 MHz)
    parameter integer HOLD_FRAMES    = 4,      // real camera frames (3..5)
    parameter integer HOLD_TIMEOUT_MS= 200,    // hard stall timeout
    parameter integer STABLE_N       = 10,
    parameter integer STABLE_POS_PX  = 16,     // 10-frame anchor range
    parameter integer RELAX_MAX      = 2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cam_frame_start, // front-end frame start, every frame

    input  wire        det_evt,     // one-cycle pulse at core S_DONE
    input  wire        det_pass,    // fresh detection (core flags[0])
    input  wire [11:0] det_cx,
    input  wire [11:0] det_cy,
    input  wire [11:0] det_r,
    input  wire [11:0] det_minx,
    input  wire [11:0] det_maxx,
    input  wire [11:0] det_miny,
    input  wire [11:0] det_maxy,
    input  wire        green_in,    // debounced interior-colour flag

    output reg         out_valid,   // display box (fresh/candidate/held)
    output reg         out_hold,    // 1 = held, not a fresh detection
    output reg         out_stable,  // 1 = STABLE_N associated detections
    output reg         out_track_ok,// a track/candidate exists (for sampler)
    output reg         out_fresh_tgl,// toggles once per fresh core pass
    output reg         out_id_chg,  // 1-cycle: target identity changed
    output reg  [11:0] out_cx,
    output reg  [11:0] out_cy,
    output reg  [11:0] out_r,
    output reg  [11:0] out_minx,
    output reg  [11:0] out_maxx,
    output reg  [11:0] out_miny,
    output reg  [11:0] out_maxy,
    output reg         out_green
);
    localparam [1:0] S_IDLE = 2'd0, S_CAND = 2'd1,
                     S_TRK  = 2'd2, S_HOLD = 2'd3;

    reg [1:0]  state;
    reg [4:0]  cand_cnt;
    reg [1:0]  relax_cnt;
    reg [31:0] ms_cnt;
    reg [31:0] ms_now;
    reg [31:0] last_ms;
    reg [15:0] frame_cnt, last_frame;
    reg [11:0] tcx, tcy, tr;
    reg [11:0] acx, acy, ar;      // candidate anchor (first of the 10)
    reg [11:0] tminx, tmaxx, tminy, tmaxy;

    // ---- free-running 1 ms tick on the CSI clock ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt <= 32'd0;
            ms_now <= 32'd0;
        end
        else if (ms_cnt >= (MS_DIV-1)) begin
            ms_cnt <= 32'd0;
            ms_now <= ms_now + 32'd1;
        end
        else ms_cnt <= ms_cnt + 32'd1;
    end

    // ---- real camera-frame counter (skipped frames still count) -------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_cnt <= 16'd0;
        else if (cam_frame_start) frame_cnt <= frame_cnt + 16'd1;
    end

    // ---- association gates (absolute/square, no multipliers) ----------
    wire signed [12:0] sdx = $signed({1'b0,det_cx}) - $signed({1'b0,tcx});
    wire signed [12:0] sdy = $signed({1'b0,det_cy}) - $signed({1'b0,tcy});
    wire [12:0] adx = sdx[12] ? (~sdx + 13'd1) : sdx;
    wire [12:0] ady = sdy[12] ? (~sdy + 13'd1) : sdy;
    wire [12:0] gn  = {1'b0,tr} + {2'b0,tr[11:1]} + 13'd16;       // 1.5r+16
    wire [12:0] gr  = ({1'b0,tr} << 1) + {2'b0,tr[11:1]} + 13'd32; // 2.5r+32
    wire [11:0] rtol = (tr >> 3) + (tr >> 4) + (tr >> 6) + 12'd2;  // ~20%+2
    wire signed [12:0] sdr = $signed({1'b0,det_r}) - $signed({1'b0,tr});
    wire [12:0] adr = sdr[12] ? (~sdr + 13'd1) : sdr;
    wire pos_n = (adx <= gn) && (ady <= gn);
    wire pos_r = (adx <= gr) && (ady <= gr);
    wire [12:0] rtol2 = {1'b0,rtol} << 1;   // relaxed radius ~40%+4
    wire rad_n = (adr <= {1'b0,rtol});
    wire rad_r = (adr <= rtol2);
    wire match_n = pos_n && rad_n;
    wire match_r = pos_r && rad_r;

    // ---- 10-frame small-range stability gate (anchor, separate from the
    //      1.5r+16 motion association above).  A slow drift that stays within
    //      the motion gate but leaves the anchor window resets the window;
    //      it can never accumulate 10 "static" frames while drifting.
    wire signed [12:0] adx_a = $signed({1'b0,det_cx}) - $signed({1'b0,acx});
    wire signed [12:0] ady_a = $signed({1'b0,det_cy}) - $signed({1'b0,acy});
    wire signed [12:0] adr_a = $signed({1'b0,det_r})  - $signed({1'b0,ar});
    wire [12:0] aadx = adx_a[12] ? (~adx_a + 13'd1) : adx_a;
    wire [12:0] aady = ady_a[12] ? (~ady_a + 13'd1) : ady_a;
    wire [12:0] aadr = adr_a[12] ? (~adr_a + 13'd1) : adr_a;
    wire [11:0] artol = (ar >> 4) + (ar >> 5) + (ar >> 7) + 12'd2; // ~10%+2
    wire anchor_ok = (aadx <= STABLE_POS_PX) &&
                     (aady <= STABLE_POS_PX) &&
                     (aadr <= {1'b0,artol});

    // Tight colour-continuity gate relative to the current track.  A target
    // that jumps further than this (even if it passes the motion/relaxed
    // association gate) must not inherit the previous GREEN.
    wire signed [12:0] cdx_c = $signed({1'b0,det_cx}) - $signed({1'b0,tcx});
    wire signed [12:0] cdy_c = $signed({1'b0,det_cy}) - $signed({1'b0,tcy});
    wire signed [12:0] cdr_c = $signed({1'b0,det_r})  - $signed({1'b0,tr});
    wire [12:0] cadx = cdx_c[12] ? (~cdx_c + 13'd1) : cdx_c;
    wire [12:0] cady = cdy_c[12] ? (~cdy_c + 13'd1) : cdy_c;
    wire [12:0] cadr = cdr_c[12] ? (~cdr_c + 13'd1) : cdr_c;
    wire [11:0] ctol = (tr >> 4) + (tr >> 5) + (tr >> 7) + 12'd2;
    wire color_ok = (cadx <= STABLE_POS_PX) &&
                    (cady <= STABLE_POS_PX) &&
                    (cadr <= {1'b0,ctol});

    wire frame_expired = ((frame_cnt - last_frame) >= HOLD_FRAMES);
    wire ms_expired    = ((ms_now - last_ms) >= HOLD_TIMEOUT_MS);
    wire hold_expired  = frame_expired || ms_expired;

    // ---- one-cycle detection / hold processing ------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; cand_cnt <= 5'd0; relax_cnt <= 2'd0;
            last_ms <= 32'd0; last_frame <= 16'd0;
            tcx <= 12'd0; tcy <= 12'd0; tr <= 12'd0;
            acx <= 12'd0; acy <= 12'd0; ar <= 12'd0;
            tminx <= 12'd0; tmaxx <= 12'd0; tminy <= 12'd0; tmaxy <= 12'd0;
            out_valid <= 1'b0; out_hold <= 1'b0; out_stable <= 1'b0;
            out_track_ok <= 1'b0; out_fresh_tgl <= 1'b0;
            out_green <= 1'b0; out_id_chg <= 1'b0;
        end
        else if (det_evt) begin
            out_id_chg <= 1'b0;
            // One toggle per REAL core pass.  Skipped processed frames
            // (det_evt=0 or det_pass=0) and the bounded hold never toggle it;
            // a CSI-domain consumer can 2FF-sync this and edge-detect it.
            if (det_pass) out_fresh_tgl <= ~out_fresh_tgl;
            if (det_pass) begin
                case (state)
                // ---------------- new candidate / confirming ----------
                S_IDLE: begin
                    tcx<=det_cx; tcy<=det_cy; tr<=det_r;
                    acx<=det_cx; acy<=det_cy; ar<=det_r;
                    tminx<=det_minx; tmaxx<=det_maxx;
                    tminy<=det_miny; tmaxy<=det_maxy;
                    cand_cnt<=5'd1; relax_cnt<=2'd0;
                    last_ms<=ms_now; last_frame<=frame_cnt;
                    state<=S_CAND; out_valid<=1'b1; out_hold<=1'b0;
                    out_stable<=1'b0; out_track_ok<=1'b1;
                    out_green<=1'b0; out_id_chg<=1'b1;
                end
                S_CAND: begin
                    // Update the tracked box on every associated detection.
                    tcx<=det_cx; tcy<=det_cy; tr<=det_r;
                    tminx<=det_minx; tmaxx<=det_maxx;
                    tminy<=det_miny; tmaxy<=det_maxy;
                    last_ms<=ms_now; last_frame<=frame_cnt;
                    if (!match_n) begin
                        // motion gate failed: restart the candidate here
                        acx<=det_cx; acy<=det_cy; ar<=det_r;
                        cand_cnt<=5'd1; relax_cnt<=2'd0;
                        out_stable<=1'b0; out_green<=1'b0; out_id_chg<=1'b1;
                    end
                    else if (!anchor_ok) begin
                        // still moving within the motion gate but outside the
                        // small anchor window: re-anchor, do NOT count stable
                        acx<=det_cx; acy<=det_cy; ar<=det_r;
                        cand_cnt<=5'd1; relax_cnt<=2'd0;
                        out_stable<=1'b0; out_green<=1'b0; out_id_chg<=1'b1;
                    end
                    else begin
                        // static/small-jitter window: count toward stable
                        if ((cand_cnt + 5'd1) >= STABLE_N) begin
                            cand_cnt<=STABLE_N;
                            state<=S_TRK; out_stable<=1'b1;
                        end
                        else cand_cnt<=cand_cnt + 5'd1;
                        out_green<=green_in;
                    end
                    out_valid<=1'b1; out_hold<=1'b0;
                    out_track_ok<=1'b1;
                end
                // ---------------- stable track / hold -------------------
                S_TRK, S_HOLD: begin
                    if (match_n) begin
                        tcx<=det_cx; tcy<=det_cy; tr<=det_r;
                        tminx<=det_minx; tmaxx<=det_maxx;
                        tminy<=det_miny; tmaxy<=det_maxy;
                        last_ms<=ms_now; last_frame<=frame_cnt; relax_cnt<=2'd0;
                        state<=S_TRK; out_valid<=1'b1; out_hold<=1'b0;
                        out_stable<=1'b1; out_track_ok<=1'b1;
                        if (color_ok) out_green<=green_in;
                        else begin out_green<=1'b0; out_id_chg<=1'b1; end
                    end
                    else if (match_r && (relax_cnt < RELAX_MAX)) begin
                        // limited re-lock after a brief perturbation
                        tcx<=det_cx; tcy<=det_cy; tr<=det_r;
                        tminx<=det_minx; tmaxx<=det_maxx;
                        tminy<=det_miny; tmaxy<=det_maxy;
                        last_ms<=ms_now; last_frame<=frame_cnt; relax_cnt<=relax_cnt + 2'd1;
                        state<=S_TRK; out_valid<=1'b1; out_hold<=1'b0;
                        out_stable<=1'b1; out_track_ok<=1'b1;
                        if (color_ok) out_green<=green_in;
                        else begin out_green<=1'b0; out_id_chg<=1'b1; end
                    end
                    else begin
                        // out-of-gate: do not steal the lock; hold last box
                        state<=hold_expired ? S_IDLE : S_HOLD;
                        out_hold<=~hold_expired;
                        out_valid<=~hold_expired;
                        out_stable<=~hold_expired;
                        out_track_ok<=~hold_expired;
                        if (hold_expired) out_green<=1'b0;
                    end
                end
                default: state<=S_IDLE;
                endcase
            end
            else begin
                // processed frame without a fresh detection
                case (state)
                S_CAND: begin
                    // no hold before the target is stable
                    state<=S_IDLE; cand_cnt<=5'd0; relax_cnt<=2'd0;
                    out_valid<=1'b0; out_hold<=1'b0; out_stable<=1'b0;
                    out_track_ok<=1'b0; out_green<=1'b0;
                end
                S_TRK: begin
                    state<=hold_expired ? S_IDLE : S_HOLD;
                    out_hold<=~hold_expired;
                    out_valid<=~hold_expired;
                    out_stable<=~hold_expired;
                    out_track_ok<=~hold_expired;
                    if (hold_expired) out_green<=1'b0;
                end
                S_HOLD: begin
                    if (hold_expired) begin
                        state<=S_IDLE; cand_cnt<=5'd0; relax_cnt<=2'd0;
                        out_valid<=1'b0; out_hold<=1'b0; out_stable<=1'b0;
                        out_track_ok<=1'b0; out_green<=1'b0;
                    end
                end
                default: ;
                endcase
            end
        end
        else begin
            out_id_chg <= 1'b0;
            if ((state == S_CAND) || (state == S_TRK) || (state == S_HOLD)) begin
                // no result arriving at all (stalled/skipped processing):
                // never hold beyond HOLD_FRAMES / HOLD_TIMEOUT_MS from last fresh.
                if (hold_expired) begin
                    state<=S_IDLE; cand_cnt<=5'd0; relax_cnt<=2'd0;
                    out_valid<=1'b0; out_hold<=1'b0; out_stable<=1'b0;
                    out_track_ok<=1'b0; out_green<=1'b0;
                end
            end
        end
    end

    // Display box/centre: the last FRESH detection (updated only on match).
    always @(*) begin
        out_cx = tcx; out_cy = tcy; out_r = tr;
        out_minx = tminx; out_maxx = tmaxx;
        out_miny = tminy; out_maxy = tmaxy;
    end
endmodule
