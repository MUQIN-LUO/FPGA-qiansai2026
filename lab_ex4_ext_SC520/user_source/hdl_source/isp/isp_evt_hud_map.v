// ==========================================================================
// EVT circle detector -> HUD diagnostic mapping.
// --------------------------------------------------------------------------
// HUD byte layout (shared with the phase-1 CCL path):
//       {ovf, live, chord, rows, asp, size, pass, valid}
// The event detector has no chord/rows/asp/size criteria.  Those positions
// are reused so an actual event overflow and a skipped frame are separately
// visible instead of being merged into [7]:
//
//   [7] ovf  = the processed frame exceeded EVMAX (actual event overflow)
//   [6] live = the processed frame had >=1 captured event (same snapshot
//              as O_cnt; no longer sampled at frame_end one frame early)
//   [5] drop = >=1 frame was skipped since the previous processed result
//   [4:2]    = reserved 0 (phase-1 chord/rows/asp/size positions)
//   [1] pass = this processed frame produced a passing circle
//   [0] valid= detector O_valid (fresh pass or bounded hold)
//
// O_cnt is the 10-bit HUD field.  The HUD renders exactly three decimal
// digits and its comparator only handles 0..999, so the contract is:
//       O_cnt = min(I_cnt, 999)
// 999 therefore means ">=999 events": a full EVMAX=1536 frame reads 999,
// never a wrapped 512.  I_cnt is the exact processed-frame count and is
// latched with the result inside isp_circle_hough2 (S_DONE).
// ==========================================================================
module isp_evt_hud_map (
    input  wire [7:0]  I_flags,   // {4'd0, drop_seen, 1'b0, proc_ovf, pass}
    input  wire [11:0] I_cnt  ,   // exact processed-frame event count
    input  wire        I_valid,
    output wire [7:0]  O_flags,
    output wire [9:0]  O_cnt
);
    // Saturate (not truncate): 1000..1023 would render as the wrong decimal
    // value in the HUD, and 1536 would wrap to 512.
    wire [9:0] S_cnt_sat = (I_cnt > 12'd999) ? 10'd999 : I_cnt[9:0];

    assign O_flags = { I_flags[1],        // [7] actual overflow of processed frame
                       (I_cnt != 12'd0),  // [6] live, same snapshot as O_cnt
                       I_flags[3],        // [5] skipped frame(s)
                       3'b000,            // [4:2] reserved
                       I_flags[0],        // [1] pass
                       I_valid };         // [0] valid
    assign O_cnt   = S_cnt_sat;
endmodule
