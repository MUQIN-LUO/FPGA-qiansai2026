// ==========================================================================
// "Two Tigers" (两只老虎) single-voice square-wave player for a passive buzzer.
// --------------------------------------------------------------------------
// The song is stored as a compact 32-entry note ROM (3-bit note index +
// 3-bit duration in 0.25 s half-beats), not as a WAV.  120 BPM, 64 half-beats
// = 16.0 s:
//   C5 D5 E5 C5 | C5 D5 E5 C5          (1 beat each)
//   E5 F5 G5(2) | E5 F5 G5(2)
//   G5 A5 G5 F5 (half) E5 C5 (1) | same
//   C5 G4 (1) C5(2) | same
//
// Trigger: the display-valid green box (isp_top O_det_valid) -- a fresh valid
// detection, or the tracker's own bounded hold of the SAME target.  It does
// NOT use the tracker's 10-frame static "stable" gate, so a moving ball
// triggers on its first valid box.  A held box is not a new detection: the
// level merely stays high, so it produces no new rising edge.
// A rising edge starts one song.  While playing, further edges are ignored
// (no stacking, no restart on brief misses / skipped frames / hold).  After
// the song ends, the same still-present target does not replay: the trigger
// re-arms only after the level has been absent for REARM_MS (default 1 s), so
// flicker cannot retrigger it.
//
// CDC: target_present is a slow level from the CSI clock domain; it is
// synchronized with a 2FF here.  The song/note dividers run on the stable
// 50 MHz system clock.  Reset forces the output low and clears the sequencer.
// ==========================================================================
module isp_two_tigers #(
    parameter integer CLK_HZ   = 50_000_000,
    parameter integer REARM_MS = 1000,
    parameter integer GAP_MS   = 20
)(
    input  wire clk,
    input  wire rst_n,
    input  wire target_present,  // green-box display-valid level (CSI domain)
    output reg  buzzer
);
    // Note indices in the compact ROM
    localparam [2:0] N_G4 = 3'd0, N_C5 = 3'd1, N_D5 = 3'd2, N_E5 = 3'd3,
                     N_F5 = 3'd4, N_G5 = 3'd5, N_A5 = 3'd6;

    // Equal-tempered note frequencies; divider = CLK/(2*f) gives a 50% square
    localparam integer D_G4 = CLK_HZ / (2 * 392);
    localparam integer D_C5 = CLK_HZ / (2 * 523);
    localparam integer D_D5 = CLK_HZ / (2 * 587);
    localparam integer D_E5 = CLK_HZ / (2 * 659);
    localparam integer D_F5 = CLK_HZ / (2 * 698);
    localparam integer D_G5 = CLK_HZ / (2 * 784);
    localparam integer D_A5 = CLK_HZ / (2 * 880);

    // 32-note song ROM: {note[2:0], duration_in_half_beats[2:0]}
    function [5:0] song_note;
        input [4:0] idx;
        begin
            case (idx)
            5'd0:  song_note = {N_C5, 3'd2}; // C5 1 beat
            5'd1:  song_note = {N_D5, 3'd2};
            5'd2:  song_note = {N_E5, 3'd2};
            5'd3:  song_note = {N_C5, 3'd2};
            5'd4:  song_note = {N_C5, 3'd2};
            5'd5:  song_note = {N_D5, 3'd2};
            5'd6:  song_note = {N_E5, 3'd2};
            5'd7:  song_note = {N_C5, 3'd2};
            5'd8:  song_note = {N_E5, 3'd2};
            5'd9:  song_note = {N_F5, 3'd2};
            5'd10: song_note = {N_G5, 3'd4}; // G5 2 beats
            5'd11: song_note = {N_E5, 3'd2};
            5'd12: song_note = {N_F5, 3'd2};
            5'd13: song_note = {N_G5, 3'd4};
            5'd14: song_note = {N_G5, 3'd1}; // half beat
            5'd15: song_note = {N_A5, 3'd1};
            5'd16: song_note = {N_G5, 3'd1};
            5'd17: song_note = {N_F5, 3'd1};
            5'd18: song_note = {N_E5, 3'd2};
            5'd19: song_note = {N_C5, 3'd2};
            5'd20: song_note = {N_G5, 3'd1};
            5'd21: song_note = {N_A5, 3'd1};
            5'd22: song_note = {N_G5, 3'd1};
            5'd23: song_note = {N_F5, 3'd1};
            5'd24: song_note = {N_E5, 3'd2};
            5'd25: song_note = {N_C5, 3'd2};
            5'd26: song_note = {N_C5, 3'd2};
            5'd27: song_note = {N_G4, 3'd2};
            5'd28: song_note = {N_C5, 3'd4};
            5'd29: song_note = {N_C5, 3'd2};
            5'd30: song_note = {N_G4, 3'd2};
            default: song_note = {N_C5, 3'd4}; // 5'd31: C5 2 beats
            endcase
        end
    endfunction

    function [15:0] note_div;
        input [2:0] n;
        begin
            case (n)
            N_G4: note_div = D_G4;
            N_C5: note_div = D_C5;
            N_D5: note_div = D_D5;
            N_E5: note_div = D_E5;
            N_F5: note_div = D_F5;
            N_G5: note_div = D_G5;
            N_A5: note_div = D_A5;
            default: note_div = D_C5;
            endcase
        end
    endfunction

    // ---- 1 ms tick on the stable system clock --------------------------
    localparam integer MS_DIV = (CLK_HZ >= 1000) ? (CLK_HZ / 1000) : 1;
    reg [31:0] ms_cnt;
    reg        ms_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt <= 32'd0; ms_tick <= 1'b0;
        end
        else if (ms_cnt >= (MS_DIV - 1)) begin
            ms_cnt <= 32'd0; ms_tick <= 1'b1;
        end
        else begin
            ms_cnt <= ms_cnt + 32'd1; ms_tick <= 1'b0;
        end
    end

    // ---- target CDC + rising edge + absence/re-arm timer --------------
    reg tgt_s1, tgt_s2, tgt_d;
    reg [31:0] absent_ms;
    reg armed;
    wire rise = tgt_s2 && !tgt_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tgt_s1 <= 1'b0; tgt_s2 <= 1'b0; tgt_d <= 1'b0;
            absent_ms <= 32'd0;
        end
        else begin
            tgt_s1 <= target_present;
            tgt_s2 <= tgt_s1;
            tgt_d  <= tgt_s2;

            if (ms_tick) begin
                if (!tgt_s2) begin
                    if (absent_ms < REARM_MS) absent_ms <= absent_ms + 32'd1;
                end
                else absent_ms <= 32'd0;
            end
        end
    end

    // ---- song sequencer + note divider (50 MHz domain) -----------------
    reg        playing;
    reg [4:0]  note_i;
    reg [15:0] note_ms, gap_ms;
    reg        note_on;
    reg [15:0] ndiv, half_cnt;
    reg        sq;

    wire [5:0] ent0 = song_note(note_i);
    wire [5:0] ent_next = song_note(note_i + 5'd1);
    wire [15:0] note_div_now = note_div(ent0[5:3]);
    wire [15:0] dur_ms_now = {10'd0, ent0[2:0]} * 16'd250;
    wire start = rise && armed && !playing;

    // Re-arm control is independent of the sequencer: a long absence during
    // playback must never freeze the notes.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) armed <= 1'b1;
        else if (start) armed <= 1'b0;
        else if (!tgt_s2 && (absent_ms >= (REARM_MS-1))) armed <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            playing <= 1'b0; note_i <= 5'd0;
            note_ms <= 16'd0; gap_ms <= 16'd0; note_on <= 1'b0;
            ndiv <= D_C5; half_cnt <= 16'd0; sq <= 1'b0;
            buzzer <= 1'b0;
        end
        else begin
            if (start) begin
                playing <= 1'b1;
                note_i  <= 5'd0;
                note_ms <= (16'd2 * 16'd250) - GAP_MS - 16'd1; // first: C5 1 beat
                gap_ms  <= GAP_MS - 16'd1;
                ndiv    <= D_C5;
                half_cnt<= 16'd0;
                sq      <= 1'b0;
                note_on <= 1'b1;
            end
            else if (playing && ms_tick) begin
                if (note_on) begin
                    if (note_ms != 16'd0) note_ms <= note_ms - 16'd1;
                    else begin
                        note_on <= 1'b0;
                        gap_ms  <= GAP_MS - 16'd1;
                    end
                end
                else begin
                    if (gap_ms != 16'd0) gap_ms <= gap_ms - 16'd1;
                    else if (note_i == 5'd31) begin
                        playing <= 1'b0;
                        note_on <= 1'b0;
                        sq      <= 1'b0;
                    end
                    else begin
                        note_i  <= note_i + 5'd1;
                        note_ms <= ({10'd0, ent_next[2:0]} * 16'd250) - GAP_MS - 16'd1;
                        gap_ms  <= GAP_MS - 16'd1;
                        ndiv    <= note_div(ent_next[5:3]);
                        half_cnt<= 16'd0;
                        sq      <= 1'b0;
                        note_on <= 1'b1;
                    end
                end
            end

            // note square-wave divider
            if (playing && note_on) begin
                if (half_cnt >= (ndiv - 16'd1)) begin
                    half_cnt <= 16'd0;
                    sq <= ~sq;
                end
                else half_cnt <= half_cnt + 16'd1;
            end
            else begin
                half_cnt <= 16'd0;
                sq <= 1'b0;
            end

            buzzer <= playing && note_on && sq;
        end
    end
endmodule
