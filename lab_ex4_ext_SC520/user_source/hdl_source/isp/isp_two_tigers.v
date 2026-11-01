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
// Play rules (rev 2p):
//   - A REAL detection pass starts the song from note 0 immediately if the
//     trigger is armed and no song is playing.
//   - While playing, further real detections only clear the 1 s watchdog;
//     they never restart or re-stack the song.
//   - If no real detection has arrived for TIMEOUT_MS (default 1 s), the song
//     is stopped at once, the output is forced low, and the trigger re-arms.
//     The timeout counts from the LAST REAL PASS, not from the green-box
//     hold: skipped frames and the tracker's 4-frame hold do not reset it.
//   - After the song completes, a continuously detected target does not
//     replay it.  A >=1 s gap with no real detection re-arms; the next real
//     detection starts a new song at note 0.
//   - Reset: output low, sequencer cleared, trigger armed.
//
// CDC: det_tgl is a CSI-domain toggle, flipped once per real core pass (EVT:
// tracker det_pass; CCL: det_result O_pass).  It is 2FF-synchronized here and
// edge-detected into a one-cycle 50 MHz event.  At camera frame rates the
// toggle period is >= one processed frame (~16 ms), far longer than the 2FF
// latency, so no event can be lost.  Skipped processed frames and the tracker
// hold never toggle it.
// ==========================================================================
module isp_two_tigers #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer TIMEOUT_MS = 1000,
    parameter integer GAP_MS     = 20
)(
    input  wire clk,
    input  wire rst_n,
    input  wire det_tgl,       // CSI-domain toggle: one per real detection pass
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

    // ---- CSI toggle -> 2FF -> one-cycle real-detection event -----------
    reg tgl_s1, tgl_s2, tgl_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tgl_s1 <= 1'b0; tgl_s2 <= 1'b0; tgl_d <= 1'b0;
        end
        else begin
            tgl_s1 <= det_tgl;
            tgl_s2 <= tgl_s1;
            tgl_d  <= tgl_s2;
        end
    end
    // Suppress the reset-release edge: det_tgl may already be 1 when rst_n
    // deasserts, and the 2FF would otherwise report that initial level as a
    // detection.  Three 50 MHz cycles are enough for both sync stages to
    // settle and are negligible against the >=16 ms real-detection period.
    reg [2:0] fresh_blank;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fresh_blank <= 3'd0;
        else fresh_blank <= {fresh_blank[1:0], 1'b1};
    end
    wire fresh_evt = fresh_blank[2] && (tgl_s2 ^ tgl_d);

    // ---- 1 s watchdog counted from the last REAL detection ------------
    // It is cleared only by fresh_evt.  ms_tick with no fresh event
    // increments it; skipped frames / the green-box hold do not clear it.
    reg [31:0] since_ms;
    wire det_timeout = (since_ms >= TIMEOUT_MS);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) since_ms <= 32'd0;
        else if (fresh_evt) since_ms <= 32'd0;
        else if (ms_tick) begin
            if (since_ms < TIMEOUT_MS) since_ms <= since_ms + 32'd1;
        end
    end

    // ---- song sequencer state -----------------------------------------
    reg        playing;
    reg [4:0]  note_i;
    reg [15:0] note_ms, gap_ms;
    reg        note_on;
    reg [15:0] ndiv, half_cnt;
    reg        sq;
    reg        armed;

    wire [5:0]  ent0 = song_note(note_i);
    wire [5:0]  ent_next = song_note(note_i + 5'd1);
    wire        start = fresh_evt && armed && !playing;
    wire        abort_now = det_timeout && !fresh_evt && playing;

    // Armed only after 1 s with no real detection; cleared when a song starts.
    // Continuous detections after a completed song keep it at 0, so no replay.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) armed <= 1'b1;
        else if (start) armed <= 1'b0;
        else if (det_timeout && !fresh_evt) armed <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            playing <= 1'b0; note_i <= 5'd0;
            note_ms <= 16'd0; gap_ms <= 16'd0; note_on <= 1'b0;
            ndiv <= D_C5; half_cnt <= 16'd0; sq <= 1'b0;
            buzzer <= 1'b0;
        end
        else begin
            if (abort_now) begin
                // 1 s without a real detection: stop at once, mute, re-arm.
                playing <= 1'b0;
                note_on <= 1'b0;
                sq      <= 1'b0;
                note_ms <= 16'd0;
                gap_ms  <= 16'd0;
            end
            else if (start) begin
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

            // Hard mute on the timeout edge; otherwise normal note output.
            buzzer <= abort_now ? 1'b0 : (playing && note_on && sq);
        end
    end
endmodule
