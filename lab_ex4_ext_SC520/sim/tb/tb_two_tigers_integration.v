`timescale 1ns/1ps
// ==========================================================================
// Integration test: isp_ball_track (CSI) + isp_two_tigers (50 MHz), wired as
// design_top_wrapper wires them: the player consumes the tracker's
// out_fresh_tgl (one toggle per real core pass), NOT the green-box hold.
//
// Asynchronous clocks: 500 kHz system clock (2 us) and 333.33 kHz CSI clock
// (3 us); one processed frame is 5556 CSI cycles (16.668 ms, ~60 fps).
//
// Covers:
//  1. moving ball never satisfies the static stable gate, yet its real passes
//     start exactly one song;
//  2. brief loss while still a moving candidate: the box drops (no hold
//     before stable) but the song continues and never restarts;
//  3. static track -> 4-frame hold: out_valid/out_hold stay up with no fresh
//     pass, the song continues, and no new start is generated;
//  4. after the hold expires the box is gone, but the player keeps playing
//     until 1 s after the LAST REAL PASS (not 1 s after the box goes away),
//     then stops, mutes and re-arms;
//  5. re-detection starts again at note 0;
//  6. mid-song reset is silent.
// ==========================================================================
module tb_two_tigers_integration;
    localparam integer SYS_CLK_HZ       = 500_000;
    localparam integer FRAME_CSI_CYCLES = 5556;

    localparam [1:0] MODE_NONE   = 2'd0,
                     MODE_MOVE   = 2'd1,
                     MODE_STATIC = 2'd2;

    reg clk = 0, csi_clk = 0, rst_n = 0;
    always #1000 clk     = ~clk;      // 500 kHz, 2 us
    always #1500 csi_clk = ~csi_clk;  // 333.3 kHz, 3 us, asynchronous

    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    // ---- tracker stimulus -------------------------------------------------
    reg         cam_frame_start = 0;
    reg         det_evt         = 0;
    reg         det_pass        = 0;
    reg  [11:0] det_cx          = 12'd400;
    reg  [11:0] det_cy          = 12'd360;
    reg  [11:0] det_r           = 12'd40;
    reg  [11:0] det_minx        = 12'd360;
    reg  [11:0] det_maxx        = 12'd440;
    reg  [11:0] det_miny        = 12'd320;
    reg  [11:0] det_maxy        = 12'd400;
    reg  [1:0]  det_mode        = MODE_NONE;

    reg  [11:0] sweep_cx   = 12'd400;
    integer     sweep_dir  = 1;
    integer     frame_div  = 0;
    integer     frame_cnt  = 0;
    integer     last_static_cyc = 0;

    always @(negedge csi_clk) begin
        frame_div = frame_div + 1;
        if (frame_div >= FRAME_CSI_CYCLES) begin
            frame_div       = 0;
            cam_frame_start = 1'b1;
            det_evt         = 1'b1;
            frame_cnt       = frame_cnt + 1;
            if (det_mode == MODE_NONE) begin
                det_pass = 1'b0;
            end
            else begin
                det_pass = 1'b1;
                if (det_mode == MODE_MOVE) begin
                    if (sweep_dir > 0) begin
                        if (sweep_cx >= 12'd850) sweep_dir = -1;
                        else sweep_cx = sweep_cx + 12'd50;
                    end
                    else begin
                        if (sweep_cx <= 12'd250) sweep_dir = 1;
                        else sweep_cx = sweep_cx - 12'd50;
                    end
                end
                if (det_mode == MODE_STATIC) last_static_cyc = cyc;
                det_cx   = sweep_cx;
                det_minx = det_cx - det_r;
                det_maxx = det_cx + det_r;
                det_miny = det_cy - det_r;
                det_maxy = det_cy + det_r;
            end
        end
        else begin
            cam_frame_start = 1'b0;
            det_evt         = 1'b0;
            det_pass        = 1'b0;
        end
    end

    // ---- tracker (CSI domain) --------------------------------------------
    wire        trk_valid, trk_hold, trk_stable, trk_ok, trk_green;
    wire        trk_fresh_tgl, trk_id_chg;
    wire [11:0] trk_cx, trk_cy, trk_r;
    wire [11:0] trk_minx, trk_maxx, trk_miny, trk_maxy;

    isp_ball_track #(
        .IMG_W(1280), .IMG_H(720),
        .MS_DIV(333), .HOLD_FRAMES(4), .HOLD_TIMEOUT_MS(200),
        .STABLE_N(10), .STABLE_POS_PX(16), .RELAX_MAX(2)
    ) u_trk (
        .clk            (csi_clk),
        .rst_n          (rst_n),
        .cam_frame_start(cam_frame_start),
        .det_evt        (det_evt),
        .det_pass       (det_pass),
        .det_cx         (det_cx),
        .det_cy         (det_cy),
        .det_r          (det_r),
        .det_minx       (det_minx),
        .det_maxx       (det_maxx),
        .det_miny       (det_miny),
        .det_maxy       (det_maxy),
        .green_in       (1'b1),
        .out_valid      (trk_valid),
        .out_hold       (trk_hold),
        .out_stable     (trk_stable),
        .out_track_ok   (trk_ok),
        .out_fresh_tgl  (trk_fresh_tgl),
        .out_id_chg     (trk_id_chg),
        .out_cx         (trk_cx),
        .out_cy         (trk_cy),
        .out_r          (trk_r),
        .out_minx       (trk_minx),
        .out_maxx       (trk_maxx),
        .out_miny       (trk_miny),
        .out_maxy       (trk_maxy),
        .out_green      (trk_green)
    );

    // ---- player (50 MHz domain), trigger = real-pass toggle ---------------
    wire buzzer;
    isp_two_tigers #(
        .CLK_HZ(SYS_CLK_HZ), .TIMEOUT_MS(1000), .GAP_MS(20)
    ) u_player (
        .clk(clk), .rst_n(rst_n),
        .det_tgl(trk_fresh_tgl),
        .buzzer(buzzer)
    );

    // ---- checks -----------------------------------------------------------
    integer errors = 0;
    task check;
        input cond;
        input [8*110-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task wait_frames;
        input integer n;
        integer i;
        begin for (i=0;i<n;i=i+1) @(posedge cam_frame_start); end
    endtask

    task wait_playing;
        input v;
        input integer tmo;
        integer t;
        begin
            t = 0;
            while ((u_player.playing !== v) && (t < tmo)) begin @(posedge clk); t = t + 1; end
            if (u_player.playing !== v) begin
                $display("  [FAIL] timeout waiting playing=%0d", v);
                errors = errors + 1;
            end
        end
    endtask

    task wait_trk_valid;
        input v;
        input integer max_frames;
        integer f;
        begin
            f = 0;
            while ((trk_valid !== v) && (f < max_frames)) begin wait_frames(1); f = f + 1; end
            if (trk_valid !== v) begin
                $display("  [FAIL] timeout waiting trk_valid=%0d", v);
                errors = errors + 1;
            end
        end
    endtask

    // ---- start count / abort time ---------------------------------------
    integer start_cnt = 0;
    integer abort_cyc = 0;
    reg     playing_d = 0;
    always @(posedge clk) begin
        if (u_player.start) start_cnt = start_cnt + 1;
        playing_d <= u_player.playing;
        if (playing_d && !u_player.playing) abort_cyc = cyc;
    end

    integer note_before, fresh_before;
    initial begin
        det_mode = MODE_NONE;
        repeat (10) @(negedge csi_clk);
        repeat (10) @(negedge clk);

        check(buzzer == 0, "reset: buzzer low");
        check(u_player.playing == 0, "reset: sequencer idle");
        check(trk_valid == 0, "reset: no green box");

        rst_n = 1;
        repeat (50) @(negedge clk);
        check(u_player.playing == 0 && buzzer == 0,
              "released with no target: silent");

        // 1) moving ball (stable never asserts) starts exactly one song
        det_mode = MODE_MOVE;
        wait_trk_valid(1'b1, 5);
        check(trk_valid == 1, "moving ball: green box appears");
        check(trk_stable == 0, "moving ball: static stable gate NOT satisfied");
        wait_playing(1'b1, 2000);
        check(u_player.playing == 1, "moving ball starts the song");
        check(u_player.ent0 == 6'h0A, "first note is C5");
        check(u_player.note_i == 5'd0, "starts at note 0");
        check(start_cnt == 1, "exactly one start pulse");
        wait_frames(10);
        check(u_player.playing == 1 && start_cnt == 1,
              "moving detections keep playing, no restart");

        // 2) brief loss while still a moving candidate (<1 s)
        note_before = u_player.note_i;
        det_mode = MODE_NONE;
        wait_frames(2);
        check(u_player.playing == 1, "moving brief loss: song continues");
        det_mode = MODE_MOVE;
        wait_trk_valid(1'b1, 5);
        check(u_player.playing == 1, "moving brief loss: still playing");
        check(u_player.note_i >= note_before, "moving brief loss: no restart");
        check(start_cnt == 1, "moving brief loss: no new start");

        // 3) static track -> stable, then a 4-frame hold
        det_mode = MODE_STATIC;
        wait_frames(15);
        check(trk_stable == 1, "static frames: stable track confirmed");
        check(u_player.playing == 1, "before loss: playing");
        fresh_before = last_static_cyc;   // last REAL pass
        det_mode = MODE_NONE;
        wait_frames(2);
        check(trk_valid == 1, "4-frame hold: box still up");
        check(trk_hold == 1, "4-frame hold: out_hold=1, not a fresh pass");
        check(u_player.playing == 1, "hold: song continues");
        check(start_cnt == 1, "hold: no new start");

        // 4) the hold expires, but the player's 1 s timeout is counted from
        //    the LAST REAL PASS, not from the box going away.
        while ((cyc - fresh_before) < 50000) @(posedge clk); // 100 ms
        check(trk_valid == 0, "hold expired: green box gone");
        check(u_player.playing == 1, "100 ms after last real pass: still playing");
        wait_playing(1'b0, 600000);
        check(u_player.playing == 0, "1 s without a real pass: song stops");
        check(buzzer == 0, "timeout: output muted");
        check(u_player.armed == 1, "timeout: trigger re-armed");
        $display("  DBG abort latency=%0d cycles after the last real pass",
                 abort_cyc - fresh_before);
        check((abort_cyc - fresh_before) >= 495000 &&
              (abort_cyc - fresh_before) <= 505000,
              "timeout counts from the last REAL pass, not from hold expiry");

        // 5) re-detection starts from note 0
        det_mode = MODE_STATIC;
        wait_playing(1'b1, 40000);
        check(u_player.playing == 1, "re-detection starts again");
        check(u_player.note_i == 5'd0, "re-detection: starts at note 0");
        check(u_player.ent0 == 6'h0A, "re-detection: first note C5");
        check(start_cnt == 2, "re-detection: exactly one new start");

        // 6) reset in mid-song
        @(negedge clk); rst_n = 0;
        repeat (10) @(negedge clk);
        check(buzzer == 0, "reset mid-song: buzzer low");
        check(u_player.playing == 0, "reset mid-song: sequencer cleared");
        det_mode = MODE_NONE;
        repeat (10) @(negedge clk); rst_n = 1;
        repeat (50) @(negedge clk);
        check(buzzer == 0 && u_player.playing == 0,
              "after reset with no target: silent");

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
