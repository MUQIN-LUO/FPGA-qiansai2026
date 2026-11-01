`timescale 1ns/1ps
// ==========================================================================
// Integration test: isp_ball_track (CSI domain) + isp_two_tigers (50 MHz
// domain), wired exactly as design_top_wrapper wires them: the player trigger
// is the GREEN-BOX display level (tracker out_valid), NOT the 10-frame static
// "stable" gate.
//
// Clocks are deliberately asynchronous: 500 kHz system clock (2 us) and
// 333.33 kHz CSI clock (3 us).  CLK_HZ is scaled for the note divider so the
// note frequencies and the 16.0 s song are real; CSI MS_DIV=333 makes 1 ms
// real in the tracker, and one processed frame is 5556 CSI cycles (16.668 ms,
// ~60 fps).
//
// Cases:
//  1. moving ball (never satisfies the 16 px anchor -> stable=0) still raises
//     the green box and starts exactly one song (the reported bug);
//  2. brief loss during playback while still a moving candidate: the box
//     drops (no hold before stable) but the song keeps its position: no
//     restart, no stacking;
//  3. static detections confirm a stable track, then a 2-frame miss: the box
//     is HELD (out_hold=1), produces no new edge, no start pulse;
//  4. full 32-note song: sequence, note frequency, 1/2/0.5-beat durations,
//     20 ms gaps; target still present afterwards -> no auto replay;
//  5. target absent > 1 s -> re-arm; reacquire -> a new song starts at note 0;
//  6. asynchronous 2FF CDC latency is bounded (no missed level);
//  7. mid-song reset -> buzzer low, sequencer cleared; release with no target
//     stays silent.
// ==========================================================================
module tb_two_tigers_integration;
    localparam integer SYS_CLK_HZ        = 500_000;
    localparam integer FRAME_CSI_CYCLES  = 5556;

    localparam [1:0] MODE_NONE   = 2'd0,
                     MODE_MOVE   = 2'd1,
                     MODE_STATIC = 2'd2;

    reg clk = 0, csi_clk = 0, rst_n = 0;
    always #1000 clk     = ~clk;      // 500 kHz, 2 us
    always #1500 csi_clk = ~csi_clk;  // 333.3 kHz, 3 us, asynchronous

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

    // One processed frame: cam_frame_start and det_evt are one CSI-cycle
    // pulses; det_pass follows det_mode.  A moving target steps 50 px/frame,
    // which passes the motion gate (1.5*r+16 = 76) but never the 16 px anchor,
    // so it can never become "stable".
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
    wire        trk_valid, trk_hold, trk_stable, trk_ok, trk_green, trk_id_chg;
    wire [11:0] trk_cx, trk_cy, trk_r;
    wire [11:0] trk_minx, trk_maxx, trk_miny, trk_maxy;

    isp_ball_track #(
        .IMG_W(1280), .IMG_H(720),
        .MS_DIV(333),            // 333 CSI cycles = 1 ms at 3 us/cycle
        .HOLD_FRAMES(4),
        .HOLD_TIMEOUT_MS(200),
        .STABLE_N(10),
        .STABLE_POS_PX(16),
        .RELAX_MAX(2)
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

    // ---- player (50 MHz domain), trigger = green-box valid level ----------
    wire buzzer;
    isp_two_tigers #(
        .CLK_HZ(SYS_CLK_HZ), .REARM_MS(1000), .GAP_MS(20)
    ) u_player (
        .clk            (clk),
        .rst_n          (rst_n),
        .target_present (trk_valid),
        .buzzer         (buzzer)
    );

    // ---- bookkeeping ------------------------------------------------------
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    integer errors = 0;
    task check;
        input cond;
        input [8*110-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task fail_now;
        input [8*110-1:0] msg;
        begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
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

    task wait_cycles;
        input integer n;
        integer i;
        begin for (i=0;i<n;i=i+1) @(posedge clk); end
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

    // ---- start-pulse counter / CDC latency -------------------------------
    integer start_cnt = 0;
    integer first_valid_cyc = 0;
    integer first_start_cyc = 0;
    reg     valid_seen = 0;
    always @(posedge clk) begin
        if (trk_valid && !valid_seen) begin
            valid_seen = 1'b1;
            if (first_valid_cyc == 0) first_valid_cyc = cyc;
        end
        if (u_player.start) begin
            start_cnt = start_cnt + 1;
            if (first_start_cyc == 0) first_start_cyc = cyc;
        end
    end

    // The moving phase must never satisfy the static stable gate.
    reg moving_stable_seen = 0;
    always @(posedge csi_clk)
        if ((det_mode == MODE_MOVE) && trk_stable) moving_stable_seen = 1'b1;

    // ---- note observation ------------------------------------------------
    reg [5:0]  seen   [0:31];
    integer    seen_t [0:31];
    integer    seen_n = 0;
    reg [4:0]  last_i = 5'd31;
    always @(posedge clk) begin
        if (u_player.playing) begin
            if (u_player.note_i != last_i) begin
                if (seen_n < 32) begin
                    seen[seen_n] = u_player.ent0;
                    seen_t[seen_n] = cyc;
                    seen_n = seen_n + 1;
                end
                last_i = u_player.note_i;
            end
        end
        else last_i = 5'd31;
    end

    // first-note period (C5) measured on the actual buzzer output
    integer p_t1 = 0, p_t2 = 0;
    reg     buzzer_d = 0;
    always @(posedge clk) begin
        buzzer_d <= buzzer;
        if (u_player.playing && u_player.note_on && u_player.note_i == 5'd0 &&
            buzzer && !buzzer_d) begin
            if (p_t1 == 0) p_t1 = cyc;
            else if (p_t2 == 0) p_t2 = cyc;
        end
    end

    // first articulation gap
    integer g_t1 = 0, g_t2 = 0;
    always @(posedge clk) begin
        if (g_t1 == 0 && !u_player.note_on && u_player.playing) g_t1 = cyc;
        else if (g_t1 != 0 && g_t2 == 0 && u_player.note_on && u_player.playing) g_t2 = cyc;
    end

    // Expected compact ROM entries (same table as tb_two_tigers).
    reg [5:0] exp [0:31];
    integer i;
    initial begin
        exp[0]=6'h0A; exp[1]=6'h12; exp[2]=6'h1A; exp[3]=6'h0A;
        exp[4]=6'h0A; exp[5]=6'h12; exp[6]=6'h1A; exp[7]=6'h0A;
        exp[8]=6'h1A; exp[9]=6'h22; exp[10]=6'h2C;
        exp[11]=6'h1A; exp[12]=6'h22; exp[13]=6'h2C;
        exp[14]=6'h29; exp[15]=6'h31; exp[16]=6'h29; exp[17]=6'h21;
        exp[18]=6'h1A; exp[19]=6'h0A;
        exp[20]=6'h29; exp[21]=6'h31; exp[22]=6'h29; exp[23]=6'h21;
        exp[24]=6'h1A; exp[25]=6'h0A;
        exp[26]=6'h0A; exp[27]=6'h02; exp[28]=6'h0C;
        exp[29]=6'h0A; exp[30]=6'h02; exp[31]=6'h0C;
    end

    // ---- main sequence ---------------------------------------------------
    integer note_before, start_before;
    integer bad;
    initial begin
        det_mode = MODE_NONE;
        repeat (10) @(negedge csi_clk);
        repeat (10) @(negedge clk);

        // 0) reset with no target
        check(buzzer == 0, "reset: buzzer low");
        check(u_player.playing == 0, "reset: sequencer idle");
        check(trk_valid == 0, "reset: no green box");

        rst_n = 1;
        repeat (50) @(negedge clk);
        check(u_player.playing == 0 && buzzer == 0,
              "released with no target: silent");

        // 1) MOVING ball, never static -> green box must trigger one song
        det_mode = MODE_MOVE;
        wait_trk_valid(1'b1, 5);
        check(trk_valid == 1, "moving ball: green box appears");
        check(trk_stable == 0, "moving ball: stable gate is NOT satisfied");
        check(moving_stable_seen == 0, "moving ball: stable never asserted");
        wait_playing(1'b1, 2000);
        check(u_player.playing == 1, "moving ball triggers the song");
        check(u_player.ent0 == 6'h0A, "first note is C5 1 beat");
        check(u_player.ndiv == (SYS_CLK_HZ/(2*523)), "C5 divider = CLK/(2*523)");
        check(start_cnt == 1, "exactly one start pulse");
        $display("  DBG CDC latency: valid@%0d start@%0d = %0d sys cycles",
                 first_valid_cyc, first_start_cyc,
                 first_start_cyc - first_valid_cyc);
        check((first_start_cyc - first_valid_cyc) >= 1 &&
              (first_start_cyc - first_valid_cyc) <= 10,
              "CDC: trigger within 10 sys cycles (2FF)");

        // first-note frequency and articulation gap (real frequencies)
        wait_cycles(2000);
        $display("  DBG p_t1=%0d p_t2=%0d period=%0d", p_t1, p_t2, p_t2-p_t1);
        check((p_t2 - p_t1) >= 954 && (p_t2 - p_t1) <= 958,
              "C5 output period ~956 cycles (523 Hz @500 kHz)");
        wait_cycles(300000);
        $display("  DBG g_t1=%0d g_t2=%0d gap=%0d", g_t1, g_t2, g_t2-g_t1);
        check((g_t2 - g_t1) >= 9900 && (g_t2 - g_t1) <= 10100,
              "20 ms note gap (10,000 cycles)");

        // 2) brief loss during playback, still a moving candidate:
        //    the box drops at once (no hold before stable), but the song
        //    must keep its position and never restart.
        note_before = u_player.note_i;
        start_before = start_cnt;
        det_mode = MODE_NONE;
        wait_frames(2);
        check(trk_valid == 0, "moving brief loss: box drops (no hold yet)");
        check(u_player.playing == 1, "moving brief loss: song still playing");
        det_mode = MODE_MOVE;
        wait_trk_valid(1'b1, 5);
        check(u_player.playing == 1, "moving brief loss: still playing");
        check(u_player.note_i >= note_before, "moving brief loss: no restart");
        check(start_cnt == start_before, "moving brief loss: no new start");

        // 3) become static -> stable track; then a miss inside the hold must
        //    hold the box without creating a new detection/start.
        det_mode = MODE_STATIC;
        wait_frames(15);
        check(trk_stable == 1, "static frames: stable track confirmed");
        note_before  = u_player.note_i;
        start_before = start_cnt;
        det_mode = MODE_NONE;
        wait_frames(2);
        check(trk_valid == 1, "stable brief loss: box is HELD");
        check(trk_hold == 1, "stable brief loss: out_hold, not a fresh box");
        check(u_player.playing == 1, "stable brief loss: song continues");
        check(start_cnt == start_before, "held box: no new start pulse");
        det_mode = MODE_STATIC;
        wait_frames(2);
        check(u_player.playing == 1 && u_player.note_i >= note_before,
              "brief loss: no restart");

        // 4) finish the 16 s song
        wait_playing(1'b0, 9000000);
        check(u_player.playing == 0, "song completes");
        check(seen_n == 32, "all 32 notes observed");
        bad = 0;
        for (i=0;i<32;i=i+1)
            if (seen[i] !== exp[i]) begin
                bad = bad + 1;
                $display("  note[%0d]=%02h exp %02h", i, seen[i], exp[i]);
            end
        check(bad == 0, "note sequence matches the compact ROM");
        check((seen_t[31]-seen_t[0]) >= 7490000 &&
              (seen_t[31]-seen_t[0]) <= 7510000,
              "first 31 notes = 15 s at 120 BPM");
        check((seen_t[1]-seen_t[0]) >= 249000 && (seen_t[1]-seen_t[0]) <= 251000,
              "1-beat note duration (0.5 s)");
        check((seen_t[11]-seen_t[10]) >= 499000 &&
              (seen_t[11]-seen_t[10]) <= 501000,
              "2-beat note duration (1.0 s)");
        check((seen_t[15]-seen_t[14]) >= 124000 &&
              (seen_t[15]-seen_t[14]) <= 126000,
              "half-beat note duration (0.25 s)");

        // target still present -> no automatic replay
        start_before = start_cnt;
        wait_cycles(100000);   // 200 ms
        check(u_player.playing == 0, "target still present: no auto replay");
        check(start_cnt == start_before, "target still present: no new start");

        // 5) target absent > 1 s -> re-arm; reacquire -> new song at note 0
        det_mode = MODE_NONE;
        wait_trk_valid(1'b0, 20);
        check(trk_valid == 0, "long loss: green box gone");
        wait_cycles(600000);   // 1.2 s
        check(u_player.armed == 1, "1 s absence: trigger re-armed");
        start_before = start_cnt;
        det_mode = MODE_STATIC;
        wait_playing(1'b1, 40000);
        check(u_player.playing == 1, "reacquire: new song starts");
        check(u_player.note_i == 5'd0, "reacquire: starts at note 0");
        check(u_player.ent0 == 6'h0A, "reacquire: first note C5");
        check(start_cnt == start_before + 1, "reacquire: exactly one new start");

        // 6) reset mid-song
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
