`timescale 1ns/1ps
// ==========================================================================
// Directed test for isp_two_tigers (Two Tigers buzzer player), rev 2p rules.
//
// CLK_HZ is scaled to 500 kHz so the 16 s song is 8M cycles while the note
// divider formula keeps the real note frequencies.  det_tgl is driven from
// an asynchronous 333.3 kHz "CSI" clock: one toggle per real detection pass.
//
// Covers: reset; 1.2 s with no real detection (no start); first real
// detection starts at note 0; first C5 period and 20 ms gap; continuous
// detections never restart; brief <1 s miss continues; a detection at 999 ms
// prevents the timeout; a 67 ms hold-like gap does not reset the 1 s timer;
// the timeout stops at 1 s from the LAST REAL PASS (not from the hold) and
// re-arms; re-detection starts at note 0; full 16 s song with continuous
// detections; no replay after completion while detections continue; a >1 s
// gap then re-detection replays from note 0; mid-song reset is silent.
// ==========================================================================
module tb_two_tigers;
    localparam integer CLK_HZ            = 500_000;
    localparam integer FRESH_CSI_CYCLES  = 5556; // ~16.7 ms at 3 us/cycle

    reg clk = 0, csi_clk = 0, rst_n = 0;
    always #1000 clk     = ~clk;      // 500 kHz, 2 us
    always #1500 csi_clk = ~csi_clk;  // 333.3 kHz, 3 us, asynchronous

    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    reg     det_tgl    = 0;
    reg     inject     = 0;
    reg     keep_fresh = 0;
    integer fresh_div  = 0;
    integer last_fresh_cyc = 0;

    // One toggle per real detection.  `inject` requests a single event;
    // `keep_fresh` produces a continuous 60 fps-like stream.
    always @(negedge csi_clk) begin
        if (inject) begin
            det_tgl <= ~det_tgl;
            last_fresh_cyc = cyc;
            inject = 0;
        end
        else if (keep_fresh) begin
            fresh_div = fresh_div + 1;
            if (fresh_div >= FRESH_CSI_CYCLES) begin
                fresh_div = 0;
                det_tgl <= ~det_tgl;
                last_fresh_cyc = cyc;
            end
        end
        else fresh_div = 0;
    end

    wire buzzer;
    isp_two_tigers #(
        .CLK_HZ(CLK_HZ), .TIMEOUT_MS(1000), .GAP_MS(20)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .det_tgl(det_tgl),
        .buzzer(buzzer)
    );

    // ---- checks ----------------------------------------------------------
    integer errors = 0;
    task check;
        input cond;
        input [8*100-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task fresh;
        begin
            inject = 1;
            @(det_tgl);            // wait for the requested toggle
        end
    endtask

    task wait_playing;
        input v;
        input integer tmo;
        integer t;
        begin
            t = 0;
            while ((u_dut.playing !== v) && (t < tmo)) begin @(posedge clk); t = t + 1; end
            if (u_dut.playing !== v) begin
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

    // ---- start count / abort time ---------------------------------------
    integer start_cnt = 0;
    integer abort_cyc = 0;
    reg     playing_d = 0;
    always @(posedge clk) begin
        if (u_dut.start) start_cnt = start_cnt + 1;
        playing_d <= u_dut.playing;
        if (playing_d && !u_dut.playing) abort_cyc = cyc;
    end

    // ---- note sequence ---------------------------------------------------
    reg [5:0]  seen   [0:31];
    integer    seen_t [0:31];
    integer    seen_n = 0;
    reg [4:0]  last_i = 5'd31;
    always @(posedge clk) begin
        if (u_dut.playing) begin
            if (u_dut.note_i != last_i) begin
                if (seen_n < 32) begin
                    seen[seen_n] = u_dut.ent0;
                    seen_t[seen_n] = cyc;
                    seen_n = seen_n + 1;
                end
                last_i = u_dut.note_i;
            end
        end
        else last_i = 5'd31;
    end

    // first C5 period / first articulation gap
    integer p_t1 = 0, p_t2 = 0;
    reg     buzzer_d = 0;
    always @(posedge clk) begin
        buzzer_d <= buzzer;
        if (u_dut.playing && u_dut.note_on && u_dut.note_i == 5'd0 &&
            buzzer && !buzzer_d) begin
            if (p_t1 == 0) p_t1 = cyc;
            else if (p_t2 == 0) p_t2 = cyc;
        end
    end
    integer g_t1 = 0, g_t2 = 0;
    always @(posedge clk) begin
        if (g_t1 == 0 && !u_dut.note_on && u_dut.playing) g_t1 = cyc;
        else if (g_t1 != 0 && g_t2 == 0 && u_dut.note_on && u_dut.playing) g_t2 = cyc;
    end

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
    integer bad;
    initial begin
        repeat (10) @(negedge csi_clk);
        repeat (10) @(negedge clk);

        // 0) reset
        check(buzzer == 0, "reset: buzzer low");
        check(u_dut.playing == 0, "reset: not playing");
        check(u_dut.armed == 1, "reset: trigger armed");

        check(u_dut.since_ms == 0, "reset: watchdog cleared");
        rst_n = 1;
        repeat (50) @(negedge clk);
        check(u_dut.playing == 0 && buzzer == 0,
              "released with no detection: silent");

        // 1) long gap with no real detection -> no start
        wait_cycles(600000);   // 1.2 s
        check(u_dut.playing == 0 && buzzer == 0,
              "1.2 s with no real detection: no start");
        check(u_dut.armed == 1, "1.2 s gap: still armed");

        // 2) first real detection starts at note 0 immediately
        fresh();
        wait_playing(1'b1, 20);
        check(u_dut.playing == 1, "first real detection starts the song");
        check(u_dut.ent0 == 6'h0A, "first note is C5 1 beat");
        check(u_dut.note_i == 5'd0, "starts at note 0");
        check(u_dut.ndiv == (CLK_HZ/(2*523)), "C5 divider = CLK/(2*523)");
        check(start_cnt == 1, "exactly one start pulse");

        wait_cycles(2000);
        $display("  DBG p_t1=%0d p_t2=%0d period=%0d", p_t1, p_t2, p_t2-p_t1);
        check((p_t2 - p_t1) >= 954 && (p_t2 - p_t1) <= 958,
              "first note period ~956 cycles (523 Hz @500 kHz)");
        wait_cycles(300000);
        $display("  DBG g_t1=%0d g_t2=%0d gap=%0d", g_t1, g_t2, g_t2-g_t1);
        check((g_t2 - g_t1) >= 9900 && (g_t2 - g_t1) <= 10100,
              "first note gap ~20 ms (10,000 cycles)");

        // 3) continuous detections: song advances, never restarts
        keep_fresh = 1;
        wait_cycles(150000);   // 300 ms
        check(u_dut.playing == 1, "continuous detections: still playing");
        check(start_cnt == 1, "continuous detections: no restart");
        check(u_dut.note_i >= 5'd1, "continuous detections: notes advance");
        keep_fresh = 0;

        // 4) brief miss <1 s then a new detection: no restart
        wait_cycles(50000);    // 100 ms miss
        check(u_dut.playing == 1, "brief miss: still playing");
        check(u_dut.armed == 0, "brief miss: not re-armed");
        fresh();
        check(u_dut.playing == 1 && start_cnt == 1,
              "brief miss: re-detection does not restart");

        // 5) detection at 999 ms after the last real pass: no timeout
        wait_cycles(499500);   // 999 ms
        check(u_dut.playing == 1, "999 ms after last pass: still playing");
        check(u_dut.since_ms < 1000, "999 ms: watchdog not expired");
        fresh();
        check(u_dut.playing == 1 && start_cnt == 1,
              "999 ms return: no abort, no restart");
        check(u_dut.since_ms < 1000, "999 ms return: watchdog cleared");

        // 6) hold-like 67 ms gap must NOT reset the timer; timeout still
        //    fires 1 s after the LAST REAL PASS, not 1 s after the gap.
        wait_cycles(33500);    // 67 ms (4 camera frames at 60 fps)
        check(u_dut.playing == 1, "67 ms hold-like gap: still playing");
        wait_playing(1'b0, 600000);
        check(u_dut.playing == 0, "1 s without a real pass: song stops");
        check(buzzer == 0, "timeout: output muted");
        check(u_dut.armed == 1, "timeout: trigger re-armed");
        $display("  DBG abort latency=%0d cycles after last pass",
                 abort_cyc - last_fresh_cyc);
        check((abort_cyc - last_fresh_cyc) >= 497000 &&
              (abort_cyc - last_fresh_cyc) <= 502000,
              "timeout is 1 s from the last REAL pass, not from the hold");

        // 7) re-detection starts from the first note
        seen_n = 0; last_i = 5'd31;
        fresh();
        wait_playing(1'b1, 20);
        check(u_dut.playing == 1, "re-detection after timeout starts again");
        check(u_dut.note_i == 5'd0, "re-detection: starts at note 0");
        check(u_dut.ent0 == 6'h0A, "re-detection: first note C5");
        check(start_cnt == 2, "re-detection: exactly one new start");

        // 8) full song with continuous detections
        keep_fresh = 1;
        wait_playing(1'b0, 9000000);
        check(u_dut.playing == 0, "song completes");
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
        check(start_cnt == 2, "full song: no restart");

        // 9) target continuously detected after completion -> no replay
        wait_cycles(100000);   // 200 ms, keep_fresh still 1
        check(u_dut.playing == 0, "target still detected after song: no replay");
        check(start_cnt == 2, "target still detected: no new start");

        // 10) >1 s gap then re-detection -> replay from note 0
        keep_fresh = 0;
        wait_cycles(600000);   // 1.2 s without a real pass
        check(u_dut.armed == 1, "1 s gap after completion: re-armed");
        check(u_dut.playing == 0, "1 s gap: still silent");
        fresh();
        wait_playing(1'b1, 20);
        check(u_dut.playing == 1, "re-detection after gap replays");
        check(u_dut.note_i == 5'd0, "replay starts at note 0");
        check(u_dut.ent0 == 6'h0A, "replay first note C5");
        check(start_cnt == 3, "replay is exactly one new start");

        // 11) reset in mid-song
        @(negedge clk); rst_n = 0;
        repeat (10) @(negedge clk);
        check(buzzer == 0, "reset mid-song: buzzer low");
        check(u_dut.playing == 0, "reset mid-song: sequencer cleared");
        keep_fresh = 0;
        repeat (10) @(negedge clk); rst_n = 1;
        repeat (50) @(negedge clk);
        check(buzzer == 0 && u_dut.playing == 0,
              "after reset with no detection: silent");

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
