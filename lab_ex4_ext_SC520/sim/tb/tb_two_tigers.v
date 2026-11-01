`timescale 1ns/1ps
// ==========================================================================
// Directed test for isp_two_tigers (Two Tigers buzzer player).
//
// CLK_HZ is scaled to 500 kHz so the 16 s song is 8M cycles, while the note
// divider formula gives the real note frequencies (C5=523 Hz etc.).
// The target_present level (green-box display valid) is driven from an
// asynchronous 333 kHz "CSI" clock to exercise the 2FF CDC and the rising-edge
// trigger.
//
// Covers: note sequence/durations/first-note period, gap, start, brief miss
// during playback (no restart), completion, no auto-replay, 1 s re-arm,
// no stacking, reset silence and CDC.
// ==========================================================================
module tb_two_tigers;
    localparam integer CLK_HZ = 500_000;

    reg clk = 0, csi_clk = 0, rst_n = 0;
    always #1000 clk = ~clk;        // 500 kHz
    always #1500 csi_clk = ~csi_clk; // 333.3 kHz, asynchronous

    reg  target_csi = 0;
    wire target_present = target_csi;
    wire buzzer;

    isp_two_tigers #(
        .CLK_HZ(CLK_HZ), .REARM_MS(1000), .GAP_MS(20)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .target_present(target_present),
        .buzzer(buzzer)
    );

    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    integer errors = 0;
    task check;
        input cond;
        input [8*80-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task set_target;
        input v;
        begin
            @(negedge csi_clk); target_csi = v;
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

    // ---- song note observation -----------------------------------------
    reg [5:0]  seen   [0:31];
    integer    seen_t [0:31];
    integer    seen_n = 0;
    reg [4:0]  last_i = 5'd31;
    reg        mon_on = 0;

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
        else begin
            last_i = 5'd31;
        end
    end

    // ---- first-note period --------------------------------------------
    integer p_t1 = 0, p_t2 = 0, p_val = 0;
    reg     buzzer_d = 0;
    always @(posedge clk) begin
        buzzer_d <= buzzer;
        if (u_dut.playing && u_dut.note_on && u_dut.note_i == 5'd0 &&
            buzzer && !buzzer_d) begin
            if (p_t1 == 0) p_t1 = cyc;
            else if (p_t2 == 0) p_t2 = cyc;
        end
    end

    // ---- first gap (note_on low) --------------------------------------
    integer g_t1 = 0, g_t2 = 0;
    always @(posedge clk) begin
        if (g_t1 == 0 && !u_dut.note_on && u_dut.playing) g_t1 = cyc;
        else if (g_t1 != 0 && g_t2 == 0 && u_dut.note_on && u_dut.playing) g_t2 = cyc;
    end

    // Expected compact ROM entries {note[2:0], duration_units[2:0]}
    // C5=1 D5=2 E5=3 F5=4 G5=5 A5=6 G4=0
    reg [5:0] exp [0:31];
    integer i;
    initial begin
        // C5 D5 E5 C5 | C5 D5 E5 C5
        exp[0]=6'h0A; exp[1]=6'h12; exp[2]=6'h1A; exp[3]=6'h0A;
        exp[4]=6'h0A; exp[5]=6'h12; exp[6]=6'h1A; exp[7]=6'h0A;
        // E5 F5 G5(2) | E5 F5 G5(2)
        exp[8]=6'h1A; exp[9]=6'h22; exp[10]=6'h2C;
        exp[11]=6'h1A; exp[12]=6'h22; exp[13]=6'h2C;
        // G5 A5 G5 F5 (half) E5 C5 | same
        exp[14]=6'h29; exp[15]=6'h31; exp[16]=6'h29; exp[17]=6'h21;
        exp[18]=6'h1A; exp[19]=6'h0A;
        exp[20]=6'h29; exp[21]=6'h31; exp[22]=6'h29; exp[23]=6'h21;
        exp[24]=6'h1A; exp[25]=6'h0A;
        // C5 G4 | C5(2) twice
        exp[26]=6'h0A; exp[27]=6'h02; exp[28]=6'h0C;
        exp[29]=6'h0A; exp[30]=6'h02; exp[31]=6'h0C;

        // reset
        repeat (10) @(negedge clk);
        rst_n = 1; repeat (5) @(negedge clk);
        check(buzzer == 0, "reset: buzzer low");
        check(u_dut.playing == 0, "reset: not playing");

        // start on a real target rising edge (async CSI domain)
        set_target(1'b1);
        wait_playing(1'b1, 2000);
        check(u_dut.playing == 1, "target edge starts the song");
        check(u_dut.ent0 == 6'h0A, "first note is C5 1 beat");
        check(u_dut.ndiv == (CLK_HZ/(2*523)), "C5 divider = CLK/(2*523)");

        // first-note period and first gap
        wait_cycles(2000);
        $display("  DBG p_t1=%0d p_t2=%0d period=%0d", p_t1, p_t2, p_t2-p_t1);
        check((p_t2 - p_t1) >= 954 && (p_t2 - p_t1) <= 958,
              "first note period ~956 cycles (523 Hz @500 kHz)");
        wait_cycles(300000);   // let note_on fall and gap finish
        $display("  DBG g_t1=%0d g_t2=%0d gap=%0d", g_t1, g_t2, g_t2-g_t1);
        check((g_t2 - g_t1) >= 9900 && (g_t2 - g_t1) <= 10100,
              "first note gap ~20 ms (10,000 cycles)");

        // brief loss during playback: no restart, no stacking
        begin : brief_loss
            integer note_before;
            note_before = u_dut.note_i;
            set_target(1'b0);
            wait_cycles(40000);    // 80 ms < REARM_MS
            set_target(1'b1);
            check(u_dut.playing == 1, "brief loss: song keeps playing");
            check(u_dut.note_i >= note_before, "brief loss: no restart");
        end

        // finish the 16 s song
        wait_playing(1'b0, 9000000);
        check(u_dut.playing == 0, "song completes");
        check(seen_n == 32, "all 32 notes observed");
        begin : seq_check
            integer bad;
            bad = 0;
            for (i=0;i<32;i=i+1)
                if (seen[i] !== exp[i]) begin
                    bad = bad + 1;
                    $display("  note[%0d]=%02h exp %02h", i, seen[i], exp[i]);
                end
            check(bad == 0, "note sequence matches the compact ROM");
        end
        check((seen_t[31]-seen_t[0]) >= 7490000 &&
              (seen_t[31]-seen_t[0]) <= 7510000,
              "first 31 notes = 15 s at 120 BPM (scaled)");
        // first note lasts 1 beat = 250,000 cycles; G5 2 beats = 500,000
        check((seen_t[1]-seen_t[0]) >= 249000 && (seen_t[1]-seen_t[0]) <= 251000,
              "1-beat note duration");
        check((seen_t[11]-seen_t[10]) >= 499000 && (seen_t[11]-seen_t[10]) <= 501000,
              "2-beat note duration");
        check((seen_t[15]-seen_t[14]) >= 124000 && (seen_t[15]-seen_t[14]) <= 126000,
              "half-beat note duration");

        // after completion, target still present -> no auto replay
        set_target(1'b1);
        wait_cycles(100000);   // 200 ms
        check(u_dut.playing == 0, "target still present: no auto replay");

        // absent > 1 s -> re-arm; a new edge starts a new song
        set_target(1'b0);
        wait_cycles(600000);   // 1.2 s
        set_target(1'b1);
        wait_playing(1'b1, 3000);
        check(u_dut.playing == 1 && u_dut.ent0 == 6'h0A,
              "after >1 s absence: re-armed and new song starts");
        check(u_dut.note_i == 5'd0, "new song starts at note 0");

        // reset in mid-song: output low, sequencer cleared
        @(negedge clk); rst_n = 0;
        repeat (10) @(negedge clk);
        check(buzzer == 0, "reset mid-song: buzzer low");
        check(u_dut.playing == 0, "reset mid-song: sequencer cleared");
        set_target(1'b0);
        repeat (10) @(negedge clk); rst_n = 1;
        repeat (50) @(negedge clk);
        check(buzzer == 0 && u_dut.playing == 0,
              "after reset with target low: silent");

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
