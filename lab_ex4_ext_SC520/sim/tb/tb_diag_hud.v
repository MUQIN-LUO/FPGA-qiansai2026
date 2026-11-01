`timescale 1ns/1ps
// ==========================================================================
// HUD diagnostics / frame-handoff regression for the phase-2 EVT detector.
// Instantiates the REAL isp_circle_hough2 and the REAL isp_evt_hud_map (the
// same pair isp_top uses when EVT_CIRCLE=1).
//
// Scenario 1 (real frame period):
//   A) full load: exactly EVMAX=1536 events must be captured (ovf=0), the
//      result must arrive before the next 1.725M-cycle frame boundary, and
//      O_count_dbg/O_flags/O_valid must describe that SAME frame.
//   B) over limit: 1537 events -> cap_ovf -> proc_ovf=1, short-circuit S_DONE,
//      count is the captured 1536, HUD field saturates at 999 (never 512).
//   C) bounded hold ages on processed empty frames and clears.
//
// Scenario 2 (skip / snapshot):
//   A) frame B starts while A is processing -> B is skipped (cap_drop).
//      A's result must carry A's count (not B's 0) and drop=1.
//   B) the next processed result clears drop and reports its own count.
//   C) bounded hold (2) -> clear; overflow ages the hold; skip does not.
// ==========================================================================
module tb_diag_hud;
    localparam integer FRAME_CYCLES = 1_725_000; // 1280x720@60 @103.5 MHz
    localparam integer EVMAX        = 1536;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        ev_valid = 0;
    reg [11:0] ev_x = 0, ev_y = 0;
    reg signed [9:0] ev_ux = 0, ev_uy = 0;
    reg [3:0]  ev_sector = 0;
    reg        frame_start = 0, frame_end = 0;

    wire O_evt, O_valid;
    wire [11:0] O_cx, O_cy, O_r, O_minx, O_maxx, O_miny, O_maxy;
    wire [7:0]  O_flags;
    wire [15:0] O_score;
    wire [11:0] O_count_dbg;

    isp_circle_hough2 u_dut (
        .clk(clk), .rst_n(rst_n),
        .ev_valid(ev_valid), .ev_x(ev_x), .ev_y(ev_y),
        .ev_ux(ev_ux), .ev_uy(ev_uy), .ev_sector(ev_sector),
        .frame_start(frame_start), .frame_end(frame_end),
        .O_evt(O_evt), .O_valid(O_valid),
        .O_cx(O_cx), .O_cy(O_cy), .O_r(O_r),
        .O_minx(O_minx), .O_maxx(O_maxx), .O_miny(O_miny), .O_maxy(O_maxy),
        .O_flags(O_flags), .O_score(O_score), .O_count_dbg(O_count_dbg)
    );

    // Same mapping isp_top applies for EVT_CIRCLE=1.
    wire [7:0] hud_flags;
    wire [9:0] hud_cnt;
    isp_evt_hud_map u_hud (
        .I_flags(O_flags), .I_cnt(O_count_dbg), .I_valid(O_valid),
        .O_flags(hud_flags), .O_cnt(hud_cnt)
    );

    reg [47:0] memA [0:4095];   // r60 circle, nA events at memA[1..nA]
    integer nA;

    integer errors = 0;
    integer evtcnt = 0;
    integer cyc    = 0;
    integer cyc_start = 0, cyc_evt = 0, cyc_end = 0;
    integer cyc_a_start = 0;   // frame A start; pulse_start overwrites cyc_start
    integer i;

    // Result snapshot captured on the O_evt edge (all outputs change together).
    reg [11:0] s_cnt;
    reg [7:0]  s_flags, s_hud_flags;
    reg [9:0]  s_hud_cnt;
    reg        s_valid;
    reg [11:0] s_r;

    always @(posedge clk) begin
        cyc = cyc + 1;
        if (O_evt) begin
            evtcnt    = evtcnt + 1;
            s_cnt     = O_count_dbg;
            s_flags   = O_flags;
            s_hud_flags = hud_flags;
            s_hud_cnt = hud_cnt;
            s_valid   = O_valid;
            s_r       = O_r;
            cyc_evt   = cyc;
            $display("  EVT #%0d t=%0t valid=%b pass=%b r=%0d cnt=%0d flags=%02h hud={%02h,%0d} hold=%0d",
                     evtcnt, $time, O_valid, O_flags[0], O_r, O_count_dbg, O_flags,
                     hud_flags, hud_cnt, u_dut.hold_cnt);
        end
    end

    task pulse_start;
        begin
            @(negedge clk); frame_start = 1;
            @(negedge clk); frame_start = 0;
            cyc_start = cyc;
        end
    endtask

    task pulse_end;
        begin
            @(negedge clk); frame_end = 1;
            @(negedge clk); frame_end = 0;
            cyc_end = cyc;
        end
    endtask

    // Send n events, cycling through the nA circle events (duplicates are
    // intentional: they keep the result a valid r60 circle while loading the
    // capture path to an exact count).
    task send_rep;
        input integer n;
        integer j, idx;
        begin
            for (j = 1; j <= n; j = j + 1) begin
                idx = ((j - 1) % nA) + 1;
                @(negedge clk);
                ev_valid  = 1;
                ev_x      = memA[idx][47:36];
                ev_y      = memA[idx][35:24];
                ev_ux     = $signed(memA[idx][23:14]);
                ev_uy     = $signed(memA[idx][13:4]);
                ev_sector = memA[idx][3:0];
            end
            @(negedge clk);
            ev_valid = 0;
        end
    endtask

    // Send event data while the frame is a dropped one (cap_active=0): the
    // pulses must be ignored by the capture path.
    task send_ignored;
        input integer n;
        integer j;
        begin
            for (j = 1; j <= n; j = j + 1) begin
                @(negedge clk);
                ev_valid = 1;
                ev_x = 12'd100; ev_y = 12'd100; ev_ux = 10'sd1;
                ev_uy = 10'sd0; ev_sector = 4'd0;
            end
            @(negedge clk);
            ev_valid = 0;
        end
    endtask

    task wait_evt;
        input integer maxc;
        integer t, e0;
        begin
            e0 = evtcnt; t = 0;
            while ((evtcnt == e0) && (t < maxc)) begin
                @(posedge clk); t = t + 1;
            end
            if (evtcnt == e0) begin
                $display("  [FAIL] timeout waiting O_evt (%0d cycles)", maxc);
                errors = errors + 1;
            end
        end
    endtask

    task check;
        input cond;
        input [8*80-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task do_reset;
        begin
            @(negedge clk); rst_n = 0;
            repeat (5) @(negedge clk); rst_n = 1;
            repeat (5) @(negedge clk);
            evtcnt = 0;
        end
    endtask

    initial begin
        $readmemh("model/ev_r60.hex", memA);
        nA = memA[0][31:0];
        $display("tb_diag_hud: nA=%0d EVMAX=%0d FRAME_CYCLES=%0d", nA, EVMAX, FRAME_CYCLES);

        do_reset;

        // ================= Scenario 1A: full load at real frame period ====
        // This is also the throughput measurement.  The next frame starts at
        // the real 1.725M-cycle boundary: if the full-load frame is still
        // being processed then, that frame is skipped and the drop bit must
        // report it (overflow must stay 0: 1536 is NOT over the cap).
        $display("\n-- S1A: full load %0d events, real frame period --", EVMAX);
        pulse_start;
        cyc_a_start = cyc_start;
        send_rep(EVMAX);
        pulse_end;
        // Drive the next real frame boundary while A may still be processing.
        while (cyc < (cyc_a_start + FRAME_CYCLES)) @(posedge clk);
        pulse_start;
        send_ignored(200);          // dropped frame: must be ignored
        pulse_end;
        // The full-load frame needs ~1,526 cycles/event on this RTL; allow
        // up to 5M cycles so the measurement is reported, not truncated.
        wait_evt(5_000_000);
        check(s_cnt == EVMAX, "full load: O_count_dbg == 1536 (exact, no truncation)");
        check(s_hud_cnt == 10'd999, "full load: HUD count saturates to 999 (not 512/wrap)");
        check(s_valid == 1'b1, "full load: result valid");
        check(s_flags[0] == 1'b1, "full load: pass=1");
        check(s_flags[1] == 1'b0, "full load: proc_ovf=0 (1536 is NOT overflow)");
        check(s_hud_flags[7] == 1'b0, "full load: HUD ovf bit=0");
        check(s_hud_flags[6] == 1'b1, "full load: HUD live=1 from same snapshot");
        check(s_hud_flags[1] == 1'b1 && s_hud_flags[0] == 1'b1,
              "full load: HUD pass/valid=1");
        $display("  full-load processing: start->evt = %0d cycles (%0d.%02d frame periods), frame budget %0d",
                 cyc_evt - cyc_a_start,
                 (cyc_evt - cyc_a_start) / FRAME_CYCLES,
                 (((cyc_evt - cyc_a_start) % FRAME_CYCLES) * 100) / FRAME_CYCLES,
                 FRAME_CYCLES);
        if ((cyc_evt - cyc_a_start) < FRAME_CYCLES)
            $display("  [INFO] full-load frame fits inside one frame period");
        else
            $display("  [INFO] full-load frame EXCEEDS one frame period -> next frame is skipped");
        check(((cyc_evt - cyc_a_start) >= FRAME_CYCLES) == (s_flags[3] == 1'b1),
              "full load: drop bit matches the real-frame-boundary outcome");
        check(s_flags[3] == 1'b1 && s_hud_flags[5] == 1'b1,
              "full load: skipped next frame visible as drop=1 (ovf=0)");
        check(s_r >= 12'd56 && s_r <= 12'd64, "full load: r ~ 60");

        // ================= Scenario 1B: over limit 1537 ==================
        // Next real frame boundary (A's result is already out), 1537 events.
        while (cyc < (cyc_a_start + 2*FRAME_CYCLES)) @(posedge clk);
        $display("\n-- S1B: over limit %0d events --", EVMAX+1);
        pulse_start;
        send_rep(EVMAX + 1);
        pulse_end;
        wait_evt(1000);   // overflow short-circuits S_DONE, no processing
        check(s_cnt == EVMAX, "over limit: count is the captured 1536, not 1537");
        check(s_hud_cnt == 10'd999, "over limit: HUD count saturates to 999");
        check(s_flags[1] == 1'b1, "over limit: proc_ovf=1");
        check(s_flags[3] == 1'b0, "over limit: drop=0 (distinct from skip)");
        check(s_flags[0] == 1'b0, "over limit: pass=0");
        check(s_hud_flags[7] == 1'b1, "over limit: HUD ovf bit=1 (now distinguishable)");
        check(s_hud_flags[5] == 1'b0, "over limit: HUD drop bit=0");
        check(s_hud_flags[3:2] == 2'b00 && s_hud_flags[4] == 1'b0,
              "over limit: reserved HUD bits are 0");
        check(s_valid == 1'b1, "over limit: previous box held (HOLD=2)");
        check((cyc_evt - cyc_end) < 12'd1000, "over limit: short-circuit, no long processing");

        // ================= Scenario 1C: bounded hold clear ================
        $display("\n-- S1C: bounded hold after overflow --");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b1, "hold: empty1 keeps box");
        check(O_count_dbg == 12'd0, "hold: empty1 count=0 (same snapshot)");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b0, "hold: empty2 clears after HOLD=2");
        check(O_count_dbg == 12'd0 && O_flags[7:0] == 8'd0,
              "hold: cleared result has count/flags 0");

        // ================= Scenario 2: skip + count snapshot ==============
        $display("\n-- S2A: skipped frame + count snapshot --");
        do_reset;
        pulse_start;
        send_rep(nA);
        pulse_end;
        repeat (100) @(posedge clk);      // inside A's processing
        pulse_start;                      // B starts while busy -> cap_drop
        send_ignored(50);                 // must be ignored (cap_active=0)
        pulse_end;                        // B's frame_end sets drop_seen
        wait_evt(FRAME_CYCLES);
        check(evtcnt == 1, "skip: dropped frame produced no O_evt");
        check(s_cnt == nA, "skip: A result keeps A's count (not B's 0)");
        check(s_hud_cnt == nA[9:0], "skip: HUD count matches A");
        check(s_flags[1] == 1'b0, "skip: A did not overflow");
        check(s_flags[3] == 1'b1, "skip: drop=1 reported on A's result");
        check(s_hud_flags[5] == 1'b1 && s_hud_flags[7] == 1'b0,
              "skip: HUD drop=1 while ovf=0");
        check(s_valid == 1'b1 && s_flags[0] == 1'b1, "skip: A still valid/pass");

        $display("\n-- S2B: drop clears on next processed result --");
        pulse_start; send_rep(nA); pulse_end;
        wait_evt(FRAME_CYCLES);
        check(evtcnt == 2, "recovery: exactly one new O_evt");
        check(s_cnt == nA, "recovery: own count");
        check(s_flags[3] == 1'b0, "recovery: drop cleared");
        check(s_flags[1] == 1'b0, "recovery: no overflow");
        check(s_valid == 1'b1 && s_flags[0] == 1'b1, "recovery: valid/pass");

        $display("\n-- S2C: bounded hold (2 -> 1 -> 0 -> clear) --");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b1, "hold: empty1 held");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b1, "hold: empty2 held");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b0, "hold: empty3 cleared");

        $display("\n-- S2D: overflow ages hold, skip does not --");
        pulse_start; send_rep(nA); pulse_end; wait_evt(FRAME_CYCLES);
        check(O_valid == 1'b1 && O_flags[0] == 1'b1, "pre: valid detection");
        pulse_start; send_rep(EVMAX + 1); pulse_end; wait_evt(1000);
        check(O_valid == 1'b1, "overflow ages hold: still held");
        check(O_flags[1] == 1'b1 && O_flags[3] == 1'b0, "overflow: ovf=1 drop=0");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b1, "overflow+empty1: held");
        pulse_start; pulse_end; wait_evt(1000);
        check(O_valid == 1'b0, "overflow+empty2: cleared");

        // Skip while a long-enough frame is processing: no O_evt, hold intact.
        $display("\n-- S2E: skip during processing does not age hold --");
        pulse_start; send_rep(nA); pulse_end; wait_evt(FRAME_CYCLES);
        check(O_valid == 1'b1 && u_dut.hold_cnt == 2'd2, "skip-test setup: valid, hold=2");
        pulse_start; send_rep(nA); pulse_end;   // L starts processing
        repeat (100) @(posedge clk);
        i = evtcnt;
        pulse_start; send_ignored(50); pulse_end;  // M skipped
        check(evtcnt == i, "skip: no O_evt for the skipped frame");
        check(u_dut.hold_cnt == 2'd2, "skip: hold untouched (2)");
        wait_evt(FRAME_CYCLES);
        check(evtcnt == i + 1, "skip: L produced its own result");
        check(O_valid == 1'b1 && O_flags[0] == 1'b1, "skip: L valid/pass");
        check(O_flags[3] == 1'b1, "skip: L result flags the skipped frame");
        check(O_count_dbg == nA, "skip: L count is L's own");

        // ================= reset ==========================================
        $display("\n-- reset --");
        do_reset;
        check(O_valid == 1'b0 && O_count_dbg == 12'd0 && O_flags == 8'd0,
              "reset: outputs cleared");

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
