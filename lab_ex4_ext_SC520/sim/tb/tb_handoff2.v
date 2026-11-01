`timescale 1ns/1ps
// Multi-frame handoff / hold test for isp_circle_hough2 (board-feedback fix).
//
// Scenario 1 (the bug): frame A is processed; frame B starts while proc_busy=1
// (cap_drop=1); processing of A finishes INSIDE frame B; then B's frame_end.
// Before the fix the core started a "processing" for the dropped frame and
// S_DONE wrote an invalid result (O_evt with O_valid=0), clearing the box.
// After the fix the dropped frame must be skipped: no O_evt, O_valid holds.
//
// Then: frame C is a new valid detection; processed empty frames keep the box
// for HOLD frames and then clear; an overflow frame ages the hold like an
// empty one; reset clears.  Skipped frames must NOT age the hold.
module tb_handoff2;
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

    reg [47:0] memA [0:4095];   // r60 at (640,360)
    reg [47:0] memC [0:4095];   // r100 at (640,360)
    reg [47:0] memX [0:4095];   // checker overfill
    integer nA, nC, nX;
    integer errors = 0;
    integer evtcnt = 0;
    always @(posedge clk) if (O_evt) begin
        evtcnt = evtcnt + 1;
        $display("  EVT t=%0t valid=%b r=%0d hold=%0d best=%b cnt=%0d capdrop=%b procdrop=%b",
                 $time, O_valid, O_r, u_dut.hold_cnt, u_dut.best_ok, u_dut.proc_count,
                 u_dut.cap_drop, u_dut.proc_drop);
    end

    task pulse_start; begin
        @(negedge clk); frame_start = 1;
        @(negedge clk); frame_start = 0;
    end endtask
    task pulse_end; begin
        @(negedge clk); frame_end = 1;
        @(negedge clk); frame_end = 0;
    end endtask

    task send_events(input integer n, input integer which); // 0=A 1=C 2=X
        integer i;
        begin
            for (i = 1; i <= n; i = i + 1) begin
                @(negedge clk);
                ev_valid = 1;
                if (which == 0) begin
                    ev_x = memA[i][47:36]; ev_y = memA[i][35:24];
                    ev_ux = $signed(memA[i][23:14]); ev_uy = $signed(memA[i][13:4]);
                    ev_sector = memA[i][3:0];
                end else if (which == 1) begin
                    ev_x = memC[i][47:36]; ev_y = memC[i][35:24];
                    ev_ux = $signed(memC[i][23:14]); ev_uy = $signed(memC[i][13:4]);
                    ev_sector = memC[i][3:0];
                end else begin
                    ev_x = memX[i][47:36]; ev_y = memX[i][35:24];
                    ev_ux = $signed(memX[i][23:14]); ev_uy = $signed(memX[i][13:4]);
                    ev_sector = memX[i][3:0];
                end
            end
            @(negedge clk); ev_valid = 0;
        end
    endtask

    task wait_proc_busy(input integer v);
        integer t;
        begin
            t = 0;
            while ((u_dut.proc_busy !== v) && (t < 3000000)) begin
                @(posedge clk); t = t + 1;
            end
            if (u_dut.proc_busy !== v) begin
                $display("  [FAIL] timeout waiting proc_busy=%0d", v); errors = errors + 1;
            end
        end
    endtask

    task wait_evt(input integer maxc);
        integer t, e0;
        begin
            e0 = evtcnt; t = 0;
            while ((evtcnt == e0) && (t < maxc)) begin @(posedge clk); t = t + 1; end
            if (evtcnt == e0) begin
                $display("  [FAIL] timeout waiting O_evt"); errors = errors + 1;
            end
        end
    endtask

    task check(input cond, input [8*64-1:0] msg);
        begin
            if (!cond) begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
            else       $display("  [ok]   %0s", msg);
        end
    endtask

    integer e0;
    initial begin
        $readmemh("model/ev_r60.hex",  memA); nA = memA[0][31:0];
        $readmemh("model/ev_r100.hex", memC); nC = memC[0][31:0];
        $readmemh("model/ev_checker.hex", memX); nX = memX[0][31:0];
        repeat (10) @(negedge clk); rst_n = 1; repeat (5) @(negedge clk);

        // ---------- frame A: valid circle, then B starts MID-processing ----
        pulse_start; send_events(nA, 0); pulse_end;
        wait_proc_busy(1);
        repeat (20000) @(posedge clk);     // inside A's processing (~470k cycles)
        pulse_start;                       // B starts while proc_busy=1 -> cap_drop=1
        wait_evt(3000000);                 // A's result appears during B
        check(O_valid === 1'b1, "A: valid detection");
        check(O_r >= 12'd56 && O_r <= 12'd64, "A: r ~ 60");
        $display("  A bbox=(%0d,%0d)-(%0d,%0d) r=%0d", O_minx,O_miny,O_maxx,O_maxy,O_r);
        wait_proc_busy(0);                 // A's processing finished inside B
        pulse_end;                         // dropped frame end
        e0 = evtcnt;
        repeat (5000) @(posedge clk);
        check(evtcnt == e0, "B: dropped frame produced no O_evt");
        check(O_valid === 1'b1, "B: held result still valid");
        check(O_r >= 12'd56 && O_r <= 12'd64, "B: held r ~ 60");
        $display("  B held bbox=(%0d,%0d)-(%0d,%0d) r=%0d flags=%02h", O_minx,O_miny,O_maxx,O_maxy,O_r,O_flags);

        // ---------- frame C: new valid detection ----------
        pulse_start; send_events(nC, 1); pulse_end;
        wait_evt(3000000);
        check(O_valid === 1'b1, "C: valid detection");
        check(O_r >= 12'd96 && O_r <= 12'd104, "C: r ~ 100");
        $display("  C bbox=(%0d,%0d)-(%0d,%0d) r=%0d", O_minx,O_miny,O_maxx,O_maxy,O_r);

        // ---------- processed empty frames age the hold ----------
        pulse_start; pulse_end; wait_evt(100000);
        check(O_valid === 1'b1, "empty1: held");
        pulse_start; pulse_end; wait_evt(100000);
        check(O_valid === 1'b1, "empty2: held");
        check(O_r >= 12'd96 && O_r <= 12'd104, "empty2: held bbox from C");
        pulse_start; pulse_end; wait_evt(100000);
        check(O_valid === 1'b0, "empty3: cleared after bounded hold");

        // ---------- overflow frame ages the hold like empty ----------
        pulse_start; send_events(nA, 0); pulse_end; wait_evt(3000000);
        check(O_valid === 1'b1, "valid again");
        pulse_start; send_events((nX > 1200) ? 1200 : nX, 2); pulse_end; wait_evt(200000);
        check(O_valid === 1'b1, "overflow1: held");
        check(u_dut.proc_ovf === 1'b1, "overflow: proc_ovf latched");
        pulse_start; pulse_end; wait_evt(100000);
        pulse_start; pulse_end; wait_evt(100000);
        check(O_valid === 1'b0, "overflow + 2 empty: cleared");

        // ---------- reset clears ----------
        @(negedge clk); rst_n = 0; repeat (5) @(negedge clk); rst_n = 1;
        repeat (5) @(negedge clk);
        check(O_valid === 1'b0, "reset: outputs cleared");

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
