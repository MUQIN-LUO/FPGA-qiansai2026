`timescale 1ns/1ps
// Unit test for isp_ball_track (scaled ms timer: MS_DIV=10, HOLD_MS=20).
// Covers: 10-frame stability, bounded hold, hold not counting as a detection,
// background ignored during hold, global recovery, radius/position gates,
// limited relaxation and green latch.
module tb_ball_track;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        cam_frame_start = 0;
    reg        det_evt = 0, det_pass = 0;
    reg [11:0] det_cx = 0, det_cy = 0, det_r = 0;
    reg [11:0] det_minx = 0, det_maxx = 0, det_miny = 0, det_maxy = 0;
    reg        green_in = 0;

    wire out_valid, out_hold, out_stable, out_track_ok, out_green, out_id_chg;
    wire [11:0] out_cx, out_cy, out_r, out_minx, out_maxx, out_miny, out_maxy;

    isp_ball_track #(
        .IMG_W(1280), .IMG_H(720),
        .MS_DIV(10), .HOLD_FRAMES(4), .HOLD_TIMEOUT_MS(200),
        .STABLE_N(10), .RELAX_MAX(2)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .cam_frame_start(cam_frame_start),
        .det_evt(det_evt), .det_pass(det_pass),
        .det_cx(det_cx), .det_cy(det_cy), .det_r(det_r),
        .det_minx(det_minx), .det_maxx(det_maxx),
        .det_miny(det_miny), .det_maxy(det_maxy),
        .green_in(green_in),
        .out_valid(out_valid), .out_hold(out_hold),
        .out_stable(out_stable), .out_track_ok(out_track_ok),
        .out_cx(out_cx), .out_cy(out_cy), .out_r(out_r),
        .out_minx(out_minx), .out_maxx(out_maxx),
        .out_miny(out_miny), .out_maxy(out_maxy),
        .out_green(out_green)
    );

    integer errors = 0;
    task check;
        input cond;
        input [8*72-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task det;
        input [11:0] cx, cy, r;
        input        pass;
        input        grn;
        begin
            @(negedge clk);
            det_cx=cx; det_cy=cy; det_r=r;
            det_minx=(cx>r)?(cx-r):0; det_maxx=cx+r;
            det_miny=(cy>r)?(cy-r):0; det_maxy=cy+r;
            det_pass=pass; green_in=grn; det_evt=1'b1;
            @(negedge clk); det_evt=1'b0; det_pass=1'b0;
        end
    endtask

    task wait_cycles;
        input integer n;
        integer i;
        begin for (i=0;i<n;i=i+1) @(posedge clk); end
    endtask

    task frame_tick;
        input integer n;
        integer i;
        begin
            for (i=0;i<n;i=i+1) begin
                @(negedge clk); cam_frame_start=1'b1;
                @(negedge clk); cam_frame_start=1'b0;
            end
            repeat (2) @(posedge clk);
        end
    endtask

    integer i;
    initial begin
        repeat (5) @(negedge clk); rst_n = 1; repeat (5) @(negedge clk);
        check(out_valid==0 && out_stable==0 && out_hold==0, "reset: idle");

        // ---- 10 consecutive detections -> stable -----------------------
        for (i=0;i<10;i=i+1) det(600,360,60,1'b1,1'b1);
        check(out_valid==1, "candidate/stable display valid");
        check(out_stable==1, "10 consecutive -> stable");
        check(out_green==1, "green latch");
        check(out_cx==600 && out_cy==360 && out_r==60, "track values");

        // ---- miss -> hold, no detection count --------------------------
        // drive pass=0 with DIFFERENT values (simulates the core's own stale
        // hold): the tracker must not treat it as a new detection or update
        // the box / refresh the hold timer.
        det(100,100,90,1'b0,1'b0);
        check(out_valid==1 && out_hold==1 && out_stable==1, "miss -> bounded hold");
        check(out_cx==600 && out_cy==360 && out_r==60,
              "stale core hold values ignored");
        check(out_green==1, "green retained during hold");
        frame_tick(2);
        check(out_valid==1 && out_hold==1, "hold alive before 4 camera frames");
        frame_tick(2);
        check(out_valid==0 && out_hold==0 && out_stable==0,
              "hold expires after 4 real camera frames");
        check(out_green==0, "green clears with box");

        // ---- global recovery: far detection starts candidate -----------
        det(980,180,70,1'b1,1'b0);
        check(out_valid==1 && out_stable==0, "global recovery -> candidate");
        check(out_cx==980 && out_cy==180 && out_r==70, "candidate values");
        for (i=0;i<9;i=i+1) det(980,180,70,1'b1,1'b0);
        check(out_stable==1, "reconfirmed 10 -> stable");

        // ---- background ignored during hold ----------------------------
        det(980,180,70,1'b0,1'b0);          // miss -> hold
        check(out_hold==1, "second miss -> hold");
        det(100,100,70,1'b1,1'b0);          // far background during hold
        check(out_cx==980 && out_cy==180, "far detection ignored during hold");
        check(out_hold==1 && out_valid==1, "still holding old box");
        frame_tick(4);
        check(out_valid==0, "hold expired after 4 frames (ignored target)");

        // ---- new candidate at the far location -------------------------
        det(100,100,70,1'b1,1'b0);
        check(out_valid==1 && out_stable==0 && out_cx==100,
              "after expiry: global candidate at new location");
        for (i=0;i<9;i=i+1) det(100,100,70,1'b1,1'b0);
        check(out_stable==1, "candidate reconfirmed 10 -> stable");

        // ---- radius gate on a stable track -----------------------------
        det(100,100,84,1'b1,1'b0);          // +20% -> normal
        $display("  DBG r84 out_r=%0d hold=%b stable=%b st=%0d relax=%0d", out_r,out_hold,out_stable,u_dut.state,u_dut.relax_cnt);
        check(out_r==84 && out_stable==1, "radius +20% accepted");
        det(100,100,110,1'b1,1'b0);         // +31% from 84 -> relaxed
        $display("  DBG r110 out_r=%0d hold=%b stable=%b st=%0d relax=%0d", out_r,out_hold,out_stable,u_dut.state,u_dut.relax_cnt);
        check(out_r==110 && out_stable==1, "radius relaxed +31% accepted");
        det(100,100,165,1'b1,1'b0);         // +50% -> outside relaxed
        $display("  DBG r165 out_r=%0d hold=%b stable=%b st=%0d relax=%0d", out_r,out_hold,out_stable,u_dut.state,u_dut.relax_cnt);
        check(out_r==110 && out_hold==1, "radius >relaxed -> hold");
        frame_tick(4);
        check(out_valid==0, "radius-loss hold clears after 4 frames");

        // ---- limited relaxation (position) ----------------------------
        for (i=0;i<10;i=i+1) det(100,100,70,1'b1,1'b0);
        check(out_stable==1, "stable before position relaxation");
        det(250,100,70,1'b1,1'b0);
        check(out_cx==250, "relaxed position 1 accepted");
        det(430,100,70,1'b1,1'b0);
        check(out_cx==430, "relaxed position 2 accepted");
        det(700,100,70,1'b1,1'b0);
        check(out_cx==430 && out_hold==1, "relax limit -> hold");
        frame_tick(4);

        // ---- fast motion within normal gate ----------------------------
        det(800,300,60,1'b1,1'b0);
        check(out_valid==1 && out_stable==0, "fast reacquire candidate");
        for (i=0;i<9;i=i+1) det(800,300,60,1'b1,1'b0);
        check(out_stable==1, "fast reacquire stable");

        // ---- hard timeout with no camera frames (stall) -----------------
        for (i=0;i<10;i=i+1) det(800,300,60,1'b1,1'b0);
        check(out_stable==1, "stable before stall");
        wait_cycles(2500);              // > HOLD_TIMEOUT_MS=200 at MS_DIV=10
        check(out_valid==0, "stall timeout clears box (no frames)");

        // ---- continuous drift must NOT become static-stable -------------
        // Each +5 px step passes the motion gate (1.5r+16=106), but the
        // 10-frame anchor window is only 16 px, so the window keeps resetting.
        det(600,360,60,1'b1,1'b0);
        for (i=0;i<20;i=i+1) det(600+5*(i+1),360,60,1'b1,1'b0);
        check(out_stable==0, "continuous drift never static-stable");
        check(out_valid==1, "drift still tracked/displayed");
        // stop at the last drift position: 10 static frames -> stable
        for (i=0;i<10;i=i+1) det(700,360,60,1'b1,1'b0);
        check(out_stable==1, "stop after drift + 10 static -> stable");

        // ---- anchor window vs motion gate are separate ------------------
        @(negedge clk); rst_n=0; repeat (3) @(negedge clk); rst_n=1;
        repeat (3) @(negedge clk);
        for (i=0;i<5;i=i+1) det(600,360,60,1'b1,1'b0);
        check(out_stable==0, "5 static frames: not yet stable");
        det(620,360,60,1'b1,1'b0);   // 20 px: outside anchor 16, inside motion 106
        check(out_stable==0 && out_valid==1,
              "anchor jump resets count but keeps tracking");
        for (i=0;i<9;i=i+1) det(620,360,60,1'b1,1'b0);
        check(out_stable==1, "after anchor jump + 10 static -> stable");

        // ---- green -> red target switch, no empty frame ----------------
        @(negedge clk); rst_n=0; repeat (3) @(negedge clk); rst_n=1;
        repeat (3) @(negedge clk);
        for (i=0;i<10;i=i+1) det(600,360,60,1'b1,1'b1);
        check(out_stable==1 && out_green==1, "stable green target");
        // red target 150 px away: inside the relaxed motion gate (182) but
        // outside the 16 px colour/identity gate -> the old GREEN must drop
        det(750,360,60,1'b1,1'b1);   // green_in still stale 1 on purpose
        check(out_cx==750 && out_r==60 && out_valid==1,
              "relaxed target switch still tracked");
        check(out_green==0, "target switch clears old GREEN immediately");
        det(750,360,60,1'b1,1'b0);   // new red sample
        check(out_green==0, "red target keeps GREEN off");
        for (i=0;i<10;i=i+1) det(750,360,60,1'b1,1'b1); // new green target
        check(out_green==1, "new green target re-confirms GREEN");

        // far target during hold: old green box is held (correct), then the
        // new far target starts a candidate with GREEN cleared.
        det(1100,360,60,1'b0,1'b0);  // miss -> hold old green at 750
        check(out_cx==750 && out_hold==1 && out_green==1,
              "hold keeps old green box/colour");
        det(1100,360,60,1'b1,1'b1);  // far new target; stale green_in=1
        check(out_cx==750 && out_hold==1, "far target ignored during hold");
        frame_tick(4);
        check(out_valid==0, "hold expires after 4 frames");
        det(1100,360,60,1'b1,1'b1);  // now global candidate at the far target
        check(out_cx==1100 && out_valid==1 && out_green==0,
              "new far target starts with GREEN cleared");

        @(negedge clk); rst_n=0; repeat (3) @(negedge clk); rst_n=1;
        repeat (3) @(negedge clk);
        check(out_valid==0 && out_stable==0 && out_hold==0, "reset clears");

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else           $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
