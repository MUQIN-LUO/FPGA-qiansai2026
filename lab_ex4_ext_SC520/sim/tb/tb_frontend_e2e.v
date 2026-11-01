`timescale 1ns/1ps
// End-to-end research TB: synthetic 1280x720 grayscale beats -> isp_grad_event96
// -> isp_circle_hough2.  Scene is selectable; this is still a synthetic image,
// not a camera frame.  Short by design (one frame per run).
//
// Plusargs: CX CY R INS OUT EVALID
//           SCENE=0 disk, 1 rect, 2 ellipse, 3 two disks, 4 flat background
//           RA RB (ellipse), HW HH (rect), CX2 CY2 R2 (second disk)
//           GAPW  (occluding vertical slab on the right; 0 = none)
//           NOISE (uniform +/- amplitude added to every pixel; 0 = none)
module tb_frontend_e2e;
    parameter integer EV_TH_TB = 512;
    localparam integer W = 1280, H = 720;
    localparam integer LINE_CYCLES = 2300;
    localparam integer ACTIVE = W/4;
    localparam integer HBLANK = LINE_CYCLES - ACTIVE - 1;
    localparam integer VBLANK = 30;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [31:0] tdata = 0;
    reg [11:0] ix = 0, iy = 0;

    wire        ev_valid, fstart, fend;
    wire [11:0] ev_x, ev_y;
    wire signed [9:0] ev_ux, ev_uy;
    wire [3:0]  ev_sector;

    isp_grad_event96 #(.EV_TH(EV_TH_TB)) u_fe (
        .clk(clk), .rst_n(rst_n),
        .I_y4(tdata), .I_valid(tvalid), .I_tuser(tuser), .I_tlast(tlast),
        .I_x(ix), .I_y(iy),
        .O_ev_valid(ev_valid), .O_ev_x(ev_x), .O_ev_y(ev_y),
        .O_ev_ux(ev_ux), .O_ev_uy(ev_uy), .O_ev_sector(ev_sector),
        .O_frame_start(fstart), .O_frame_end(fend)
    );

    wire O_evt, O_valid;
    wire [11:0] O_cx, O_cy, O_r, O_minx, O_maxx, O_miny, O_maxy;
    wire [7:0] O_flags;
    wire [15:0] O_score;
    wire [11:0] O_count_dbg;

    isp_circle_hough2 u_h (
        .clk(clk), .rst_n(rst_n),
        .ev_valid(ev_valid), .ev_x(ev_x), .ev_y(ev_y),
        .ev_ux(ev_ux), .ev_uy(ev_uy), .ev_sector(ev_sector),
        .frame_start(fstart), .frame_end(fend),
        .O_evt(O_evt), .O_valid(O_valid),
        .O_cx(O_cx), .O_cy(O_cy), .O_r(O_r),
        .O_minx(O_minx), .O_maxx(O_maxx), .O_miny(O_miny), .O_maxy(O_maxy),
        .O_flags(O_flags), .O_score(O_score), .O_count_dbg(O_count_dbg)
    );

    integer CX, CY, RR, INS, OUT, EXP_V;
    integer SCENE, RA, RB, HW, HH, CX2, CY2, R2, GAPW, NOISE;
    integer C_R, C_G, C_B, BK_R, BK_G, BK_B, SHADOW;
    integer seed;
    integer errors = 0, done = 0, tmo, ecnt = 0;
    integer ev_total = 0, ev_off = 0, ev_baddir = 0;
    integer sdx, sdy, sr2, sr2lo, sr2hi, scross, slen, sline, ev_farline = 0;

    always @(posedge clk) if (ev_valid) begin
        ev_total = ev_total + 1;
        if (SCENE == 0 && GAPW == 0 && RR > 0) begin
            sdx = ev_x - CX; sdy = ev_y - CY;
            sr2 = sdx*sdx + sdy*sdy;
            sr2lo = (RR-5)*(RR-5); sr2hi = (RR+5)*(RR+5);
            if (sr2 < sr2lo || sr2 > sr2hi) ev_off = ev_off + 1;
            scross = sdx*ev_uy - sdy*ev_ux;
            slen = (sdx<0?-sdx:sdx) + (sdy<0?-sdy:sdy);
            if ((scross<0?-scross:scross) > slen*60) ev_baddir = ev_baddir + 1;
            sline = ((scross<0?-scross:scross)) >> 8;
            if (sline > 16) ev_farline = ev_farline + 1;
        end
    end
    always @(posedge clk) if (ev_valid && ecnt < 8) begin
        ecnt = ecnt + 1;
        $display("EV x=%0d y=%0d ux=%0d uy=%0d sec=%0d", ev_x, ev_y, ev_ux, ev_uy, ev_sector);
    end

    function integer in_ellipse;
        input integer px, py;
        integer dx, dy;
        begin
            dx = px - CX; dy = py - CY;
            in_ellipse = (((dx*dx)/(RA*RA)) + ((dy*dy)/(RB*RB))) <= 1;
        end
    endfunction

    function integer inside;
        input integer px, py;
        integer dx, dy, d1, d2;
        begin
            dx = px - CX; dy = py - CY;
            d1 = dx*dx + dy*dy;
            d2 = (px-CX2)*(px-CX2) + (py-CY2)*(py-CY2);
            inside = 0;
            case (SCENE)
              0: inside = (d1 <= RR*RR);
              1: inside = ((dx <= HW) && (dx >= -HW) && (dy <= HH) && (dy >= -HH));
              2: inside = in_ellipse(px, py);
              3: inside = (d1 <= RR*RR) || (d2 <= R2*R2);
              5: inside = (d1 <= RR*RR) || ((px-CX2 <= HW) && (px-CX2 >= -HW) && (py-CY2 <= HH) && (py-CY2 >= -HH));
              6: inside = (d1 <= RR*RR) || (((((px-CX2)*(px-CX2))/(RA*RA)) + (((py-CY2)*(py-CY2))/(RB*RB))) <= 1);
              7: inside = (d1 <= RR*RR);
              default: inside = 0;
            endcase
            if (GAPW > 0) begin
                if ((px > CX) && (dy < GAPW) && (dy > -GAPW)) inside = 0;
            end
        end
    endfunction

    function [7:0] y_of;
        input [7:0] r, g, b;
        begin y_of = (r >> 2) + (g >> 1) + (b >> 2); end
    endfunction

    function [7:0] pixel;
        input integer px, py;
        integer v, nz;
        reg [7:0] rr, gg, bb;
        begin
            if (SCENE == 7) begin
                if (inside(px, py)) begin rr = C_R; gg = C_G; bb = C_B; end
                else                begin rr = BK_R; gg = BK_G; bb = BK_B; end
                if (SHADOW != 0 && inside(px, py) && (px < CX)) begin
                    rr = rr >> 1; gg = gg >> 1; bb = bb >> 1;
                end
                v = y_of(rr, gg, bb);
            end
            else v = inside(px, py) ? INS : OUT;
            if (NOISE > 0) begin
                nz = $random(seed) % (2*NOISE + 1);
                v = v + (nz - NOISE);
            end
            if (v > 255) v = 255;
            if (v < 0)   v = 0;
            pixel = v[7:0];
        end
    endfunction

    always @(posedge clk) if (O_evt) done <= 1;
    integer pcyc = 0; reg pcnt = 0;
    always @(posedge clk) begin
        if (u_h.proc_start) begin pcyc = 0; pcnt = 1; end
        else if (pcnt) pcyc = pcyc + 1;
        if (O_evt) begin pcnt = 0; $display("PROC_CYCLES=%0d events=%0d", pcyc, u_h.proc_count); end
    end
    always @(posedge clk) if (u_h.st==6'd40)
        $display("CD2 cand=%0d cov=%0d ntr=%0d maxr=%0d peak_v=%0d peak_bin=%0d fill=%0d quad=%b nal=%0d proc=%0d ok=%b ls_i=%0d cx=%0d cy=%0d",
                 u_h.cand, u_h.cov, u_h.ntr, u_h.maxr, u_h.peak_v, u_h.peak_bin, u_h.fill_q8,
                 u_h.quad, u_h.n_align, u_h.proc_count, u_h.ok_band, u_h.ls_i, u_h.cx, u_h.cy);
    always @(posedge clk) if (u_h.st==6'd40 && 1'b0)
        $display("CD cand=%0d cov=%0d ntr=%0d maxr=%0d band=%h peak_v=%0d peak_bin=%0d",
                 u_h.cand, u_h.cov, u_h.ntr, u_h.maxr, u_h.band_r,
                 u_h.peak_v, u_h.peak_bin);
    always @(posedge clk) if (u_h.st==6'd31 && u_h.dv_done)
        $display("REF cand=%0d iter=%0d n=%0d tr=%0d gx=%0d gy=%0d r0=%0d cx=%0d cy=%0d",
                 u_h.cand, u_h.iter, u_h.n_ev, u_h.trace, u_h.gx_acc, u_h.gy_acc,
                 u_h.r0, u_h.cx, u_h.cy);
    always @(posedge clk) if (u_h.st==6'd24 && u_h.med_bin==8'd144)
        $display("MED n=%0d r0=%0d", u_h.n_ev, u_h.r0);
    integer evcap = 0;
    always @(posedge clk) if (ev_valid && u_h.cap_active) evcap = evcap + 1;
    always @(posedge clk) if (u_h.frame_start || u_h.frame_end)
        $display("CORE t=%0t fs=%b fe=%b cap=%b busy=%b wr=%0d proc=%0d evcap=%0d",
                 $time, u_h.frame_start, u_h.frame_end, u_h.cap_active, u_h.proc_busy,
                 u_h.wr_cnt, u_h.proc_count, evcap);

    task drive_line;
        input integer yy;
        input integer first;
        integer xx;
        begin
            for (xx = 0; xx < W; xx = xx + 4) begin
                @(negedge clk);
                tvalid = 1;
                tdata  = {pixel(xx,yy), pixel(xx+1,yy), pixel(xx+2,yy), pixel(xx+3,yy)};
                tuser  = (first && xx == 0);
                tlast  = (xx == 1276);
                ix = xx[11:0]; iy = yy[11:0];
            end
            @(negedge clk);
            tvalid = 0; tuser = 0; tlast = 0;
            repeat (HBLANK) @(negedge clk);
        end
    endtask

    initial begin
        if (!$value$plusargs("CX=%d", CX)) CX = 640;
        if (!$value$plusargs("CY=%d", CY)) CY = 360;
        if (!$value$plusargs("R=%d", RR)) RR = 60;
        if (!$value$plusargs("INS=%d", INS)) INS = 200;
        if (!$value$plusargs("OUT=%d", OUT)) OUT = 40;
        if (!$value$plusargs("EVALID=%d", EXP_V)) EXP_V = 1;
        if (!$value$plusargs("SCENE=%d", SCENE)) SCENE = 0;
        if (!$value$plusargs("RA=%d", RA)) RA = 80;
        if (!$value$plusargs("RB=%d", RB)) RB = 50;
        if (!$value$plusargs("HW=%d", HW)) HW = 200;
        if (!$value$plusargs("HH=%d", HH)) HH = 200;
        if (!$value$plusargs("CX2=%d", CX2)) CX2 = 900;
        if (!$value$plusargs("CY2=%d", CY2)) CY2 = 360;
        if (!$value$plusargs("R2=%d", R2)) R2 = 100;
        if (!$value$plusargs("GAPW=%d", GAPW)) GAPW = 0;
        if (!$value$plusargs("NOISE=%d", NOISE)) NOISE = 0;
        if (!$value$plusargs("C_R=%d", C_R)) C_R = 200;
        if (!$value$plusargs("C_G=%d", C_G)) C_G = 40;
        if (!$value$plusargs("C_B=%d", C_B)) C_B = 40;
        if (!$value$plusargs("BK_R=%d", BK_R)) BK_R = 140;
        if (!$value$plusargs("BK_G=%d", BK_G)) BK_G = 140;
        if (!$value$plusargs("BK_B=%d", BK_B)) BK_B = 140;
        if (!$value$plusargs("SHADOW=%d", SHADOW)) SHADOW = 0;
        $display("PARAMS SCENE=%0d CX=%0d CY=%0d R=%0d HW=%0d HH=%0d GAPW=%0d NOISE=%0d EVALID=%0d",
                 SCENE, CX, CY, RR, HW, HH, GAPW, NOISE, EXP_V);
        seed = 32'h12345;
        repeat (20) @(negedge clk);
        rst_n = 1;
        repeat (10) @(negedge clk);

        for (iy = 0; iy < H; iy = iy + 1) drive_line(iy, (iy == 0));
        repeat (VBLANK * LINE_CYCLES) @(negedge clk);

        for (tmo = 0; tmo < 6000000 && !done; tmo = tmo + 1) @(posedge clk);
        repeat (10) @(negedge clk);
        $display("E2E done=%0d valid=%0d cx=%0d cy=%0d r=%0d flags=%02h score=%0d ev_count=%0d",
                 done, O_valid, O_cx, O_cy, O_r, O_flags, O_score, O_count_dbg);
        $display(" DIAG events=%0d off_circle(>5px)=%0d bad_dir(>~14deg)=%0d farline(>16px)=%0d",
                 ev_total, ev_off, ev_baddir, ev_farline);
        for (tmo = 0; tmo < 4; tmo = tmo + 1)
            $display(" top%0d cnt=%0d cell=(%0d,%0d) c=(%0d,%0d) dx=%0d dy=%0d c_r=%0d cov=%0d sc=%0d ok=%0d",
                     tmo, u_h.top_cnt[tmo], u_h.top_x[tmo], u_h.top_y[tmo],
                     (u_h.top_x[tmo]<<2)+2, (u_h.top_y[tmo]<<2)+2,
                     ((u_h.top_x[tmo]<<2)+2)-CX, ((u_h.top_y[tmo]<<2)+2)-CY,
                     u_h.c_r[tmo], u_h.c_cov[tmo], u_h.c_score[tmo], u_h.c_ok[tmo]);
        $display(" dbg peak_v=%0d peak_bin=%0d r0=%0d cx=%0d cy=%0d",
                 u_h.peak_v, u_h.peak_bin, u_h.r0, u_h.cx, u_h.cy);
        if (!done) begin $display("  [FAIL] timeout"); errors=errors+1; end
        if (EXP_V == 0) begin
            if (O_valid !== 1'b0) begin $display("  [FAIL] expected no detection"); errors=errors+1; end
        end
        else if (O_valid !== 1'b1) begin $display("  [FAIL] no detection"); errors=errors+1; end
        else begin
            if (O_r < RR-4 || O_r > RR+4) begin $display("  [FAIL] r %0d vs %0d", O_r, RR); errors=errors+1; end
            if (O_cx < CX-5 || O_cx > CX+5 || O_cy < CY-5 || O_cy > CY+5) begin
                $display("  [FAIL] center (%0d,%0d) vs (%0d,%0d)", O_cx,O_cy,CX,CY); errors=errors+1;
            end
        end
        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
