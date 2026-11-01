`timescale 1ns/1ps
// Short event-level TB for the phase-2 core (research rev 2).
// Inputs are synthetic 4x4-block events, not the real O_dg front end.
module tb_circle_hough2;
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

    reg [47:0] mem [0:4095];
    integer n, i, t;
    integer errors = 0;
    reg done;
    reg [8*160-1:0] fname;
    reg [11:0] exp_cx, exp_cy, exp_r;
    integer exp_valid, exp_tol_c, exp_tol_r;

    always @(posedge clk) if (O_evt) done <= 1'b1;
    integer vcnt = 0;
    always @(posedge clk) if (u_dut.st==6'd5 && u_dut.ev_i==0 && vcnt < 80) begin // VOTEPROBE
        vcnt = vcnt + 1;
        $display("VOTE pol=%b k=%0d ayp=%0d vm=%0d we=%b wa=%0d wd=%0d rd=%0d",
                 u_dut.pol, u_dut.k, u_dut.ayp, u_dut.vaddr_m, u_dut.acc_we_w,
                 u_dut.acc_wa_w, u_dut.acc_wd_w, u_dut.acc_rd);
    end
    always @(posedge clk) if (u_dut.st==6'd7 && u_dut.sy_d==9'd90 && u_dut.sx_d==9'd100) // ACCPROBE
        $display("ACC circle-cell=%0d", u_dut.u_acc.m[90*320+100]);
    always @(posedge clk) if (u_dut.st==6'd8 && u_dut.sx_d==9'd100 && u_dut.sy_d==9'd90)
        $display("TOPEV acc_rd=%0d tops=%0d@(%0d,%0d) %0d@(%0d,%0d) %0d@(%0d,%0d) %0d@(%0d,%0d) cur=(%0d,%0d)",
                 u_dut.acc_rd,
                 u_dut.top_cnt[0],u_dut.top_x[0],u_dut.top_y[0],
                 u_dut.top_cnt[1],u_dut.top_x[1],u_dut.top_y[1],
                 u_dut.top_cnt[2],u_dut.top_x[2],u_dut.top_y[2],
                 u_dut.top_cnt[3],u_dut.top_x[3],u_dut.top_y[3],
                 u_dut.sx_d, u_dut.sy_d);
    integer pcyc = 0; reg pcnt = 0;
    always @(posedge clk) begin
        if (u_dut.proc_start) begin pcyc = 0; pcnt = 1; end
        else if (pcnt) pcyc = pcyc + 1;
        if (O_evt) begin pcnt = 0; $display("PROC_CYCLES=%0d events=%0d", pcyc, u_dut.proc_count); end
    end
    always @(posedge clk) if (u_dut.st==6'd40)
        $display("CD cand=%0d cov=%0d ntr=%0d maxr=%0d band=%h peak_v=%0d peak_bin=%0d cx=%0d cy=%0d r0=%0d",
                 u_dut.cand, u_dut.cov, u_dut.ntr, u_dut.maxr, u_dut.band_r,
                 u_dut.peak_v, u_dut.peak_bin, u_dut.cx, u_dut.cy, u_dut.r0);

    integer dbgcnt = 0;
    always @(posedge clk) begin
        if (dbgcnt < 40 && u_dut.st == 6'd31) begin
            dbgcnt = dbgcnt + 1;
            $display("RWAITM done=%b sr=%0d n=%0d trace=%0d gx=%0d gy=%0d cx=%0d cy=%0d r0=%0d",
                     u_dut.dv_done, u_dut.sr, u_dut.n_ev, u_dut.trace,
                     u_dut.gx_acc, u_dut.gy_acc, u_dut.cx, u_dut.cy, u_dut.r0);
        end
        if (dbgcnt < 40 && u_dut.st == 6'd29) begin
            $display("RMULX mulp=%0d step=%0d qx=%0d qy=%0d r0=%0d sxsign=%b",
                     u_dut.mul_p, u_dut.step_raw, u_dut.qx, u_dut.qy, u_dut.r0, u_dut.sx_sign);
        end
    end

    initial begin
        if (!$value$plusargs("EVFILE=%s", fname)) fname = "model/ev_r60.hex";
        if (!$value$plusargs("ECX=%d", exp_cx)) exp_cx = 640;
        if (!$value$plusargs("ECY=%d", exp_cy)) exp_cy = 360;
        if (!$value$plusargs("ER=%d", exp_r)) exp_r = 60;
        if (!$value$plusargs("EVALID=%d", exp_valid)) exp_valid = 1;
        if (!$value$plusargs("ETOLC=%d", exp_tol_c)) exp_tol_c = 4;
        if (!$value$plusargs("ETOLR=%d", exp_tol_r)) exp_tol_r = 3;
        $readmemh(fname, mem);
        n = mem[0][31:0];
        $display("TB file=%0s events=%0d", fname, n);
        done = 0;
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        @(negedge clk); frame_start = 1;
        @(negedge clk); frame_start = 0;
        for (i = 1; i <= n; i = i + 1) begin
            @(negedge clk);
            ev_valid  = 1;
            ev_x      = mem[i][47:36];
            ev_y      = mem[i][35:24];
            ev_ux     = $signed(mem[i][23:14]);
            ev_uy     = $signed(mem[i][13:4]);
            ev_sector = mem[i][3:0];
        end
        @(negedge clk);
        ev_valid = 0;
        frame_end = 1;
        @(negedge clk);
        frame_end = 0;

        for (t = 0; t < 3000000 && !done; t = t + 1) @(posedge clk);
        repeat (5) @(negedge clk);
        $display("DONE=%0d valid=%0d cx=%0d cy=%0d r=%0d score=%0d flags=%02h count=%0d",
                 done, O_valid, O_cx, O_cy, O_r, O_score, O_flags, O_count_dbg);
        for (i = 0; i < 4; i = i + 1)
            $display(" top%0d cnt=%0d x=%0d y=%0d c_r=%0d cov=%0d sc=%0d ok=%0d",
                     i, u_dut.top_cnt[i], u_dut.top_x[i], u_dut.top_y[i],
                     u_dut.c_r[i], u_dut.c_cov[i], u_dut.c_score[i], u_dut.c_ok[i]);
        $display(" dbg st=%0d cand=%0d pass=%0d r0=%0d cx=%0d cy=%0d peak_v=%0d",
                 u_dut.st, u_dut.cand, u_dut.pass, u_dut.r0, u_dut.cx, u_dut.cy, u_dut.peak_v);
        if (!done) begin $display("  [FAIL] timeout"); errors=errors+1; end
        else begin
            if (O_valid !== exp_valid[0]) begin
                $display("  [FAIL] valid %0d != %0d", O_valid, exp_valid); errors=errors+1; end
            if (exp_valid && O_valid) begin
                if (O_r > exp_r + exp_tol_r || O_r + exp_tol_r < exp_r) begin
                    $display("  [FAIL] r %0d != %0d +-%0d", O_r, exp_r, exp_tol_r); errors=errors+1; end
                if ((O_cx > exp_cx + exp_tol_c) || (O_cx + exp_tol_c < exp_cx) ||
                    (O_cy > exp_cy + exp_tol_c) || (O_cy + exp_tol_c < exp_cy)) begin
                    $display("  [FAIL] center (%0d,%0d) != (%0d,%0d) +-%0d", O_cx,O_cy,exp_cx,exp_cy,exp_tol_c);
                    errors=errors+1; end
            end
        end
        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
