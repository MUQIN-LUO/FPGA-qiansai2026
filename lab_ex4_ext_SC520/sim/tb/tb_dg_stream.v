// ==========================================================================
// tb_dg_stream — integrated check of the reserved detection gray stream
// --------------------------------------------------------------------------
// Drives the real isp_top (1280x720 timing) for two frames with different
// display modes (GRAY then EDGE) and checks the new O_dg_* port:
//   * 4 px/beat, O_dg_y4 matches (R/4 + G/2 + B/4) of the AWB output pixel
//     beat, delayed by exactly one CSI clock;
//   * O_dg_valid / O_dg_tuser / O_dg_tlast frame structure: 230400 valid
//     beats and 720 tlasts per frame, one tuser at (0,0);
//   * O_dg_tuser is NEVER off valid (unlike the early display tuser);
//   * O_dg_x = 0,4,...,1276 and O_dg_y = 0..719, restarting at tuser;
//   * the stream is identical under two different I_mode values (it has no
//     display-mode input); only the reference-Y comparison is needed because
//     AWB itself may still be converging frame to frame.
// This does not claim any gradient/Sobel result on this stream.
// ==========================================================================
`timescale 1ns/1ps
module tb_dg_stream;

    localparam integer W = 1280;
    localparam integer H = 720;
    localparam integer LINE_CYCLES = 2300;
    localparam integer ACTIVE = W/4;
    localparam integer HBLANK = LINE_CYCLES - ACTIVE - 1;
    localparam integer VBLANK_LINES = 30;
    localparam integer FRAMES = 2;

    // +LINES=n drives only the first n active lines per frame.  The default
    // 720 is the full frame; a small value is a bounded wiring/alignment test.
    integer nlines;

    reg aclk = 0, cclk = 0;
    always #4.83 aclk = ~aclk;
    always #10.0 cclk = ~cclk;

    reg rst_n = 0;
    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [39:0] tdata = 0;
    reg [9:0]  tdest = 0;
    reg [1:0]  mode = 2;

    wire [11:0] det_minx, det_maxx, det_miny, det_maxy;
    wire det_valid, det_cand;
    wire [9:0] det_cnt;
    wire [7:0] det_flags;
    wire [7:0] rgb_r, rgb_g, rgb_b, lum;
    wire lum_tgl;
    wire [9:0] gr100, gb100;
    wire [1:0] cal_st;
    wire frm_tgl;
    wire [11:0] red_minx, red_maxx, red_miny, red_maxy;
    wire red_valid, red_cand, red_update;
    wire [9:0] red_count;
    wire [127:0] o_tdata;
    wire o_tlast, o_tuser, o_tvalid, i_tready;

    wire [31:0] dg_y4;
    wire dg_valid, dg_tuser, dg_tlast;
    wire [11:0] dg_x, dg_y;

    isp_top u_isp_top (
        .axi4s_video_aclk(aclk),
        .I_ccl_src_clk   (cclk),
        .I_cal_mode      (1'b0),
        .I_rst_n         (rst_n),
        .I_tlast         (tlast),
        .I_tuser         (tuser),
        .I_tdata         (tdata),
        .I_tvalid        (tvalid),
        .I_tdest         (tdest),
        .I_mode          (mode),
        .I_pal           (2'd0),
        .O_det_minx(det_minx), .O_det_maxx(det_maxx),
        .O_det_miny(det_miny), .O_det_maxy(det_maxy),
        .O_det_valid(det_valid), .O_det_cand(det_cand), .O_det_cnt(det_cnt),
        .O_det_flags(det_flags),
        .O_rgb_r(rgb_r), .O_rgb_g(rgb_g), .O_rgb_b(rgb_b),
        .O_lum(lum), .O_lum_tgl(lum_tgl),
        .O_gr100(gr100), .O_gb100(gb100),
        .I_cal_trig(1'b0), .O_cal_st(cal_st), .O_frm_tgl(frm_tgl),
        .O_red_minx(red_minx), .O_red_maxx(red_maxx),
        .O_red_miny(red_miny), .O_red_maxy(red_maxy),
        .O_red_valid(red_valid), .O_red_cand(red_cand),
        .O_red_update(red_update), .O_red_count(red_count),
        .O_tready(1'b1),
        .O_tdata(o_tdata), .O_tlast(o_tlast), .O_tuser(o_tuser),
        .O_tvalid(o_tvalid), .I_tready(i_tready),
        .O_dg_y4(dg_y4), .O_dg_valid(dg_valid), .O_dg_tuser(dg_tuser),
        .O_dg_tlast(dg_tlast), .O_dg_x(dg_x), .O_dg_y(dg_y)
    );

    // ---------------- Input driver ----------------
    function [9:0] row_val;
        input integer yy;
        begin
            row_val = (((yy / 8) % 2) == 0) ? 10'd200 : 10'd800;
        end
    endfunction

    task drive_line;
        input integer yy;
        input integer frame_first;
        integer xx;
        reg [9:0] v;
        begin
            for (xx = 0; xx < W; xx = xx + 4) begin
                @(negedge aclk);
                v = row_val(yy);
                tvalid <= 1'b1;
                tdata  <= {v, v, v, v};
                tuser  <= (frame_first && xx == 0);
                tlast  <= (xx == 1276);
                while (i_tready !== 1'b1) @(negedge aclk);
            end
            @(negedge aclk);
            tvalid <= 1'b0; tuser <= 0; tlast <= 0;
            repeat (HBLANK) @(negedge aclk);
        end
    endtask

    task drive_frame;
        integer yy;
        begin
            for (yy = 0; yy < nlines; yy = yy + 1)
                drive_line(yy, (yy == 0));
            repeat (VBLANK_LINES * LINE_CYCLES) @(negedge aclk);
        end
    endtask

    // ---------------- Reference Y of the AWB pixel beat ----------------
    function [7:0] ref_y8;
        input [23:0] p;
        begin
            ref_y8 = {2'b0,p[23:18]} + {1'b0,p[15:9]} + {2'b0,p[7:2]};
        end
    endfunction

    function [31:0] ref_y4;
        input [95:0] r;
        begin
            ref_y4 = { ref_y8(r[95:72]), ref_y8(r[71:48]),
                       ref_y8(r[47:24]), ref_y8(r[23:0]) };
        end
    endfunction

    reg [31:0] ref_y_d;
    always @(posedge aclk)
        ref_y_d <= ref_y4(u_isp_top.S_rgb_data);

    // ---------------- Checks ----------------
    integer errors = 0;
    integer checks = 0;
    integer frames_done = 0;
    integer f_valid = 0, f_tlast = 0, f_tuser = 0;
    integer global_valid = 0, global_tlast = 0, global_tuser = 0;
    integer off_valid_tuser = 0;
    reg [11:0] ex, ey;
    integer k;

    task fail;
        input [8*80-1:0] what;
        input integer a, b;
        begin
            errors = errors + 1;
            if (errors <= 30)
                $display("  [FAIL] %0s a=%0d b=%0d @%0t", what, a, b, $time);
        end
    endtask

    always @(posedge aclk) begin
        #1;
        if (dg_tuser && !dg_valid) begin
            off_valid_tuser = off_valid_tuser + 1;
            $display("  [FAIL] O_dg_tuser off valid @%0t", $time);
            errors = errors + 1;
        end

        if (dg_tuser) begin
            if (frames_done > 0) begin
                $display("DG_FRAME %0d VALID=%0d TLAST=%0d TUSER=%0d",
                         frames_done-1, f_valid, f_tlast, f_tuser);
                if (f_valid != nlines*(W/4)) begin
                    $display("  [FAIL] dg frame valid %0d != %0d", f_valid, nlines*(W/4));
                    errors = errors + 1;
                end
                if (f_tlast != nlines) begin
                    $display("  [FAIL] dg frame tlast %0d != %0d", f_tlast, nlines);
                    errors = errors + 1;
                end
                if (f_tuser != 1) begin
                    $display("  [FAIL] dg frame tuser %0d != 1", f_tuser);
                    errors = errors + 1;
                end
            end
            frames_done = frames_done + 1;
            f_valid = 0; f_tlast = 0; f_tuser = 0;
        end

        if (dg_valid) begin
            checks = checks + 1;
            global_valid = global_valid + 1;
            f_valid = f_valid + 1;
            if (dg_tlast) begin
                f_tlast = f_tlast + 1;
                global_tlast = global_tlast + 1;
            end
            if (dg_tuser) begin
                f_tuser = f_tuser + 1;
                global_tuser = global_tuser + 1;
                if (dg_x !== 12'd0 || dg_y !== 12'd0)
                    fail("dg_tuser_coord", dg_x, dg_y);
                // This beat is (0,0); the next beat must be (4,0).
                ex = 12'd4; ey = 12'd0;
            end
            else begin
                if (dg_x !== ex || dg_y !== ey)
                    fail("dg_coord", dg_x, ex);
                if (ex >= 12'd1276) begin
                    ex = 12'd0;
                    ey = (ey >= 12'd719) ? 12'd0 : ey + 12'd1;
                end
                else begin
                    ex = ex + 12'd4;
                end
            end

            // Pixel value must be the shared Y of the AWB beat one clock ago.
            if (dg_y4 !== ref_y_d) fail("dg_y4", 0, 0);
            if ((^{dg_y4,ref_y_d}) === 1'bx) fail("dg_y4_x", 0, 0);
        end
    end

    integer f, nframes;
    initial begin
        if (!$value$plusargs("FRAMES=%d", nframes)) nframes = FRAMES;
        if (!$value$plusargs("LINES=%d", nlines)) nlines = H;
        repeat (20) @(negedge aclk);
        rst_n = 1;
        repeat (20) @(negedge aclk);
        for (f = 0; f < nframes; f = f + 1) begin
            mode = (f == 0) ? 2'd2 : 2'd1;   // GRAY then EDGE
            drive_frame;
        end
        repeat (5000) @(negedge aclk);

        // Report/check the last frame (the next tuser never arrives).
        if (frames_done > 0) begin
            $display("DG_FRAME %0d VALID=%0d TLAST=%0d TUSER=%0d (last)",
                     frames_done-1, f_valid, f_tlast, f_tuser);
            if (f_valid != nlines*(W/4)) begin
                $display("  [FAIL] last dg frame valid %0d != %0d", f_valid, nlines*(W/4));
                errors = errors + 1;
            end
            if (f_tlast != nlines) begin
                $display("  [FAIL] last dg frame tlast %0d != %0d", f_tlast, nlines);
                errors = errors + 1;
            end
            if (f_tuser != 1) begin
                $display("  [FAIL] last dg frame tuser %0d != 1", f_tuser);
                errors = errors + 1;
            end
        end
        $display("DG_TOTAL VALID=%0d TLAST=%0d TUSER=%0d OFFVALID_TUSER=%0d FRAMES=%0d CHECKS=%0d",
                 global_valid, global_tlast, global_tuser, off_valid_tuser,
                 frames_done, checks);
        if (frames_done != nframes) begin
            $display("  [FAIL] frames_done %0d != %0d", frames_done, nframes);
            errors = errors + 1;
        end
        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
