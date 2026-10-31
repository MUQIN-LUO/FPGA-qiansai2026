// ==========================================================================
// tb_display_regression — full isp_top display-path baseline comparison
// --------------------------------------------------------------------------
// The same testbench can be compiled against the 074ae1b baseline RTL or the
// phase-1 shared-gray RTL.  It drives real 1280x720 line/frame timing and:
//   * switches RAW / EDGE / GRAY / POP at frame boundaries, plus one
//     intra-frame POP -> RAW switch;
//   * counts every display valid beat and tlast per 720-line frame group;
//   * hashes the selected 96-bit pixel stream (sel_data) per group;
//   * counts display tuser pulses and off-valid tuser (the AWB display tuser
//     is intentionally early; it is reported, not treated as an error);
//   * counts O_frm_tgl transitions, which must keep running even when the
//     detector has no event;
//   * reports the existing POP line-start pixel-difference diagnostic without
//     turning it into a pass/fail criterion (the 074ae1b baseline shows the
//     same count; this TB is for A/B comparison, not a new POP claim).
//
// Baseline A/B usage:
//   compile 074ae1b isp_top.v + edge_detect_96.v + isp_pop96.v into lib_base,
//   run this TB, then compile the phase-1 files into lib_new and run again.
//   The per-group VALID/TLAST/HASH lines must match.
// ==========================================================================
`timescale 1ns/1ps
module tb_display_regression;

    localparam integer W = 1280;
    localparam integer H = 720;
    localparam integer LINE_CYCLES = 2300;
    localparam integer ACTIVE = W/4;
    localparam integer HBLANK = LINE_CYCLES - ACTIVE - 1;
    localparam integer VBLANK_LINES = 30;
    localparam integer FRAMES = 5;

    reg aclk = 0, cclk = 0;
    always #4.83 aclk = ~aclk;
    always #10.0 cclk = ~cclk;

    reg rst_n = 0;
    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [39:0] tdata = 0;
    reg [9:0]  tdest = 0;
    reg [1:0]  mode = 3;

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
        .O_tvalid(o_tvalid), .I_tready(i_tready)
    );

    wire [95:0] d_sel = u_isp_top.sel_data;
    wire        v_sel = u_isp_top.sel_valid;
    wire        u_sel = u_isp_top.sel_user;
    wire        l_sel = u_isp_top.sel_last;

    // ---------------- Input driver ----------------
    function [9:0] row_val;
        input integer yy;
        begin
            // 8-row blocks, plus a horizontal ramp so EDGE/GRAY/POP all see
            // non-trivial data.  Values stay inside the RAW10 range.
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
        input integer mid_switch;   // 1 = POP -> RAW at line 360
        integer yy;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                if (mid_switch && yy == 360) mode = 2'd0;
                drive_line(yy, (yy == 0));
            end
            repeat (VBLANK_LINES * LINE_CYCLES) @(negedge aclk);
        end
    endtask

    // ---------------- Output observation ----------------
    integer errors = 0;
    integer grp = 0;
    integer g_valid = 0, g_tlast = 0, g_tuser = 0, g_bad = 0;
    reg [63:0] g_hash = 64'd0;
    integer global_valid = 0, global_tlast = 0, global_tuser_off = 0;
    integer toggle_cnt = 0;
    reg     frm_d = 0;
    integer grp_first = 1;
    integer max_diff, dr, dg, db, kk;
    reg     prev_last = 0;
    integer line_start_chk = 0, line_start_bad = 0;
    integer j;

    always @(posedge aclk) begin
        #1;
        // O_frm_tgl must toggle once per input frame, independent of detector
        // events and independent of the selected display mode.
        if (frm_tgl !== frm_d) begin
            toggle_cnt = toggle_cnt + 1;
            frm_d = frm_tgl;
        end

        // Display tuser is intentionally early (off valid); count it anyway.
        if (u_sel) begin
            g_tuser = g_tuser + 1;
            if (!v_sel) global_tuser_off = global_tuser_off + 1;
        end

        if (v_sel) begin
            global_valid = global_valid + 1;
            g_valid = g_valid + 1;
            g_hash = (g_hash << 1) ^ (g_hash >> 63)
                   ^ d_sel[63:0] ^ {32'd0, d_sel[95:64]};
            if (l_sel) begin
                g_tlast = g_tlast + 1;
                global_tlast = global_tlast + 1;
            end

            // POP line-start diagnostic (same criterion as the old TB).
            if ((mode == 3) && (prev_last || u_sel)) begin
                line_start_chk = line_start_chk + 1;
                max_diff = 0;
                for (kk = 1; kk < 4; kk = kk + 1) begin
                    dr = (d_sel[95:88] > d_sel[95-kk*24 -: 8]) ? (d_sel[95:88] - d_sel[95-kk*24 -: 8]) : (d_sel[95-kk*24 -: 8] - d_sel[95:88]);
                    dg = (d_sel[87:80] > d_sel[87-kk*24 -: 8]) ? (d_sel[87:80] - d_sel[87-kk*24 -: 8]) : (d_sel[87-kk*24 -: 8] - d_sel[87:80]);
                    db = (d_sel[79:72] > d_sel[79-kk*24 -: 8]) ? (d_sel[79:72] - d_sel[79-kk*24 -: 8]) : (d_sel[79-kk*24 -: 8] - d_sel[79:72]);
                    if (dr > max_diff) max_diff = dr;
                    if (dg > max_diff) max_diff = dg;
                    if (db > max_diff) max_diff = db;
                end
                if (max_diff > 3) begin
                    line_start_bad = line_start_bad + 1;
                    g_bad = g_bad + 1;
                end
            end
            prev_last <= l_sel;
        end
        else if (u_sel) begin
            // display tuser is allowed here (AWB O_tuser is early)
        end

        // A frame group is 720 tlasts.  Record it and reset.  Skip the first
        // partial group after reset.
        if (g_tlast >= H) begin
            grp = grp + 1;
            if (!grp_first) begin
                $display("GROUP %0d mode=%0d VALID=%0d TLAST=%0d TUSER=%0d HASH=%016h POP_BAD=%0d",
                         grp-1, mode, g_valid, g_tlast, g_tuser, g_hash, g_bad);
                if (g_valid != 230400) begin
                    $display("  [FAIL] group valid %0d != 230400", g_valid);
                    errors = errors + 1;
                end
                if (g_tlast != 720) begin
                    $display("  [FAIL] group tlast %0d != 720", g_tlast);
                    errors = errors + 1;
                end
                if (g_tuser != 1) begin
                    $display("  [FAIL] group tuser %0d != 1", g_tuser);
                    errors = errors + 1;
                end
            end
            else begin
                $display("GROUP 0 (partial after reset) mode=%0d VALID=%0d TLAST=%0d TUSER=%0d HASH=%016h POP_BAD=%0d",
                         mode, g_valid, g_tlast, g_tuser, g_hash, g_bad);
                grp_first = 0;
            end
            g_valid = 0; g_tlast = 0; g_tuser = 0; g_hash = 64'd0; g_bad = 0;
        end
    end

    integer f, nframes;
    integer m0, m1, m2, m3, midsw;
    initial begin
        if (!$value$plusargs("FRAMES=%d", nframes)) nframes = FRAMES;
        if (!$value$plusargs("M0=%d", m0)) m0 = 3;  // POP
        if (!$value$plusargs("M1=%d", m1)) m1 = 0;  // RAW
        if (!$value$plusargs("M2=%d", m2)) m2 = 2;  // GRAY
        if (!$value$plusargs("M3=%d", m3)) m3 = 1;  // EDGE
        if (!$value$plusargs("MIDSW=%d", midsw)) midsw = 1;
        repeat (20) @(negedge aclk);
        rst_n = 1;
        repeat (20) @(negedge aclk);

        // Default: POP, RAW, GRAY, EDGE, then POP with an intra-frame
        // POP -> RAW switch at line 360.  A 2-frame run can select e.g.
        // +M0=2 +M1=1 +MIDSW=0 to compare GRAY then EDGE only.
        for (f = 0; f < nframes; f = f + 1) begin
            case (f)
                0: mode = m0[1:0];
                1: mode = m1[1:0];
                2: mode = m2[1:0];
                3: mode = m3[1:0];
                default: mode = m0[1:0];
            endcase
            drive_frame(((f == nframes-1) && (midsw != 0)) ? 1 : 0);
        end

        repeat (5000) @(negedge aclk);
        $display("TOTAL VALID=%0d TLAST=%0d TUSER_OFFVALID=%0d FRM_TGL_TOGGLES=%0d GROUPS=%0d",
                 global_valid, global_tlast, global_tuser_off, toggle_cnt, grp);
        $display("LINE_START_CHECKS=%0d POP_BAD=%0d", line_start_chk, line_start_bad);
        if (toggle_cnt == 0) begin
            $display("  [FAIL] O_frm_tgl never toggled");
            errors = errors + 1;
        end
        if (global_valid != nframes*230400) begin
            $display("  [FAIL] total valid %0d != %0d", global_valid, nframes*230400);
            errors = errors + 1;
        end
        if (global_tlast != nframes*720) begin
            $display("  [FAIL] total tlast %0d != %0d", global_tlast, nframes*720);
            errors = errors + 1;
        end
        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
