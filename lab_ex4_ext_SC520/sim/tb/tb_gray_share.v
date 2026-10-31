// ==========================================================================
// tb_gray_share — phase-1 shared-gray focused regression
// --------------------------------------------------------------------------
// What is actually checked (same RTL that is synthesized):
//   1) isp_rgb2y96 Y = R/4 + G/2 + B/4, bit-exact for directed/random pixels.
//   2) isp_gray_tap96 1-cycle stream: Y/valid/tlast/display-tuser/detect-tuser
//      and (x,y) coordinates, including an early display tuser on valid=0.
//   3) Display EDGE: edge_detect_y96 (shared Y) vs edge_detect_96 (RGB input)
//      pixel-identical after the documented 1-cycle alignment.
//   4) POP: shared-Y mode vs internal-Y mode pixel-identical (HUE_AWARE=1).
//   5) GRAY display: shared Y + 1 register vs the baseline 2-register gray
//      path, pixel-identical.
//   6) Wide-tap coordinates at x=1276 -> 0 / y+1, tlast passthrough.
//
// It does NOT claim full-resolution Sobel: EDGE still uses the original
// two-samples-per-beat, one-line-buffer algorithm.
// ==========================================================================
`timescale 1ns/1ps
module tb_gray_share;

    localparam integer SW = 16;   // small frame width, 4 beats/line
    localparam integer SH = 4;
    localparam integer SB = SW/4;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;
    reg rst_wide = 0;

    reg [95:0] rgb = 0;
    reg        valid = 0, tuser_det = 0, tuser_disp = 0, tlast = 0;
    reg [1:0]  pal = 0;

    // ---------------- DUT: shared luma + small tap ----------------
    wire [31:0] y_comb;
    isp_rgb2y96 u_y (
        .I_rgb(rgb),
        .O_y4 (y_comb)
    );

    wire [31:0] ty4;
    wire        tv, tl, tud, tue;
    wire [11:0] tx, ty;

    isp_gray_tap96 #(
        .IMG_WIDTH (SW),
        .IMG_HEIGHT(SH)
    ) u_tap_small (
        .I_clk        (clk),
        .I_rst_n      (rst_n),
        .I_y4         (y_comb),
        .I_valid      (valid),
        .I_tlast      (tlast),
        .I_tuser_disp (tuser_disp),
        .I_tuser_det  (tuser_det),
        .O_y4         (ty4),
        .O_valid      (tv),
        .O_tlast      (tl),
        .O_tuser_disp (tud),
        .O_tuser_det  (tue),
        .O_x          (tx),
        .O_y          (ty)
    );

    // ---------------- DUT: wide tap, separate reset ----------------
    wire [31:0] wy4;
    wire        wv, wl, wud, wue;
    wire [11:0] wx, wy;

    isp_gray_tap96 #(
        .IMG_WIDTH (1280),
        .IMG_HEIGHT(720)
    ) u_tap_wide (
        .I_clk        (clk),
        .I_rst_n      (rst_wide),
        .I_y4         (y_comb),
        .I_valid      (valid),
        .I_tlast      (tlast),
        .I_tuser_disp (tuser_disp),
        .I_tuser_det  (tuser_det),
        .O_y4         (wy4),
        .O_valid      (wv),
        .O_tlast      (wl),
        .O_tuser_disp (wud),
        .O_tuser_det  (wue),
        .O_x          (wx),
        .O_y          (wy)
    );

    // ---------------- DUT: old RGB edge vs new shared-Y edge ----------------
    // Two pairs, because isp_top uses two different tuser conventions:
    //   display EDGE  : early tuser, unchanged on-screen alignment
    //   foreground    : valid-aligned tuser (detection path)
    wire        eod_l, eod_u, eod_v;
    wire [95:0] eod_d;
    wire        end_l, end_u, end_v;
    wire [95:0] end_d;

    wire        eot_l, eot_u, eot_v;
    wire [95:0] eot_d;
    wire        ent_l, ent_u, ent_v;
    wire [95:0] ent_d;

    edge_detect_96 #(
        .IMG_WIDTH (SW),
        .IMG_HEIGHT(SH),
        .EDGE_TH   (11'd48)
    ) u_edge_old_disp (
        .I_clk   (clk),
        .I_rst_n (rst_n),
        .I_tlast (tlast),
        .I_tuser (tuser_disp),
        .I_tdata (rgb),
        .I_tvalid(valid),
        .I_tready(1'b1),
        .O_tlast (eod_l),
        .O_tuser (eod_u),
        .O_tdata (eod_d),
        .O_tvalid(eod_v),
        .O_tready()
    );

    edge_detect_y96 #(
        .IMG_WIDTH (SW),
        .IMG_HEIGHT(SH),
        .EDGE_TH   (11'd48)
    ) u_edge_new_disp (
        .I_clk   (clk),
        .I_rst_n (rst_n),
        .I_tlast (tl),
        .I_tuser (tud),
        .I_y4    (ty4),
        .I_tvalid(tv),
        .I_tready(1'b1),
        .O_tlast (end_l),
        .O_tuser (end_u),
        .O_tdata (end_d),
        .O_tvalid(end_v),
        .O_tready()
    );

    edge_detect_96 #(
        .IMG_WIDTH (SW),
        .IMG_HEIGHT(SH),
        .EDGE_TH   (11'd48)
    ) u_edge_old_det (
        .I_clk   (clk),
        .I_rst_n (rst_n),
        .I_tlast (tlast),
        .I_tuser (tuser_det),
        .I_tdata (rgb),
        .I_tvalid(valid),
        .I_tready(1'b1),
        .O_tlast (eot_l),
        .O_tuser (eot_u),
        .O_tdata (eot_d),
        .O_tvalid(eot_v),
        .O_tready()
    );

    edge_detect_y96 #(
        .IMG_WIDTH (SW),
        .IMG_HEIGHT(SH),
        .EDGE_TH   (11'd48)
    ) u_edge_new_det (
        .I_clk   (clk),
        .I_rst_n (rst_n),
        .I_tlast (tl),
        .I_tuser (tue),
        .I_y4    (ty4),
        .I_tvalid(tv),
        .I_tready(1'b1),
        .O_tlast (ent_l),
        .O_tuser (ent_u),
        .O_tdata (ent_d),
        .O_tvalid(ent_v),
        .O_tready()
    );

    // Old edges are 1 cycle earlier than the tap-fed cores; delay for compare.
    reg        eod_l_d, eod_u_d, eod_v_d;
    reg [95:0] eod_d_d;
    reg        eot_l_d, eot_u_d, eot_v_d;
    reg [95:0] eot_d_d;
    always @(posedge clk) begin
        eod_l_d <= eod_l;
        eod_u_d <= eod_u;
        eod_v_d <= eod_v;
        eod_d_d <= eod_d;
        eot_l_d <= eot_l;
        eot_u_d <= eot_u;
        eot_v_d <= eot_v;
        eot_d_d <= eot_d;
    end

    // ---------------- DUT: old POP vs shared-Y POP ----------------
    wire [95:0] po_d, pn_d;

    isp_pop96 #(
        .HUE_AWARE    (1'b1),
        .USE_SHARED_Y (1'b0)
    ) u_pop_old (
        .I_clk   (clk),
        .I_rst_n (rst_n),
        .I_tvalid(valid),
        .I_tuser (tuser_disp),   // isp_top POP uses the early display tuser
        .I_tlast (tlast),
        .I_tdata (rgb),
        .I_y4    (32'd0),
        .I_pal   (pal),
        .O_tdata (po_d)
    );

    isp_pop96 #(
        .HUE_AWARE    (1'b1),
        .USE_SHARED_Y (1'b1)
    ) u_pop_new (
        .I_clk   (clk),
        .I_rst_n (rst_n),
        .I_tvalid(valid),
        .I_tuser (tuser_disp),   // same early tuser as the real display path
        .I_tlast (tlast),
        .I_tdata (rgb),
        .I_y4    (y_comb),
        .I_pal   (pal),
        .O_tdata (pn_d)
    );

    // ---------------- Reference models (spec, not DUT copies) ----------------
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

    function [95:0] rep96;
        input [31:0] y;
        begin
            rep96 = { {y[31:24],y[31:24],y[31:24]},
                      {y[23:16],y[23:16],y[23:16]},
                      {y[15:8], y[15:8], y[15:8] },
                      {y[7:0],  y[7:0],  y[7:0]  } };
        end
    endfunction

    // Expected 1-cycle tap output and 2-cycle display path.
    reg [31:0] e1_y4, e2_y4;
    reg        e1_v, e2_v;
    reg        e1_ud, e1_ue, e1_l;
    reg [11:0] e1_x, e1_y;
    reg [11:0] ecx, ecy;

    reg [95:0] gr1, gr2;          // baseline gray display: 2 registers
    reg [31:0] y_d2;              // new gray display: tap + 1 register

    always @(posedge clk) begin
        if (!rst_n) begin
            e1_y4 <= 32'd0; e2_y4 <= 32'd0;
            e1_v <= 1'b0; e2_v <= 1'b0;
            e1_ud <= 1'b0; e1_ue <= 1'b0; e1_l <= 1'b0;
            e1_x <= 12'd0; e1_y <= 12'd0;
            ecx <= 12'd0; ecy <= 12'd0;
            gr1 <= 96'd0; gr2 <= 96'd0;
            y_d2 <= 32'd0;
        end
        else begin
            e1_y4 <= ref_y4(rgb);
            e1_v  <= valid;
            e1_l  <= tlast;
            e1_ud <= tuser_disp;
            e1_ue <= tuser_det;
            e2_y4 <= e1_y4;
            e2_v  <= e1_v;

            if (valid) begin
                if (tuser_det) begin
                    e1_x <= 12'd0; e1_y <= 12'd0;
                    ecx  <= 12'd4; ecy  <= 12'd0;
                end
                else begin
                    e1_x <= ecx; e1_y <= ecy;
                    if (ecx >= (SW-4)) begin
                        ecx <= 12'd0;
                        ecy <= (ecy >= (SH-1)) ? 12'd0 : ecy + 12'd1;
                    end
                    else begin
                        ecx <= ecx + 12'd4;
                    end
                end
            end

            gr1 <= rep96(ref_y4(rgb));
            gr2 <= gr1;
            y_d2 <= ty4;
        end
    end

    // ---------------- Checks ----------------
    integer errors = 0;
    integer checks = 0;
    integer i;
    integer early_disp_seen = 0;
    integer edge_white_beats = 0;
    reg check_en = 0;

    task fail;
        input [8*80-1:0] what;
        input integer a, b;
        begin
            errors = errors + 1;
            if (errors <= 30)
                $display("  [FAIL] %0s a=%0d b=%0d @%0t", what, a, b, $time);
        end
    endtask

    function [23:0] make_pix;
        input integer l, b, q;
        integer v, r, g, bl;
        begin
            case (l)
                0: begin
                    v = (b < 2) ? 16 : 224;
                    r = v; g = v; bl = v;
                end
                1: begin
                    r = 32 + b*48;
                    g = 64 + q*32;
                    bl = 16 + l*24;
                end
                2: begin
                    r = 240; g = 240; bl = 240;
                end
                default: begin
                    v = (q & 1) ? 16 : 224;
                    r = v; g = v; bl = v;
                end
            endcase
            make_pix = {r[7:0], g[7:0], bl[7:0]};
        end
    endfunction

    function [95:0] make_beat;
        input integer l, b, f;
        begin
            make_beat = { make_pix(l,b,0), make_pix(l,b,1),
                          make_pix(l,b,2), make_pix(l,b,3) };
        end
    endfunction

    task check_small;
        begin
            if (check_en) begin
                checks = checks + 1;
                if (ty4 !== e1_y4) fail("tap_y4", ty4, e1_y4);
                if (tv  !== e1_v ) fail("tap_valid", tv, e1_v);
                if (tl  !== e1_l ) fail("tap_tlast", tl, e1_l);
                if (tud !== e1_ud) fail("tap_tuser_disp", tud, e1_ud);
                if (tue !== e1_ue) fail("tap_tuser_det", tue, e1_ue);
                if (tx  !== e1_x ) fail("tap_x", tx, e1_x);
                if (ty  !== e1_y ) fail("tap_y", ty, e1_y);
                if (tue && !tv)    fail("det_tuser_off_valid", tue, tv);

                // GRAY display (2-cycle shared path) vs baseline 2-reg path
                if (y_d2 !== e2_y4) fail("gray_y2", y_d2, e2_y4);
                if (rep96(y_d2) !== gr2) fail("gray_display", 0, 0);

                // POP shared vs local (same real valid/tuser/tlast inputs)
                if (pn_d !== po_d) fail("pop_pixel", 0, 0);

                // Display EDGE: early tuser path.
                if (end_v && (end_d != 96'd0)) edge_white_beats = edge_white_beats + 1;
                if (end_d !== eod_d_d) fail("edge_disp_data", 0, 0);
                if (end_v !== eod_v_d) fail("edge_disp_valid", end_v, eod_v_d);
                if (end_u !== eod_u_d) fail("edge_disp_tuser", end_u, eod_u_d);
                if (end_l !== eod_l_d) fail("edge_disp_tlast", end_l, eod_l_d);

                // Foreground EDGE: valid-aligned detection tuser path.
                if (ent_d !== eot_d_d) fail("edge_det_data", 0, 0);
                if (ent_v !== eot_v_d) fail("edge_det_valid", ent_v, eot_v_d);
                if (ent_u !== eot_u_d) fail("edge_det_tuser", ent_u, eot_u_d);
                if (ent_l !== eot_l_d) fail("edge_det_tlast", ent_l, eot_l_d);

                // The display tuser is deliberately allowed off valid.
                if (tud && !tv) early_disp_seen = early_disp_seen + 1;
            end
        end
    endtask

    integer l, b, f, q;

    // Wide-tap expected coordinate (independent hard-coded sequence).
    task check_wide;
        input integer idx;
        begin
            checks = checks + 1;
            if (wv !== 1'b1) fail("wide_valid", wv, 1);
            if (wx !== ((idx <= 319) ? idx*4 : 0))
                fail("wide_x", wx, (idx <= 319) ? idx*4 : 0);
            if (wy !== ((idx <= 319) ? 0 : 1))
                fail("wide_y", wy, (idx <= 319) ? 0 : 1);
            if (wl !== ((idx == 319) ? 1'b1 : 1'b0))
                fail("wide_tlast", wl, (idx == 319) ? 1 : 0);
        end
    endtask

    initial begin
        // Directed isp_rgb2y96 formula check (combinational).
        errors = 0;
        rgb = {24'h010203, 24'h7f8081, 24'hfefdfc, 24'h000000};
        #1;
        if (y_comb !== ref_y4(rgb)) begin
            $display("  [FAIL] rgb2y directed %h != %h", y_comb, ref_y4(rgb));
            errors = errors + 1;
        end
        rgb = {24'hff0000, 24'h00ff00, 24'h0000ff, 24'hffffff};
        #1;
        if (y_comb !== ref_y4(rgb)) begin
            $display("  [FAIL] rgb2y primary %h != %h", y_comb, ref_y4(rgb));
            errors = errors + 1;
        end

        repeat (5) @(negedge clk);
        rst_n = 1;
        check_en = 1;

        // Early display tuser on an invalid beat, then a detection tuser on
        // the first valid beat.  This proves the two tuser paths are separate.
        @(negedge clk);
        rgb = 96'd0; valid = 0; tuser_disp = 1; tuser_det = 0; tlast = 0;
        @(negedge clk);
        tuser_disp = 0;

        // Two small frames (16x4): vertical step, colour gradient, flat,
        // checker.  tlast on every line, tuser_det on the first beat.
        // Each frame is preceded by a display tuser on an invalid beat, which
        // is how AWB's O_tuser actually arrives (early, off valid).
        for (f = 0; f < 2; f = f + 1) begin
            @(negedge clk);
            check_small;
            rgb = 96'd0;
            valid = 0;
            tuser_disp = 1; tuser_det = 0; tlast = 0;
            for (l = 0; l < SH; l = l + 1) begin
                for (b = 0; b < SB; b = b + 1) begin
                    @(negedge clk);
                    check_small;
                    rgb = make_beat(l, b, f);
                    valid = 1;
                    tuser_det  = (l == 0) && (b == 0);
                    tuser_disp = 1'b0;   // display tuser was the early pulse
                    tlast      = (b == SB-1);
                    pal        = f[1:0];
                end
            end
        end

        // Flush the pipelines.
        for (i = 0; i < 8; i = i + 1) begin
            @(negedge clk);
            check_small;
            valid = 0; tuser_det = 0; tuser_disp = 0; tlast = 0;
        end

        // ---------------- Wide tap: 1280-wide line + next-line wrap --------
        // Hold the wide tap in reset until now so it starts at (0,0).
        @(negedge clk);
        rst_wide = 1;
        for (i = 0; i < 322; i = i + 1) begin
            @(negedge clk);
            if (i > 0) check_wide(i-1);
            if (i < 321) begin
                rgb = {8'd16,8'd16,8'd16, 8'd32,8'd32,8'd32,
                       8'd48,8'd48,8'd48, 8'd64,8'd64,8'd64};
                valid = 1;
                tuser_det  = (i == 0);
                tuser_disp = (i == 0);
                tlast      = (i == 319);
            end
            else begin
                valid = 0; tuser_det = 0; tuser_disp = 0; tlast = 0;
            end
        end

        repeat (10) @(negedge clk);

        if (early_disp_seen == 0) begin
            $display("  [FAIL] display tuser never observed off valid");
            errors = errors + 1;
        end
        if (edge_white_beats == 0) begin
            $display("  [FAIL] EDGE never produced a non-black beat");
            errors = errors + 1;
        end
        $display("CHECKS=%0d ERRORS=%0d EARLY_DISP_TUSER=%0d EDGE_WHITE_BEATS=%0d",
                 checks, errors, early_disp_seen, edge_white_beats);
        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end

    // Timeout guard.
    initial begin
        #2_000_000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
