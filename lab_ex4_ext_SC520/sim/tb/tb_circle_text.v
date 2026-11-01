`timescale 1ns/1ps
// ==========================================================================
// CIRCLE text overlay bounded test (hdmi_mixer only, synthetic 1280x720 raster)
//
// Checks:
//   - signed coordinates +X right / +Y down, zero shown as +000
//   - positive / negative / zero cases (glyph sequence checked through the
//     real mixer's registered text character stream)
//   - four screen corners: placement clamped inside the active area, text
//     bbox never leaves 0..1279 x 0..719
//   - green glyphs + black outline (both present; outline neighbours exist)
//   - text disappears when I_det_valid=0
//   - multi-frame: text follows the same-frame bbox/centre and never goes stale
//
// This is a synthetic display test, not a camera/board test.
// ==========================================================================
module tb_circle_text;
    localparam integer W = 1280, H = 720;
    localparam integer TXT_W = 180, TXT_H = 10, TXT_XMAX = 1096;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        v_vsync = 0, v_hsync = 0, v_de = 0, v_user = 0, v_last = 0;
    reg [23:0] v_data = 24'h202020;

    reg [11:0] d_minx = 0, d_maxx = 0, d_miny = 0, d_maxy = 0;
    reg [11:0] d_cx = 640, d_cy = 360;
    reg        d_valid = 0;

    wire O_rd_en, O_vsync, O_hsync, O_de;
    wire [23:0] O_data;

    hdmi_mixer #(
        .H_OFFSET(0), .V_OFFSET(0),
        .IMG_WIDTH(W), .IMG_HEIGHT(H),
        .DEBUG_MODE(0)
    ) u_dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_video_vsync(v_vsync), .I_video_hsync(v_hsync),
        .I_video_de(v_de), .I_video_user(v_user), .I_video_last(v_last),
        .I_debug_status(4'd0), .I_fps_value(10'd0), .I_mode(2'd0),
        .I_lat(10'd0),
        .I_det_minx(d_minx), .I_det_maxx(d_maxx),
        .I_det_miny(d_miny), .I_det_maxy(d_maxy),
        .I_det_cx(d_cx), .I_det_cy(d_cy),
        .I_det_valid(d_valid), .I_det_cand(1'b0),
        .I_det_cnt(10'd0), .I_det_flags(8'd0),
        .I_red_valid(1'b0),
        .I_red_minx(12'd0), .I_red_maxx(12'd0),
        .I_red_miny(12'd0), .I_red_maxy(12'd0),
        .I_avg_r(8'd0), .I_avg_g(8'd0), .I_avg_b(8'd0),
        .I_gr100(10'd0), .I_gb100(10'd0),
        .I_ae(16'd0), .I_ag(16'd0), .I_cal_st(2'd0),
        .O_video_rd_en(O_rd_en), .I_video_rd_data(v_data),
        .O_hdmi_vsync(O_vsync), .O_hdmi_hsync(O_hsync),
        .O_hdmi_de(O_de), .O_hdmi_data(O_data)
    );

    // ---- output frame capture (using the delayed O_hdmi_de) ------------
    reg [23:0] cap [0:W*H-1];
    integer cap_ox = 0, cap_oy = 0;
    reg     cap_en = 0;
    always @(posedge clk) begin
        if (cap_en) begin
            if (!O_de) begin
                cap_ox = 0;
            end
            else begin
                if (cap_oy < H) cap[cap_oy*W + cap_ox] = O_data;
                if (cap_ox == W-1) begin
                    cap_ox = 0;
                    cap_oy = cap_oy + 1;
                end
                else cap_ox = cap_ox + 1;
            end
        end
    end

    // ---- text glyph sequence sampling ----------------------------------
    // Sample the glyph index in the middle of each 40-px slot.
    reg [4:0] seq [0:17];
    reg [4:0] exp_seq [0:17];
    integer si;
    always @(posedge clk) begin
        if (cap_en && u_dut.S_txt_box_q &&
            (u_dut.S_txt_celly_q == 4'd4) && (u_dut.S_txt_cellx_q == 4'd5)) begin
            si = u_dut.S_txt_slot;
            if (si >= 0 && si < 18) seq[si] = u_dut.S_txt_ch_q;
        end
    end

    integer errors = 0;
    task check;
        input cond;
        input [8*80-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task clear_seq;
        integer i;
        begin
            for (i = 0; i < 18; i = i + 1) seq[i] = 5'd0;
        end
    endtask

    task set_exp18;
        input [4:0] a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17;
        begin
            exp_seq[0]=a0; exp_seq[1]=a1; exp_seq[2]=a2; exp_seq[3]=a3;
            exp_seq[4]=a4; exp_seq[5]=a5; exp_seq[6]=a6; exp_seq[7]=a7;
            exp_seq[8]=a8; exp_seq[9]=a9; exp_seq[10]=a10; exp_seq[11]=a11;
            exp_seq[12]=a12; exp_seq[13]=a13; exp_seq[14]=a14; exp_seq[15]=a15;
            exp_seq[16]=a16; exp_seq[17]=a17;
        end
    endtask

    task check_seq;
        input [8*40-1:0] name;
        integer i; reg ok;
        begin
            ok = 1'b1;
            for (i = 0; i < 18; i = i + 1) begin
                if (seq[i] !== exp_seq[i]) begin
                    ok = 1'b0;
                    $display("  seq[%0d]=%0d expected %0d", i, seq[i], exp_seq[i]);
                end
            end
            check(ok, name);
        end
    endtask

    task set_det;
        input [11:0] mnx, mxx, mny, mxy, cx, cy;
        input v;
        begin
            d_minx=mnx; d_maxx=mxx; d_miny=mny; d_maxy=mxy;
            d_cx=cx; d_cy=cy; d_valid=v;
        end
    endtask

    task drive_frame;
        integer yy, xx, tmo;
        begin
            cap_ox = 0; cap_oy = 0; cap_en = 1; clear_seq;
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 1) begin
                    @(negedge clk);
                    v_de   = 1'b1;
                    v_user = (yy == 0 && xx == 0);
                    v_last = (xx == W-1);
                    v_data = 24'h202020;
                end
                @(negedge clk);
                v_de = 1'b0; v_user = 1'b0; v_last = 1'b0;
                repeat (4) @(negedge clk);
            end
            tmo = 0;
            while ((cap_oy < H) && (tmo < 2000)) begin
                @(posedge clk); tmo = tmo + 1;
            end
            cap_en = 0;
            if (cap_oy < H) begin
                $display("  [FAIL] frame capture timeout (%0d lines)", cap_oy);
                errors = errors + 1;
            end
            repeat (4) @(negedge clk);
        end
    endtask

    // Green/black text pixel bbox and counts inside the placed region.
    integer gcnt, bcnt, gminx, gmaxx, gminy, gmaxy;
    integer bminx, bmaxx, bminy, bmaxy;
    task scan_text;
        input integer x0, y0;
        integer xx, yy;
        begin
            gcnt=0; bcnt=0;
            gminx=W-1; gmaxx=0; gminy=H-1; gmaxy=0;
            bminx=W-1; bmaxx=0; bminy=H-1; bmaxy=0;
            for (yy = y0; yy < y0+TXT_H; yy = yy + 1) begin
                for (xx = x0; xx < x0+TXT_W; xx = xx + 1) begin
                    if (xx >= 0 && xx < W && yy >= 0 && yy < H) begin
                        if (cap[yy*W+xx] == 24'h00FF00) begin
                            gcnt = gcnt + 1;
                            if (xx < gminx) gminx = xx;
                            if (xx > gmaxx) gmaxx = xx;
                            if (yy < gminy) gminy = yy;
                            if (yy > gmaxy) gmaxy = yy;
                        end
                        if (cap[yy*W+xx] == 24'h000000) begin
                            bcnt = bcnt + 1;
                            if (xx < bminx) bminx = xx;
                            if (xx > bmaxx) bmaxx = xx;
                            if (yy < bminy) bminy = yy;
                            if (yy > bmaxy) bmaxy = yy;
                        end
                    end
                end
            end
        end
    endtask

    // Fraction (0..100) of green pixels that have a black pixel within +-2 px.
    integer oratio;
    task scan_outline;
        input integer x0, y0;
        integer xx, yy, dx, dy, near, tot;
        reg found;
        begin
            near = 0; tot = 0;
            for (yy = y0; yy < y0+TXT_H; yy = yy + 1) begin
                for (xx = x0; xx < x0+TXT_W; xx = xx + 1) begin
                    if (xx >= 2 && xx < W-2 && yy >= 2 && yy < H-2 &&
                        cap[yy*W+xx] == 24'h00FF00) begin
                        tot = tot + 1;
                        found = 1'b0;
                        for (dy = -2; dy <= 2; dy = dy + 1)
                            for (dx = -2; dx <= 2; dx = dx + 1)
                                if (cap[(yy+dy)*W+(xx+dx)] == 24'h000000)
                                    found = 1'b1;
                        if (found) near = near + 1;
                    end
                end
            end
            oratio = (tot == 0) ? 0 : (near*100)/tot;
        end
    endtask

    // Dump the captured text region at 1 char per display pixel.  This is the
    // pixel-level visual check: it shows the actual hdmi_mixer output, so a
    // mirrored glyph or a bad bit order is visible per character.
    task dump_text;
        input integer x0, y0;
        input [8*32-1:0] tag;
        integer f, xx, yy;
        begin
            f = $fopen("logs/text_render.txt", "a");
            $fwrite(f, "=== %0s x0=%0d y0=%0d ===\n", tag, x0, y0);
            for (yy = y0; yy < y0+TXT_H; yy = yy + 1) begin
                for (xx = x0; xx < x0+TXT_W; xx = xx + 1) begin
                    if (xx >= 0 && xx < W && yy >= 0 && yy < H) begin
                        if (cap[yy*W+xx] == 24'h00FF00)      $fwrite(f, "G");
                        else if (cap[yy*W+xx] == 24'h000000) $fwrite(f, "k");
                        else                                  $fwrite(f, ".");
                    end
                    else $fwrite(f, ".");
                end
                $fwrite(f, "\n");
            end
            $fclose(f);
        end
    endtask

    task dump_ppm;
        input integer x0, y0;
        integer f, xx, yy;
        begin
            f = $fopen("logs/text_render.ppm", "w");
            $fwrite(f, "P3\n%0d %0d\n255\n", TXT_W, TXT_H);
            for (yy = y0; yy < y0+TXT_H; yy = yy + 1) begin
                for (xx = x0; xx < x0+TXT_W; xx = xx + 1) begin
                    if (xx >= 0 && xx < W && yy >= 0 && yy < H) begin
                        if (cap[yy*W+xx] == 24'h00FF00)      $fwrite(f, "0 255 0\n");
                        else if (cap[yy*W+xx] == 24'h000000) $fwrite(f, "0 0 0\n");
                        else                                  $fwrite(f, "32 32 32\n");
                    end
                    else $fwrite(f, "32 32 32\n");
                end
            end
            $fclose(f);
        end
    endtask

    localparam [4:0] G_C=5'd28, G_I=5'd29, G_R=5'd16, G_L=5'd24, G_E=5'd19,
                     G_SP=5'd15, G_X=5'd14, G_Y=5'd22, G_PLUS=5'd30, G_MINUS=5'd31,
                     G_0=5'd4, G_1=5'd5, G_2=5'd6, G_3=5'd7,
                     G_4=5'd8, G_5=5'd9;

    integer x0, y0;
    initial begin
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        // ---------------- 1. centre, X+123 / Y-045 ----------------------
        $display("\n-- positive/negative: CIRCLE X+123 Y-045 --");
        set_det(500, 700, 400, 600, 763, 315, 1'b1);
        set_exp18(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_PLUS,
                  G_1,G_2,G_3,G_SP,G_Y,G_MINUS,G_0,G_4,G_5);
        drive_frame();
        x0 = 600 - TXT_W/2;          // 510
        y0 = 400 - (TXT_H + 4);      // 386
        check(u_dut.S_txt_x0_q == x0 && u_dut.S_txt_y0_q == y0,
              "placement above the box / centred");
        check(u_dut.S_txt_on_q == 1'b1, "text enabled while box valid");
        check_seq("glyph sequence X+123 Y-045");
        scan_text(x0, y0);
        $display("  text: green=%0d black=%0d gbox=(%0d,%0d)-(%0d,%0d) bbox=(%0d,%0d)-(%0d,%0d)",
                 gcnt, bcnt, gminx, gminy, gmaxx, gmaxy,
                 bminx, bminy, bmaxx, bmaxy);
        check(gcnt > 100 && bcnt > 100, "green glyphs + black outline present");
        // The C glyph's left bearing is font column 1 and the 1-font-pixel
        // outline is at cell column 0; at 4x plus a 3-cycle pipeline skew the
        // first green pixel is ~x0+10 and the first black pixel ~x0+7.
        check(bminx >= x0 && bminx <= x0+8 && bminy >= y0 && bminy <= y0+2,
              "text aligns to the placed top-left (bearing + pipeline skew)");
        check((gmaxy - gminy + 1) == 8, "glyph height is 8 display pixels");
        check(gmaxx <= x0+TXT_W && gmaxy <= y0+TXT_H,
              "text stays inside its region");
        scan_outline(x0, y0);
        $display("  outline: %0d%% of green pixels have a black neighbour", oratio);
        check(oratio > 20, "outline surrounds the glyphs");
        dump_text(bminx, bminy, "center X+123 Y-045");
        dump_ppm(bminx, bminy);

        // ---------------- 2. zero: X+000 / Y+000 ------------------------
        $display("\n-- zero: CIRCLE X+000 Y+000 --");
        set_det(600, 680, 300, 380, 640, 360, 1'b1);
        set_exp18(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_PLUS,
                  G_0,G_0,G_0,G_SP,G_Y,G_PLUS,G_0,G_0,G_0);
        drive_frame();
        x0 = 640 - TXT_W/2; y0 = 300 - (TXT_H + 4);
        check_seq("glyph sequence X+000 Y+000");
        scan_text(x0, y0);
        check(gcnt > 100 && bcnt > 100, "zero coordinate text rendered");

        // ---------------- 3. X-045 / Y+123 ------------------------------
        $display("\n-- negative X: CIRCLE X-045 Y+123 --");
        set_det(495, 695, 383, 583, 595, 483, 1'b1);
        set_exp18(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_MINUS,
                  G_0,G_4,G_5,G_SP,G_Y,G_PLUS,G_1,G_2,G_3);
        drive_frame();
        x0 = 595 - TXT_W/2; y0 = 383 - (TXT_H + 4);
        check_seq("glyph sequence X-045 Y+123");
        scan_text(x0, y0);
        check(gcnt > 100 && bcnt > 100, "negative X text rendered");
        dump_text(bminx, bminy, "negative X-045 Y+123");

        // ---------------- 4. four corners: clamp / no clipping ----------
        $display("\n-- four corners --");
        set_det(0, 60, 0, 60, 30, 30, 1'b1);          // top-left
        drive_frame();
        check(u_dut.S_txt_x0_q == 0 && u_dut.S_txt_y0_q == 64,
              "top-left: x clamped to 0, text below the box");
        scan_text(0, 64);
        check(gmaxx < W && gmaxy < H, "top-left: text inside the frame");

        set_det(1219, 1279, 0, 60, 1249, 30, 1'b1);   // top-right
        drive_frame();
        check(u_dut.S_txt_x0_q == TXT_XMAX && u_dut.S_txt_y0_q == 64,
              "top-right: x clamped to the 3px-margin limit");
        scan_text(TXT_XMAX, 64);
        check(gmaxx < W && gmaxy < H, "top-right: text inside the frame");

        set_det(0, 60, 659, 719, 30, 689, 1'b1);      // bottom-left
        drive_frame();
        check(u_dut.S_txt_x0_q == 0 && u_dut.S_txt_y0_q == (659 - (TXT_H + 4)),
              "bottom-left: x clamped, text above the box");
        scan_text(0, 659-(TXT_H+4));
        check(gmaxx < W && gmaxy < H, "bottom-left: text inside the frame");

        set_det(1219, 1279, 659, 719, 1249, 689, 1'b1); // bottom-right
        drive_frame();
        check(u_dut.S_txt_x0_q == TXT_XMAX &&
              u_dut.S_txt_y0_q == (659 - (TXT_H + 4)),
              "bottom-right: both axes clamped");
        scan_text(TXT_XMAX, 659-(TXT_H+4));
        check(gmaxx < W && gmaxy < H, "bottom-right: text inside the frame");

        // ---------------- 5. box invalid: text disappears ----------------
        $display("\n-- invalid box: text disappears --");
        set_det(500, 700, 400, 600, 763, 315, 1'b0);
        drive_frame();
        scan_text(510, 386);
        check(gcnt == 0 && bcnt == 0, "no text when I_det_valid=0");
        check(u_dut.S_txt_on_q == 1'b0, "text enable cleared");

        // ---------------- 6. multi-frame: no stale text ------------------
        $display("\n-- multi-frame: A -> B -> invalid --");
        set_det(500, 700, 400, 600, 763, 315, 1'b1);
        set_exp18(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_PLUS,
                  G_1,G_2,G_3,G_SP,G_Y,G_MINUS,G_0,G_4,G_5);
        drive_frame();
        check_seq("frame A text");
        // B's text is BELOW its box (y0=64), disjoint from A's old region
        // (y0=356), so the stale-text check is unambiguous.
        set_det(495, 695, 0, 60, 595, 30, 1'b1);
        set_exp18(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_MINUS,
                  G_0,G_4,G_5,G_SP,G_Y,G_MINUS,G_3,G_3,G_0);
        drive_frame();
        check_seq("frame B text (not stale A)");
        scan_text(505, 64);
        check(gcnt > 100, "frame B text rendered at B position");
        dump_text(bminx, bminy, "frame B X-045 Y-330");
        scan_text(510, 386);   // A's old region must be empty
        check(gcnt == 0, "frame A region empty after B");
        set_det(495, 695, 0, 60, 595, 30, 1'b0);
        drive_frame();
        check(u_dut.S_txt_on_q == 1'b0, "invalid frame: text enable cleared");
        // x>=300 avoids the top-left HUD (x=24..264); any green here would be
        // stale B text, not another overlay.
        scan_text(505, 64);
        check(gcnt == 0, "invalid frame removes B text");

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
