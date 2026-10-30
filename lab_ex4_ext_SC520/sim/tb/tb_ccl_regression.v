// ==========================================================================
// isp_ccl_circle 回归测试平台
//   · DUT      : 当前 user_source 中的 isp_ccl_circle（优化版）
//   · REF      : _timing_opt_artifacts/sim/ref/isp_ccl_circle_ref.v（优化前定版）
//   同一激励并行驱动，逐帧对比全部输出；同时按场景断言功能结论。
//
//   场景（命中掩膜在像素域生成，按 4 像素/拍送入）：
//     0 干净实心圆        1 细手指缺口(边缘 24x60)   2 圆 + 稀疏背景杂波
//     3 无球(仅杂波)      4 实心方块                 5 黄色卷尺(长条+刻度)
//     6 两圆重叠(合并)    7 两圆分离(各成一域)       8 空心圆环(仅轮廓)
//     9 标签溢出(20 个孤立块)                        10 粗手臂遮挡(跨越球体)
//
//   掩膜模型与真实场景的差距：本 TB 直接给"前景命中"，不经过
//   isp_fg_mask(色度/背景差分)、Sobel、传感器噪声与 Bayer 颜色；
//   对应关系是"几何形状进入掩膜后 CCL 的行为"，不是光学等价。
// ==========================================================================
`timescale 1ns/1ps

module tb_ccl_regression;

    localparam integer W     = 1280;
    localparam integer H     = 720;
    localparam integer BEATS = W/4;          // 320 拍/行
    localparam integer GAP   = 8;            // 行间空拍，模拟水平消隐

    // ---------------- 时钟 / 激励 ----------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    reg clr   = 1'b0;
    reg valid = 1'b0;
    reg [3:0]  hit = 4'd0;
    reg [11:0] x   = 12'd0;
    reg [11:0] y   = 12'd0;

    always #5 clk = ~clk;                    // 100 MHz，与 axi4s_video_aclk 同量级

    // ---------------- DUT / REF 输出 ----------------
    wire        d_evt, d_pass, d_cand, d_ok_size, d_ok_asp, d_ok_rows, d_ok_chord, d_ovf;
    wire [11:0] d_w, d_h, d_minx, d_maxx, d_miny, d_maxy;
    wire [23:0] d_cnt;
    wire [20:0] d_chord;
    wire [10:0] d_rows;
    wire [3:0]  d_lab;

    wire        r_evt, r_pass, r_cand, r_ok_size, r_ok_asp, r_ok_rows, r_ok_chord, r_ovf;
    wire [11:0] r_w, r_h, r_minx, r_maxx, r_miny, r_maxy;
    wire [23:0] r_cnt;
    wire [20:0] r_chord;
    wire [10:0] r_rows;
    wire [3:0]  r_lab;

    isp_ccl_circle u_dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_clr(clr),
        .I_valid(valid), .I_hit(hit), .I_x(x), .I_y(y),
        .O_evt(d_evt), .O_w(d_w), .O_h(d_h), .O_cnt(d_cnt), .O_chord(d_chord),
        .O_rows(d_rows), .O_minx(d_minx), .O_maxx(d_maxx), .O_miny(d_miny), .O_maxy(d_maxy),
        .O_pass(d_pass), .O_cand(d_cand),
        .O_ok_size(d_ok_size), .O_ok_asp(d_ok_asp), .O_ok_rows(d_ok_rows), .O_ok_chord(d_ok_chord),
        .O_lab(d_lab), .O_ovf(d_ovf)
    );

    isp_ccl_circle_ref u_ref (
        .I_clk(clk), .I_rst_n(rst_n), .I_clr(clr),
        .I_valid(valid), .I_hit(hit), .I_x(x), .I_y(y),
        .O_evt(r_evt), .O_w(r_w), .O_h(r_h), .O_cnt(r_cnt), .O_chord(r_chord),
        .O_rows(r_rows), .O_minx(r_minx), .O_maxx(r_maxx), .O_miny(r_miny), .O_maxy(r_maxy),
        .O_pass(r_pass), .O_cand(r_cand),
        .O_ok_size(r_ok_size), .O_ok_asp(r_ok_asp), .O_ok_rows(r_ok_rows), .O_ok_chord(r_ok_chord),
        .O_lab(r_lab), .O_ovf(r_ovf)
    );

    // ---------------- 场景发生器 ----------------
    function integer in_scene;
        input integer sc;
        input integer px;
        input integer py;
        integer dx, dy, r2, i, bx, by, hw;
        begin
            dx = px - 640;
            dy = py - 360;
            r2 = dx*dx + dy*dy;
            in_scene = 0;
            case (sc)
            0: in_scene = (r2 <= 10000);
            1: begin // 细缺口：球顶 24x60 的缺口（手指边缘）
                   in_scene = (r2 <= 10000);
                   if (px >= 628 && px <= 652 && py >= 250 && py <= 310) in_scene = 0;
               end
            2: begin // 圆 + 6 个背景物体（远离球体，模拟桌面上其它物体/边缘）
                   if (r2 <= 10000) in_scene = 1;
                   else begin
                       for (i = 0; i < 6; i = i + 1) begin
                           bx = 80 + i*190;
                           by = (i < 3) ? 120 : 560;
                           if (px >= bx && px <= bx+60 && py >= by && py <= by+30) in_scene = 1;
                       end
                   end
               end
            3: in_scene = (((px*px + 3*py*py + px*py*7) % 4096) < 24);
            4: in_scene = (px >= 540 && px <= 740 && py >= 260 && py <= 460);   // 方块 201x201
            5: begin // 卷尺：400x40 长条 + 每 40 px 一个 8x6 刻度块
                   in_scene = (px >= 440 && px <= 840 && py >= 340 && py <= 380);
                   if (px >= 440 && px <= 840 && ((px - 440) % 40) < 8 && py >= 381 && py <= 386) in_scene = 1;
               end
            6: begin // 两圆重叠：圆心 560/720，半径 100
                   dx = px - 560; dy = py - 360;
                   if (dx*dx + dy*dy <= 10000) in_scene = 1;
                   dx = px - 720;
                   if (dx*dx + dy*dy <= 10000) in_scene = 1;
               end
            7: begin // 两圆分离：圆心 320/960
                   dx = px - 320; dy = py - 360;
                   if (dx*dx + dy*dy <= 10000) in_scene = 1;
                   dx = px - 960;
                   if (dx*dx + dy*dy <= 10000) in_scene = 1;
               end
            8: in_scene = (r2 <= 10000) && (r2 >= 6400);   // 空心环
            9: begin // 溢出：孤立 13x13 块 20 个 + 一个圆
                   if (r2 <= 10000) in_scene = 1;
                   for (i = 0; i < 20; i = i + 1) begin
                       bx = 60 + (i % 10) * 120;
                       by = 80 + (i / 10) * 300;
                       if (px >= bx-6 && px <= bx+6 && py >= by-6 && py <= by+6) in_scene = 1;
                   end
               end
            10: begin // 粗手臂：跨越球体的 480x40 前臂（前景，与球相连）
                   in_scene = (r2 <= 10000);
                   if (px >= 400 && px <= 880 && py >= 340 && py <= 380) in_scene = 1;
                end
            11: begin // 大目标在"行末前一拍"合并：上两块，下一整块（非零候选）
                    if (py >= 200 && py <= 299 &&
                        ((px >= 900 && px <= 1000) || (px >= 1100 && px <= 1275))) in_scene = 1;
                    if (py >= 300 && py <= 500 && px >= 900 && px <= 1275) in_scene = 1;
                end
            12: begin // 大目标在最后一拍关闭（cell 319 有命中，非零候选）
                    if (py >= 250 && py <= 530 && px >= 1000 && px <= 1279) in_scene = 1;
                end
            13: begin // 跨行游程：row300 最后一格起，row301 起 100 宽矩形
                    if (py == 300 && px >= 1276) in_scene = 1;
                    if (py >= 301 && py <= 400 && px <= 99) in_scene = 1;
                end
            14: begin // 无空拍密集流：1 格命中 / 1 格间隔，关闭间隔恒为 2 拍
                    if (py >= 100 && py <= 600 && ((px/4) % 2) == 0) in_scene = 1;
                end
            default: in_scene = 0;
            endcase
        end
    endfunction

    // ---------------- 帧发射 ----------------
    task drive_frame;
        input integer sc;
        input integer pre_gap;
        integer xx, yy;
        begin
            repeat (pre_gap) @(negedge clk);   // 帧间消隐，模拟垂直空白
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge clk);
                    valid <= 1'b1;
                    x     <= xx[11:0];
                    y     <= yy[11:0];
                    hit   <= { in_scene(sc, xx+3, yy), in_scene(sc, xx+2, yy),
                               in_scene(sc, xx+1, yy), in_scene(sc, xx, yy) };
                end
                @(negedge clk);
                valid <= 1'b0;
                hit   <= 4'd0;
                repeat (GAP) @(negedge clk);
            end
        end
    endtask

    // ---------------- 结果捕获与检查 ----------------
    integer errors = 0;
    integer n_evt_dut, n_evt_ref;
    integer cap_d_w, cap_d_h, cap_d_cnt, cap_d_chord, cap_d_rows;
    integer cap_d_minx, cap_d_maxx, cap_d_miny, cap_d_maxy;
    integer cap_d_pass, cap_d_cand, cap_d_os, cap_d_oa, cap_d_or, cap_d_oc, cap_d_lab, cap_d_ovf;
    integer cap_r_w, cap_r_h, cap_r_cnt, cap_r_chord, cap_r_rows;
    integer cap_r_minx, cap_r_maxx, cap_r_miny, cap_r_maxy;
    integer cap_r_pass, cap_r_cand, cap_r_os, cap_r_oa, cap_r_or, cap_r_oc, cap_r_lab, cap_r_ovf;

    task fail;
        input [1023:0] msg;
        begin
            $display("  [FAIL] %0s", msg);
            errors = errors + 1;
        end
    endtask

    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors = errors + 1; end
        end
    endtask

    task wait_dut_evt;
        integer n;
        begin
            n = 0;
            while (!d_evt && n < 200) begin @(posedge clk); #1; n = n + 1; end
            if (!d_evt) fail("DUT O_evt timeout");
            cap_d_w=d_w; cap_d_h=d_h; cap_d_cnt=d_cnt; cap_d_chord=d_chord; cap_d_rows=d_rows;
            cap_d_minx=d_minx; cap_d_maxx=d_maxx; cap_d_miny=d_miny; cap_d_maxy=d_maxy;
            cap_d_pass=d_pass; cap_d_cand=d_cand; cap_d_os=d_ok_size; cap_d_oa=d_ok_asp;
            cap_d_or=d_ok_rows; cap_d_oc=d_ok_chord; cap_d_lab=d_lab; cap_d_ovf=d_ovf;
        end
    endtask

    task wait_ref_evt;
        integer n;
        begin
            n = 0;
            while (!r_evt && n < 400) begin @(posedge clk); #1; n = n + 1; end
            if (!r_evt) fail("REF O_evt timeout");
            cap_r_w=r_w; cap_r_h=r_h; cap_r_cnt=r_cnt; cap_r_chord=r_chord; cap_r_rows=r_rows;
            cap_r_minx=r_minx; cap_r_maxx=r_maxx; cap_r_miny=r_miny; cap_r_maxy=r_maxy;
            cap_r_pass=r_pass; cap_r_cand=r_cand; cap_r_os=r_ok_size; cap_r_oa=r_ok_asp;
            cap_r_or=r_ok_rows; cap_r_oc=r_ok_chord; cap_r_lab=r_lab; cap_r_ovf=r_ovf;
        end
    endtask

    task wait_both_evt;
        integer n, gotd, gotr;
        begin
            gotd = 0; gotr = 0; n = 0;
            while ((!gotd || !gotr) && n < 400) begin
                @(posedge clk); #1; n = n + 1;
                if (d_evt && !gotd) begin
                    cap_d_w=d_w; cap_d_h=d_h; cap_d_cnt=d_cnt; cap_d_chord=d_chord; cap_d_rows=d_rows;
                    cap_d_minx=d_minx; cap_d_maxx=d_maxx; cap_d_miny=d_miny; cap_d_maxy=d_maxy;
                    cap_d_pass=d_pass; cap_d_cand=d_cand; cap_d_os=d_ok_size; cap_d_oa=d_ok_asp;
                    cap_d_or=d_ok_rows; cap_d_oc=d_ok_chord; cap_d_lab=d_lab; cap_d_ovf=d_ovf;
                    gotd = 1;
                end
                if (r_evt && !gotr) begin
                    cap_r_w=r_w; cap_r_h=r_h; cap_r_cnt=r_cnt; cap_r_chord=r_chord; cap_r_rows=r_rows;
                    cap_r_minx=r_minx; cap_r_maxx=r_maxx; cap_r_miny=r_miny; cap_r_maxy=r_maxy;
                    cap_r_pass=r_pass; cap_r_cand=r_cand; cap_r_os=r_ok_size; cap_r_oa=r_ok_asp;
                    cap_r_or=r_ok_rows; cap_r_oc=r_ok_chord; cap_r_lab=r_lab; cap_r_ovf=r_ovf;
                    gotr = 1;
                end
            end
            if (!gotd) fail("DUT O_evt timeout");
            if (!gotr) fail("REF O_evt timeout");
        end
    endtask

    integer cap_d_w0, cap_d_h0, cap_d_chord0, cap_d_pass0, cap_d_minx0, cap_d_lab0;
    integer ratio_x1000;
    integer chord_ratio_x1000;
    integer i;
    integer n_scenes;

    task run_scene;
        input integer sc;
        input [1023:0] label;
        integer wc, hc;
        begin
            $display("\n=== scene %0d : %0s ===", sc, label);
            drive_frame(sc, 64);
            wait_both_evt;
            $display("  DUT: pass=%0d cand=%0d ovf=%0d w=%0d h=%0d bbox=(%0d,%0d)-(%0d,%0d) chord=%0d rows=%0d lab=%0d",
                     cap_d_pass, cap_d_cand, cap_d_ovf, cap_d_w, cap_d_h,
                     cap_d_minx, cap_d_miny, cap_d_maxx, cap_d_maxy, cap_d_chord, cap_d_rows, cap_d_lab);
            $display("  REF: pass=%0d cand=%0d ovf=%0d w=%0d h=%0d bbox=(%0d,%0d)-(%0d,%0d) chord=%0d rows=%0d lab=%0d",
                     cap_r_pass, cap_r_cand, cap_r_ovf, cap_r_w, cap_r_h,
                     cap_r_minx, cap_r_miny, cap_r_maxx, cap_r_maxy, cap_r_chord, cap_r_rows, cap_r_lab);
            if (sc == 13) begin
                $display("  [INFO] scene 13: REF(HEAD) has the known cross-row underflow bug;");
                $display("         DUT is checked against expected geometry, not against REF.");
            end
            // 逐字段等价（跨行修复场景除外）
            if (sc != 13 && (cap_d_pass!==cap_r_pass || cap_d_cand!==cap_r_cand || cap_d_ovf!==cap_r_ovf ||
                cap_d_w!==cap_r_w || cap_d_h!==cap_r_h || cap_d_cnt!==cap_r_cnt ||
                cap_d_chord!==cap_r_chord || cap_d_rows!==cap_r_rows ||
                cap_d_minx!==cap_r_minx || cap_d_maxx!==cap_r_maxx ||
                cap_d_miny!==cap_r_miny || cap_d_maxy!==cap_r_maxy ||
                cap_d_os!==cap_r_os || cap_d_oa!==cap_r_oa || cap_d_or!==cap_r_or ||
                cap_d_oc!==cap_r_oc || cap_d_lab!==cap_r_lab))
                fail("DUT != REF");
            else $display("  [ OK ] DUT == REF (all outputs)");

            case (sc)
            0: begin
                check("circle: pass", cap_d_pass == 1);
                check("circle: w within 192..216", cap_d_w >= 192 && cap_d_w <= 216);
                check("circle: h within 192..216", cap_d_h >= 192 && cap_d_h <= 216);
                check("circle: bbox near 540..740 / 260..460",
                      cap_d_minx >= 532 && cap_d_maxx <= 748 && cap_d_miny >= 252 && cap_d_maxy <= 468);
                wc = cap_d_w/4; hc = cap_d_h;
                chord_ratio_x1000 = (wc*hc != 0) ? (cap_d_chord*1000)/(wc*hc) : 0;
                check("circle: chord ratio 720..850", chord_ratio_x1000 >= 720 && chord_ratio_x1000 <= 850);
            end
            1: begin
                check("thin occlusion: still pass", cap_d_pass == 1);
                check("thin occlusion: bbox not exploded", cap_d_w <= 216 && cap_d_h <= 216);
            end
            2: begin
                check("clutter: still pass", cap_d_pass == 1);
                check("clutter: bbox isolated to ball", cap_d_maxx - cap_d_minx <= 216);
            end
            3: begin
                check("no ball: no pass", cap_d_pass == 0);
                check("no ball: no cand either", cap_d_cand == 0);
            end
            4: begin
                check("square: no pass", cap_d_pass == 0);
                check("square: candidate flagged (chord rejects)", cap_d_cand == 1);
            end
            5: begin
                check("tape: no pass", cap_d_pass == 0);
                check("tape: no candidate (aspect rejects)", cap_d_cand == 0);
            end
            6: begin
                check("merged circles: no pass (merged bbox too wide)", cap_d_pass == 0);
            end
            7: begin
                check("separate circles: pass one", cap_d_pass == 1);
                check("separate circles: bbox ~200", cap_d_w <= 216 && cap_d_h <= 216);
            end
            8: begin
                check("hollow ring: pass", cap_d_pass == 1);
            end
            9: begin
                check("overflow: O_ovf asserted", cap_d_ovf == 1);
            end
            11: begin
                check("row-end merge: non-zero candidate", cap_d_cand == 1 && cap_d_chord > 0);
                check("row-end merge: merged bbox ~376x301", cap_d_w >= 360 && cap_d_w <= 390 &&
                      cap_d_h >= 290 && cap_d_h <= 310);
            end
            12: begin
                check("close at last cell: non-zero candidate", cap_d_cand == 1 && cap_d_chord > 0);
                check("close at last cell: bbox ~280x281", cap_d_w >= 270 && cap_d_w <= 290 &&
                      cap_d_h >= 270 && cap_d_h <= 295);
            end
            13: begin
                check("cross-row fix: non-zero candidate", cap_d_cand == 1 && cap_d_chord > 0);
                check("cross-row fix: bbox ~100x100 near x=0", cap_d_w >= 90 && cap_d_w <= 110 &&
                      cap_d_minx <= 8 && cap_d_h >= 90 && cap_d_h <= 110);
            end
            14: begin
                check("dense no-idle: overflow flagged", cap_d_ovf == 1);
                $display("  [INFO] dense no-idle: DUT/REF equality shows A/B pipeline kept up");
            end
            10: begin
                check("wide arm: no pass", cap_d_pass == 0);
                check("wide arm: no candidate (aspect rejects)", cap_d_cand == 0);
            end
            endcase
        end
    endtask

    // ---------------- 主流程 ----------------
    initial begin
`ifdef DUMP_WAVE
        $dumpfile("tb_ccl_regression.vcd");
        $dumpvars(0, tb_ccl_regression);
`endif
        rst_n = 0;
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        run_scene(0,  "clean circle");
        run_scene(1,  "thin occlusion bite");
        run_scene(2,  "circle + sparse clutter");
        run_scene(3,  "no ball (clutter only)");
        run_scene(4,  "solid square");
        run_scene(5,  "yellow tape measure");
        run_scene(6,  "two overlapping circles");
        run_scene(7,  "two separate circles");
        run_scene(8,  "hollow ring");
        run_scene(9,  "label overflow");
        run_scene(10, "wide arm occluder");
        run_scene(11, "row-end merge (non-zero candidate)");
        run_scene(12, "close at last cell (non-zero candidate)");
        run_scene(13, "cross-row run (fix verified)");
        run_scene(14, "dense no-idle closes (every 2 beats)");

        // ---- 连续帧：同一画面两帧背靠背，结果必须逐字段一致 ----
        $display("=== consecutive frames (gap=24) ===");
        drive_frame(0, 24); wait_both_evt;
        cap_d_w0 = cap_d_w; cap_d_h0 = cap_d_h; cap_d_chord0 = cap_d_chord;
        cap_d_pass0 = cap_d_pass; cap_d_minx0 = cap_d_minx; cap_d_lab0 = cap_d_lab;
        drive_frame(0, 24); wait_both_evt;
        check("consecutive frame 2: same pass", cap_d_pass == cap_d_pass0);
        check("consecutive frame 2: same bbox", cap_d_w == cap_d_w0 && cap_d_h == cap_d_h0 &&
              cap_d_minx == cap_d_minx0);
        check("consecutive frame 2: same chord", cap_d_chord == cap_d_chord0);
        check("consecutive frame 2: DUT == REF", cap_d_w == cap_r_w && cap_d_chord == cap_r_chord);

        // mid-frame clear: 状态必须安全复位
        $display("\n=== scene 11 : mid-frame I_clr ===");
        fork
            drive_frame(0, 64);
            begin
                repeat (1000) @(negedge clk);
                clr <= 1'b1; @(negedge clk); clr <= 1'b0;
            end
        join
        wait_both_evt;
        $display("  [INFO] mid-frame clear: pass=%0d (may still rebuild a passing ball)", cap_d_pass);
        check("mid-frame clear: DUT == REF", cap_d_pass == cap_r_pass && cap_d_ovf == cap_r_ovf);

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end

endmodule
