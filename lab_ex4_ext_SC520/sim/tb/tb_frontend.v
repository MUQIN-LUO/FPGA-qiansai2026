// ==========================================================================
// 彩色前端 -> 前景掩膜 -> CCL -> 结果 的定向仿真（真实行/帧结构）
// --------------------------------------------------------------------------
// 实际 RTL：edge_detect_96 + isp_bg_est + isp_fg_mask + isp_ccl_bridge
//           （FIFO / 坐标重建 / 颜色一致性 CCL / CDC）
//
// 时序模型（不是均匀插空）：
//   行：320 拍有效像素**连续突发**（每个 MIPI 字节时钟 1 拍），
//       随后 1979 拍行消隐；LINE_CYCLES=2300（22.22 us @103.52 MHz，
//       对应 1280x720@60 的真实行周期）。
//   帧：720 行 + 30 行场消隐 = 750 行 ≈ 16.67 ms。
//   平均有效拍率 = 320/2300*103.52M ≈ 14.4 M beat/s，接近真实 13.8 M/s；
//   突发瞬间 103.5 M beat/s，FIFO 必须靠 512 深度吸收（CCL 25 M cell/s）。
//
// 场景：绿色/红色/蓝色球、手持球、手指压球、彩色/双色背景、无球、方块、
//       卷尺、过小球，以及 40/60/100 px 小尺寸球（含手遮挡/复杂背景）。
//   每场景先跑 warm 帧让背景估计收敛，再评估 1 帧。
//
// 溢出恢复：最后注入一帧“无消隐连续流”（103.5 M beat/s）必然溢出，
//   随后正常帧必须恢复检测（验证 bridge 的坏帧/重同步路径）。
//
// 说明：合成场景，不含摄像头光学/噪声；用于区分真实功能与已知局限。
// ==========================================================================
`timescale 1ns/1ps
module tb_frontend;
    localparam integer W = 1280, H = 720;
    localparam integer LINE_CYCLES = 2300;        // 22.22 us @ 103.52 MHz
    localparam integer ACTIVE_BEATS = W/4;        // 320
    localparam integer HBLANK = LINE_CYCLES - ACTIVE_BEATS - 1;
    localparam integer VBLANK_LINES = 30;

    reg mipi_clk = 0, ccl_clk = 0;
    always #4.83 mipi_clk = ~mipi_clk;            // 103.52 MHz
    always #20.00 ccl_clk = ~ccl_clk;             // 25.00 MHz
    reg rst_n = 0;

    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [95:0] tdata = 0;

    // ---------------- 边缘 ----------------
    wire [95:0] e_tdata; wire e_tvalid, e_tuser, e_tlast;
    edge_detect_96 #(.IMG_WIDTH(W), .IMG_HEIGHT(H), .EDGE_TH(11'd48)) u_edge (
        .I_clk(mipi_clk), .I_rst_n(rst_n),
        .I_tlast(tlast), .I_tuser(tuser), .I_tdata(tdata),
        .I_tvalid(tvalid), .I_tready(1'b1),
        .O_tlast(e_tlast), .O_tuser(e_tuser), .O_tdata(e_tdata),
        .O_tvalid(e_tvalid), .O_tready()
    );

    // ---------------- 背景估计 ----------------
    wire [7:0] bg_r, bg_g, bg_b;
    isp_bg_est #(.IMG_WIDTH(W), .IMG_HEIGHT(H)) u_bg (
        .I_clk(mipi_clk), .I_rst_n(rst_n),
        .I_tvalid(tvalid), .I_tuser(tuser), .I_tdata(tdata),
        .O_bg_r(bg_r), .O_bg_g(bg_g), .O_bg_b(bg_b)
    );

    // ---------------- 前景掩膜 ----------------
    wire [3:0] fg_mask; wire [14:0] fg_rgb;
    wire fg_valid, fg_tuser, fg_tlast; wire [11:0] fg_x, fg_y;
    isp_fg_mask #(.IMG_WIDTH(W), .IMG_HEIGHT(H), .BG_TH(10'd60)) u_fg (
        .I_clk(mipi_clk), .I_rst_n(rst_n),
        .I_tvalid(tvalid), .I_tlast(tlast), .I_tuser(tuser), .I_tdata(tdata),
        .I_bg_r(bg_r), .I_bg_g(bg_g), .I_bg_b(bg_b),
        .I_edge_valid(e_tvalid), .I_edge_tlast(e_tlast),
        .I_edge_tuser(e_tuser), .I_edge_tdata(e_tdata),
        .O_mask(fg_mask), .O_rgb(fg_rgb), .O_valid(fg_valid), .O_tlast(fg_tlast),
        .O_tuser(fg_tuser), .O_x(fg_x), .O_y(fg_y)
    );

    // ---------------- CCL 桥 ----------------
    wire o_evt, o_pass, o_cand, o_ovf;
    wire [11:0] o_w, o_h, o_minx, o_maxx, o_miny, o_maxy;
    wire [23:0] o_cnt; wire [20:0] o_chord; wire [10:0] o_rows;
    wire [3:0] o_lab; wire dbg_full, dbg_drop;
    isp_ccl_bridge #(
        .IMG_WIDTH(W), .IMG_HEIGHT(H),
        .MIN_W_PX(12'd24), .MAX_W_PX(12'd720), .MIN_CELLS(12'd6)
    ) u_ccl (
        .I_mipi_clk(mipi_clk), .I_rst_n(rst_n),
        .I_fg_valid(fg_valid), .I_fg_mask(fg_mask), .I_fg_rgb(fg_rgb),
        .I_fg_tuser(fg_tuser), .I_fg_tlast(fg_tlast),
        .I_det_mask(1'b0), .I_ccl_clk(ccl_clk),
        .O_evt(o_evt), .O_w(o_w), .O_h(o_h), .O_cnt(o_cnt), .O_chord(o_chord),
        .O_rows(o_rows), .O_minx(o_minx), .O_maxx(o_maxx),
        .O_miny(o_miny), .O_maxy(o_maxy),
        .O_pass(o_pass), .O_cand(o_cand),
        .O_ok_size(), .O_ok_asp(), .O_ok_rows(), .O_ok_chord(),
        .O_lab(o_lab), .O_ovf(o_ovf),
        .O_dbg_full(dbg_full), .O_dbg_drop(dbg_drop)
    );

    // ---------------- 场景 ----------------
    localparam [23:0] C_GRAY  = 24'h6e6e6e;
    localparam [23:0] C_GREEN = 24'h28b43c;
    localparam [23:0] C_SKIN  = 24'hc89678;
    localparam [23:0] C_RED   = 24'hd23c3c;
    localparam [23:0] C_BLUEB = 24'h2850c8;
    localparam [23:0] C_BLUEC = 24'h2850a0;
    localparam [23:0] C_BROWN = 24'h78503c;
    localparam [23:0] C_TAPE  = 24'hdcc83c;
    localparam [23:0] C_BLOCK = 24'h3c78c8;

    // 仿真专用整数平方根（仅测试台使用，不参与综合）
    function integer isqrt;
        input integer v;
        integer r, bit, vv;
        begin
            r = 0; bit = 1 << 15; vv = v;
            while (bit > vv) bit = bit >> 1;
            while (bit != 0) begin
                if (vv >= r + bit) begin
                    vv = vv - r - bit;
                    r  = (r >> 1) + bit;
                end
                else r = r >> 1;
                bit = bit >> 1;
            end
            isqrt = r;
        end
    endfunction

    // 球面渐变 + 高光（仿真用近似光照，不是摄像头原始 RGB）。
    //   mode 0 = 只有球面渐变
    //   mode 1 = 原始硬边高光（白核 r=20 + 一步 50% 环 r=33，圆心 590,310）
    //            —— 这是已知边界样例，保留它和真实失败结果，不改成通过。
    //   mode 2 = 平滑径向高光（线性过渡，圆心 600,320）—— 本轮 RTL 修复覆盖的场景
    function [23:0] red_sphere_light;
        input integer px, py, mode;
        integer dx, dy, lit, rr, gg, bb, hx, hy, d2, d, frac;
        begin
            dx = px - 640; dy = py - 360;
            lit = 70 + ((dx + dy + 200) * 3) / 20;   // 70..130（r=100）
            if (lit < 55)  lit = 55;
            if (lit > 130) lit = 130;
            rr = (210 * lit) / 100; if (rr > 255) rr = 255;
            gg = (60  * lit) / 100;
            bb = (60  * lit) / 100;
            if (mode == 1) begin
                // 原始硬边模型：一步跳到 50% 白，保留失败样例
                hx = px - 590; hy = py - 310;
                d2 = hx*hx + hy*hy;
                if (d2 <= 400) begin
                    rr = 235; gg = 228; bb = 220;
                end
                else if (d2 <= 1089) begin
                    rr = (rr + 235) / 2;
                    gg = (gg + 228) / 2;
                    bb = (bb + 220) / 2;
                end
            end
            else if (mode == 2) begin
                // 平滑径向模型：只有白核是严格镜面，过渡按半径线性衰减
                hx = px - 600; hy = py - 320;
                d2 = hx*hx + hy*hy;
                if (d2 <= 2500) begin
                    d = isqrt(d2);
                    if (d <= 20) begin
                        rr = 235; gg = 228; bb = 220;
                    end
                    else begin
                        frac = 50 - d;               // 0..30
                        rr = (rr*(30-frac) + 235*frac) / 30;
                        gg = (gg*(30-frac) + 228*frac) / 30;
                        bb = (bb*(30-frac) + 220*frac) / 30;
                    end
                end
            end
            red_sphere_light = {rr[7:0], gg[7:0], bb[7:0]};
        end
    endfunction

    // 小球场景辅助：ball radius / hand rect / finger rect / background
    function [23:0] scene_pix;
        input integer sc, px, py;
        integer dx, dy, r2, r, i, bx, by, bx0, bx1, by0, by1, bdx, bdy;
        reg [23:0] bgc;
        begin
            dx = px - 640; dy = py - 360; r2 = dx*dx + dy*dy;
            r = 100; bgc = C_GRAY;
            case (sc)
            0: scene_pix = (r2 <= 10000) ? C_GREEN : C_GRAY;                 // 绿球 d=200
            1: begin // 绿球 + 右手（相接）
                   if (r2 <= 10000) scene_pix = C_GREEN;
                   else if (px >= 720 && px <= 980 && py >= 310 && py <= 410)
                       scene_pix = C_SKIN;
                   else scene_pix = C_GRAY;
               end
            2: begin // 绿球 + 远处杂波
                   if (r2 <= 10000) scene_pix = C_GREEN;
                   else begin
                       scene_pix = C_GRAY;
                       for (i = 0; i < 6; i = i + 1) begin
                           bx = 80 + i*190;
                           by = (i < 3) ? 120 : 560;
                           if (px >= bx && px <= bx+60 && py >= by && py <= by+30)
                               scene_pix = C_RED;
                       end
                   end
               end
            3: scene_pix = C_GRAY;                                          // 无球
            4: begin // 黄卷尺
                   scene_pix = C_GRAY;
                   if (px >= 400 && px <= 880 && py >= 340 && py <= 380)
                       scene_pix = C_TAPE;
                   if (px >= 400 && px <= 880 && ((px-400)%40) < 8 &&
                       py >= 381 && py <= 386) scene_pix = C_TAPE;
               end
            5: scene_pix = (r2 <= 10000) ? C_BLUEB : C_GRAY;                // 蓝球
            6: scene_pix = (r2 <= 100)   ? C_GREEN : C_GRAY;                // 过小球 r=10
            7: scene_pix = (r2 <= 10000) ? C_GREEN : C_BLUEC;               // 蓝布底+绿球
            8: begin // 红球 + 右手
                   if (r2 <= 10000) scene_pix = C_RED;
                   else if (px >= 720 && px <= 980 && py >= 310 && py <= 410)
                       scene_pix = C_SKIN;
                   else scene_pix = C_GRAY;
               end
            9: begin // 手指压球右缘 + 手
                   if ((px >= 715 && px <= 745 && py >= 310 && py <= 345) ||
                       (px >= 715 && px <= 745 && py >= 375 && py <= 410))
                       scene_pix = C_SKIN;
                   else if (r2 <= 10000) scene_pix = C_GREEN;
                   else if (px >= 720 && px <= 1000 && py >= 300 && py <= 420)
                       scene_pix = C_SKIN;
                   else scene_pix = C_GRAY;
               end
            10: scene_pix = (r2 <= 10000) ? C_RED : C_GRAY;                 // 红球
            11: begin // 实心方块
                    if (px >= 540 && px <= 740 && py >= 260 && py <= 460)
                        scene_pix = C_BLOCK;
                    else scene_pix = C_GRAY;
                end
            12: scene_pix = (r2 <= 10000) ? C_GREEN :
                            ((px < 900) ? C_BLUEC : C_BROWN);               // 双色底+绿球
            13: begin // 双色底 + 红球 + 手
                    if (r2 <= 10000) scene_pix = C_RED;
                    else if (px >= 720 && px <= 980 && py >= 310 && py <= 410)
                        scene_pix = C_SKIN;
                    else scene_pix = (px < 900) ? C_BLUEC : C_BROWN;
                end
            14: scene_pix = C_BLUEC;                                        // 蓝底无球
            15: begin // 蓝底+卷尺
                    scene_pix = C_BLUEC;
                    if (px >= 400 && px <= 880 && py >= 340 && py <= 380)
                        scene_pix = C_TAPE;
                end

            // ---------------- 本轮新增：小尺寸球 ----------------
            // 16..18：d=100/60/40，灰底，无手
            16,17,18: begin
                    r = (sc==16) ? 50 : (sc==17) ? 30 : 20;
                    scene_pix = (r2 <= r*r) ? C_GREEN : C_GRAY;
                end
            // 19..21：d=100/60/40 + 右手（在球右缘相接）
            19,20,21: begin
                    r  = (sc==19) ? 50 : (sc==20) ? 30 : 20;
                    bx0 = 640 + r;      by0 = 360 - r/2;
                    bx1 = bx0 + 160;    by1 = 360 + r/2;
                    if (r2 <= r*r) scene_pix = C_GREEN;
                    else if (px >= bx0 && px <= bx1 && py >= by0 && py <= by1)
                        scene_pix = C_SKIN;
                    else scene_pix = C_GRAY;
                end
            // 22：d=60 + 手指压球右缘 + 手
            22: begin
                    if (px >= 655 && px <= 675 && py >= 340 && py <= 380)
                        scene_pix = C_SKIN;
                    else if (r2 <= 900) scene_pix = C_GREEN;
                    else if (px >= 670 && px <= 830 && py >= 330 && py <= 390)
                        scene_pix = C_SKIN;
                    else scene_pix = C_GRAY;
                end
            // 23：d=60 蓝布底
            23: begin
                    if (r2 <= 900) scene_pix = C_GREEN; else scene_pix = C_BLUEC;
                end
            // 24：d=60 + 手 + 双色底
            24: begin
                    if (r2 <= 900) scene_pix = C_GREEN;
                    else if (px >= 670 && px <= 830 && py >= 330 && py <= 390)
                        scene_pix = C_SKIN;
                    else scene_pix = (px < 900) ? C_BLUEC : C_BROWN;
                end
            // 25：红球 d=60 + 手
            25: begin
                    if (r2 <= 900) scene_pix = C_RED;
                    else if (px >= 670 && px <= 830 && py >= 330 && py <= 390)
                        scene_pix = C_SKIN;
                    else scene_pix = C_GRAY;
                end
            // 26：d=40 + 手 + 蓝布底（最苛刻的小目标组合）
            26: begin
                    if (r2 <= 400) scene_pix = C_GREEN;
                    else if (px >= 660 && px <= 800 && py >= 340 && py <= 380)
                        scene_pix = C_SKIN;
                    else scene_pix = C_BLUEC;
                end
            // ---- 本轮新增：红球漏检/复杂背景定向场景 ----
            // 27：红球 d=200 球面渐变 + 原始硬边高光（灰底）
            //     已知边界：一步 50% 白环会切碎球体，保留失败样例，不当作通过。
            27: scene_pix = (r2 <= 10000) ? red_sphere_light(px, py, 1) : C_GRAY;
            // 28：红球 d=200 渐变 + 平滑高光 + 手指压球右缘并与手相连
            28: begin
                    if (px >= 700 && px <= 760 && py >= 330 && py <= 395)
                        scene_pix = C_SKIN;                 // 手指压在球面上
                    else if (r2 <= 10000) scene_pix = red_sphere_light(px, py, 2);
                    else if (px >= 760 && px <= 980 && py >= 310 && py <= 410)
                        scene_pix = C_SKIN;                 // 手
                    else scene_pix = C_GRAY;
                end
            // 29：背景里另有一个"合法圆"（蓝盘 d=120）。形状无法判断谁是目标：
            //     本场景只记录实际选中的 bbox，不要求红球必胜。
            29: begin
                    bdx = px - 250; bdy = py - 180;
                    if (r2 <= 10000) scene_pix = red_sphere_light(px, py, 2);
                    else if (bdx*bdx + bdy*bdy <= 3600) scene_pix = C_BLUEC;
                    else scene_pix = C_GRAY;
                end
            // 30：20 个互不相连的背景块，无球（误检/标签耗尽观察）
            30: begin
                    scene_pix = C_GRAY;
                    for (i = 0; i < 20; i = i + 1) begin
                        bx = 60 + (i % 10) * 120;
                        by = (i < 10) ? 80 : 300;
                        if (px >= bx && px <= bx+60 && py >= by && py <= by+30)
                            scene_pix = ((i % 2) == 0) ? C_RED : C_BLOCK;
                    end
                end
            // 31：球前已有 16 个背景块（标签耗尽）+ 红球渐变 d=200
            //     已知边界：15 个标签耗尽后新目标被丢弃；HUD flags[7] 必须置 1。
            31: begin
                    scene_pix = C_GRAY;
                    for (i = 0; i < 16; i = i + 1) begin
                        bx = 60 + (i % 8) * 150;
                        by = (i < 8) ? 80 : 180;
                        if (px >= bx && px <= bx+60 && py >= by && py <= by+30)
                            scene_pix = ((i % 2) == 0) ? C_RED : C_BLOCK;
                    end
                    if (r2 <= 10000) scene_pix = red_sphere_light(px, py, 2);
                end
            // 32：只有球面渐变、没有高光（隔离高光是否为碎片化诱因）
            32: scene_pix = (r2 <= 10000) ? red_sphere_light(px, py, 0) : C_GRAY;
            // 33：平滑径向高光（本轮 RTL 修复覆盖的正常场景）
            33: scene_pix = (r2 <= 10000) ? red_sphere_light(px, py, 2) : C_GRAY;
            default: scene_pix = C_GRAY;
            endcase
        end
    endfunction

    // ---------------- 真实行/帧驱动 ----------------
    task drive_frame;
        input integer sc;
        integer yy, xx, k;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge mipi_clk);
                    tvalid <= 1'b1;
                    tdata  <= { scene_pix(sc,xx+3,yy), scene_pix(sc,xx+2,yy),
                                scene_pix(sc,xx+1,yy), scene_pix(sc,xx,yy) };
                    tuser  <= (xx == 0 && yy == 0);
                    tlast  <= (xx == 1276);        // 真实视频：每行末 tlast
                end
                // 行内 320 拍连续突发结束
                @(negedge mipi_clk);
                tvalid <= 1'b0;
                for (k = 0; k < HBLANK; k = k + 1) @(negedge mipi_clk);
            end
            for (k = 0; k < VBLANK_LINES * LINE_CYCLES; k = k + 1)
                @(negedge mipi_clk);
            tuser <= 0; tlast <= 0;
        end
    endtask

    // 溢出注入：行内连续突发、无任何消隐（平均 103.5 M beat/s）
    task drive_overflow_frame;
        input integer sc;
        integer yy, xx;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge mipi_clk);
                    tvalid <= 1'b1;
                    tdata  <= { scene_pix(sc,xx+3,yy), scene_pix(sc,xx+2,yy),
                                scene_pix(sc,xx+1,yy), scene_pix(sc,xx,yy) };
                    tuser  <= (xx == 0 && yy == 0);
                    tlast  <= (xx == 1276);
                end
                @(negedge mipi_clk);
                tvalid <= 1'b0;
            end
            repeat (400) @(negedge mipi_clk);
            tuser <= 0; tlast <= 0;
        end
    endtask

    // ---------------- 结果 / 调试计数 ----------------
    integer evt_cnt = 0;
    integer dbg_fgv=0, dbg_fgu=0, dbg_fgm=0, dbg_fullcnt=0, dbg_dropcnt=0;
    always @(posedge mipi_clk) begin
        if (fg_valid) dbg_fgv <= dbg_fgv + 1;
        if (fg_valid && fg_tuser) dbg_fgu <= dbg_fgu + 1;
        if (fg_valid && (fg_mask != 4'd0)) dbg_fgm <= dbg_fgm + 1;
        if (dbg_full) dbg_fullcnt <= dbg_fullcnt + 1;
        if (dbg_drop) dbg_dropcnt <= dbg_dropcnt + 1;
    end
    integer only_sc = -1;      // +SCENE_ONLY=n 时只跑该场景（调试用）
    integer last_pass, last_cand, last_w, last_h, last_minx, last_miny, last_maxx, last_maxy, last_ovf;
    always @(posedge mipi_clk) begin
        #1;
        if (o_evt) begin
            evt_cnt <= evt_cnt + 1;
            last_pass <= o_pass; last_cand <= o_cand;
            last_w <= o_w; last_h <= o_h;
            last_minx <= o_minx; last_miny <= o_miny;
            last_maxx <= o_maxx; last_maxy <= o_maxy;
            last_ovf <= o_ovf;
            // 调试：帧尾评估时刻的标签表（S_ev_fin 下一拍才清）
            $display("  L1 par=%0d cnt=%0d rows=%0d x=[%0d,%0d] y=[%0d,%0d] col=%h v=%0d",
                     u_ccl.u_circle_detect.par[1], u_ccl.u_circle_detect.l_cnt[1],
                     u_ccl.u_circle_detect.l_rows[1],
                     u_ccl.u_circle_detect.l_minx[1], u_ccl.u_circle_detect.l_maxx[1],
                     u_ccl.u_circle_detect.l_miny[1], u_ccl.u_circle_detect.l_maxy[1],
                     u_ccl.u_circle_detect.l_col[1], u_ccl.u_circle_detect.l_col_v[1]);
            $display("  L2 par=%0d cnt=%0d rows=%0d x=[%0d,%0d] y=[%0d,%0d] col=%h v=%0d",
                     u_ccl.u_circle_detect.par[2], u_ccl.u_circle_detect.l_cnt[2],
                     u_ccl.u_circle_detect.l_rows[2],
                     u_ccl.u_circle_detect.l_minx[2], u_ccl.u_circle_detect.l_maxx[2],
                     u_ccl.u_circle_detect.l_miny[2], u_ccl.u_circle_detect.l_maxy[2],
                     u_ccl.u_circle_detect.l_col[2], u_ccl.u_circle_detect.l_col_v[2]);
        end
    end

    // 调试：场景 25 的球/手边界与手上下边缘的格颜色
    always @(posedge mipi_clk) begin
        #1;
        if (only_sc == 25 && fg_valid && fg_mask != 4'd0 &&
            (fg_y == 12'd360 || fg_y == 12'd332) &&
            fg_x >= 12'd660 && fg_x <= 12'd700) begin
            $display("  CELL y=%0d x=%0d mask=%b rgb=%h", fg_y, fg_x, fg_mask, fg_rgb);
        end
        // 场景 28：红球/手指边界（看 col_split/highlight 是否误吞肤色）
        if (only_sc == 28 && fg_valid && fg_mask != 4'd0 &&
            (fg_y == 12'd360 || fg_y == 12'd340) &&
            fg_x >= 12'd680 && fg_x <= 12'd780) begin
            $display("  CELL28 y=%0d x=%0d mask=%b rgb=%h in_run=%b run_col=%h split=%b nosplit=%b spec=%b",
                     fg_y, fg_x, fg_mask, fg_rgb,
                     u_ccl.u_circle_detect.in_run,
                     u_ccl.u_circle_detect.run_col,
                     u_ccl.u_circle_detect.col_split,
                     u_ccl.u_circle_detect.no_split,
                     u_ccl.u_circle_detect.specular);
        end
    end

    // CCL 域内采样：O_evt 当拍标签表尚未被 S_ev_fin 清除
    always @(posedge ccl_clk) begin
        #1;
        if (u_ccl.u_circle_detect.O_evt && (only_sc < 0 || only_sc == 25)) begin
            $display("  C1 par=%0d cnt=%0d rows=%0d x=[%0d,%0d] y=[%0d,%0d] col=%h",
                     u_ccl.u_circle_detect.par[1], u_ccl.u_circle_detect.l_cnt[1],
                     u_ccl.u_circle_detect.l_rows[1],
                     u_ccl.u_circle_detect.l_minx[1], u_ccl.u_circle_detect.l_maxx[1],
                     u_ccl.u_circle_detect.l_miny[1], u_ccl.u_circle_detect.l_maxy[1],
                     u_ccl.u_circle_detect.l_col[1]);
            $display("  C2 par=%0d cnt=%0d rows=%0d x=[%0d,%0d] y=[%0d,%0d] col=%h",
                     u_ccl.u_circle_detect.par[2], u_ccl.u_circle_detect.l_cnt[2],
                     u_ccl.u_circle_detect.l_rows[2],
                     u_ccl.u_circle_detect.l_minx[2], u_ccl.u_circle_detect.l_maxx[2],
                     u_ccl.u_circle_detect.l_miny[2], u_ccl.u_circle_detect.l_maxy[2],
                     u_ccl.u_circle_detect.l_col[2]);
        end
    end

    // 评估扫描期逐标签取证（S_ev=5..19 时 e_live 才是稳定值）
    always @(posedge ccl_clk) begin
        #1;
        if (only_sc >= 27 &&
            u_ccl.u_circle_detect.S_ev >= 5'd5 &&
            u_ccl.u_circle_detect.S_ev <= 5'd19 &&
            u_ccl.u_circle_detect.e_sel && u_ccl.u_circle_detect.e_live) begin
            $display("  EVAL M2lab=%0d wpx=%0d hpx=%0d cnt=%0d chord=%0d rows=%0d bbox=[%0d,%0d]-[%0d,%0d] pass=%b cand=%b ok(s=%b a=%b r=%b c=%b) live=%b col=%h",
                     u_ccl.u_circle_detect.M2_lab,
                     u_ccl.u_circle_detect.M2_wpx,
                     u_ccl.u_circle_detect.M2_hpx,
                     u_ccl.u_circle_detect.M2_cnt,
                     u_ccl.u_circle_detect.M2_chord,
                     u_ccl.u_circle_detect.M2_rows,
                     u_ccl.u_circle_detect.M2_minx * 4,
                     u_ccl.u_circle_detect.M2_maxx * 4 + 3,
                     u_ccl.u_circle_detect.M2_miny,
                     u_ccl.u_circle_detect.M2_maxy,
                     u_ccl.u_circle_detect.e_pass,
                     u_ccl.u_circle_detect.e_cand,
                     u_ccl.u_circle_detect.e_size_ok,
                     u_ccl.u_circle_detect.e_asp_ok,
                     u_ccl.u_circle_detect.e_rows_ok,
                     u_ccl.u_circle_detect.e_chord_ok,
                     u_ccl.u_circle_detect.M2_live,
                     u_ccl.u_circle_detect.l_col[u_ccl.u_circle_detect.M2_lab]);
        end
    end

    task run_scene;
        input integer sc;
        input [1023:0] name;
        input integer warm;
        input integer emin, emax;     // 期望 bbox 宽范围（0,0 = 不检查）
        integer e0,v0,u0,m0,f0,d0,w;
        begin : rs_body
            if (only_sc >= 0 && only_sc != sc) disable rs_body;
            for (w = 0; w < warm; w = w + 1) begin
                drive_frame(sc);
                repeat (2000) @(negedge mipi_clk);
            end
            e0=evt_cnt; v0=dbg_fgv; u0=dbg_fgu; m0=dbg_fgm; f0=dbg_fullcnt; d0=dbg_dropcnt;
            drive_frame(sc);
            repeat (4000) @(negedge mipi_clk);
            $display("SCENE %0s: evt=%0d pass=%0d cand=%0d bbox=(%0d,%0d)-(%0d,%0d) w=%0d h=%0d ovf=%0d bg=(%0d,%0d,%0d)",
                     name, evt_cnt-e0, last_pass, last_cand, last_minx, last_miny,
                     last_maxx, last_maxy, last_w, last_h, last_ovf, bg_r, bg_g, bg_b);
            $display("  DBG fg_valid=%0d fg_tuser=%0d fg_hit_beats=%0d full=%0d drop=%0d",
                     dbg_fgv-v0, dbg_fgu-u0, dbg_fgm-m0, dbg_fullcnt-f0, dbg_dropcnt-d0);
            if (emin < 0)
                $display("  SCENE_RESULT %0s: expected KNOWN-LIMIT | actual pass=%0d cand=%0d bbox=(%0d,%0d)-(%0d,%0d) w=%0d h=%0d ovf=%0d",
                         name, last_pass, last_cand, last_minx, last_miny, last_maxx,
                         last_maxy, last_w, last_h, last_ovf);
            else if (emin == 0)
                $display("  SCENE_RESULT %0s: expected no-pass | actual pass=%0d cand=%0d bbox=(%0d,%0d)-(%0d,%0d) w=%0d h=%0d => %0s",
                         name, last_pass, last_cand, last_minx, last_miny, last_maxx,
                         last_maxy, last_w, last_h,
                         (last_pass==0) ? "PASS" : "FAIL");
            else
                $display("  SCENE_RESULT %0s: expected pass w=%0d..%0d | actual pass=%0d cand=%0d bbox=(%0d,%0d)-(%0d,%0d) w=%0d h=%0d => %0s",
                         name, emin, emax, last_pass, last_cand, last_minx, last_miny,
                         last_maxx, last_maxy, last_w, last_h,
                         (last_pass==1 && last_w>=emin && last_w<=emax) ? "PASS" : "FAIL");
        end
    endtask

    integer errors = 0;
    integer full0, drop0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    task check_pass;
        input [1023:0] name;
        input integer emin, emax;
        begin
            check(name, last_pass==1 && last_w>=emin && last_w<=emax);
        end
    endtask

    // 单场景运行时只执行该场景的断言：避免用上一个场景的 last_* 产生假失败。
    task check_sc;
        input integer sc;
        input [1023:0] name;
        input cond;
        begin
            if (only_sc < 0 || only_sc == sc) check(name, cond);
        end
    endtask

    task check_pass_sc;
        input integer sc;
        input [1023:0] name;
        input integer emin, emax;
        begin
            if (only_sc < 0 || only_sc == sc) check_pass(name, emin, emax);
        end
    endtask

    // 已知边界：打印实际结果，但不计入回归错误。
    task limit_note;
        input integer sc;
        input [1023:0] name;
        begin
            if (only_sc < 0 || only_sc == sc)
                $display("  [LIMIT] %0s (actual pass=%0d cand=%0d bbox=(%0d,%0d)-(%0d,%0d) w=%0d h=%0d ovf=%0d)",
                         name, last_pass, last_cand, last_minx, last_miny,
                         last_maxx, last_maxy, last_w, last_h, last_ovf);
        end
    endtask

    // 目标必须在中心球的位置（而不是高光/背景物体）
    task check_ball_bbox_sc;
        input integer sc;
        input [1023:0] name;
        begin
            if (only_sc < 0 || only_sc == sc)
                check(name, last_pass==1 &&
                      last_w>=190 && last_w<=216 && last_h>=190 && last_h<=216 &&
                      last_minx<=560 && last_maxx>=720 &&
                      last_miny<=280 && last_maxy>=440);
        end
    endtask

    initial begin
        if (!$value$plusargs("SCENE_ONLY=%d", only_sc)) only_sc = -1;
        repeat (20) @(negedge mipi_clk);
        rst_n = 1;
        repeat (20) @(negedge mipi_clk);

        // ---- 大球/基础场景 ----
        run_scene(0,  "green ball",                 2, 190,216); check_pass_sc(0, "ball alone passes",190,216);
        run_scene(1,  "green ball + hand",          0, 186,216); check_pass_sc(1, "hand+ball: ball separated",186,216);
        run_scene(2,  "ball + clutter",             0, 186,216); check_pass_sc(2, "ball + clutter: ball passes",186,216);
        run_scene(3,  "no ball",                    0, 0,0);     check_sc(3, "no ball: no pass", last_pass==0);
        run_scene(4,  "yellow tape",                0, 0,0);     check_sc(4, "tape: no pass", last_pass==0);
        run_scene(5,  "blue ball",                  0, 190,216); check_pass_sc(5, "blue ball passes",190,216);
        run_scene(6,  "tiny ball r=10",             0, 0,0);     check_sc(6, "tiny ball: no pass", last_pass==0);
        run_scene(7,  "blue cloth + green ball",    2, 186,216); check_pass_sc(7, "colored bg: ball passes",186,216);
        run_scene(8,  "red ball + hand",            2, 186,216); check_pass_sc(8, "red hand+ball: separated",186,216);
        run_scene(9,  "green ball + fingers",       0, 186,216); check_pass_sc(9, "fingers over ball: passes",186,216);
        run_scene(10, "red ball",                   0, 190,216); check_pass_sc(10, "red ball passes",190,216);
        run_scene(11, "square block",               0, 0,0);     check_sc(11, "square block: no pass", last_pass==0);
        run_scene(12, "two-tone bg + green ball",   2, 186,216); check_pass_sc(12, "two-tone bg: ball passes",186,216);
        run_scene(13, "two-tone bg + red + hand",   0, 186,216); check_pass_sc(13, "two-tone bg hand+ball",186,216);
        run_scene(14, "blue cloth, no ball",        2, 0,0);     check_sc(14, "colored bg no ball: no pass", last_pass==0);
        run_scene(15, "blue cloth + tape",          0, 0,0);     check_sc(15, "colored bg tape: no pass", last_pass==0);

        // ---- 小尺寸球（本轮重点）----
        run_scene(16, "green d=100",                2, 84,116);  check_pass_sc(16, "d=100 passes",84,116);
        run_scene(17, "green d=60",                 0, 48,76);   check_pass_sc(17, "d=60 passes",48,76);
        run_scene(18, "green d=40",                 0, 30,56);   check_pass_sc(18, "d=40 passes",30,56);
        run_scene(19, "green d=100 + hand",         0, 80,116);  check_pass_sc(19, "d=100 + hand separated",80,116);
        run_scene(20, "green d=60 + hand",          0, 44,76);   check_pass_sc(20, "d=60 + hand separated",44,76);
        run_scene(21, "green d=40 + hand",          0, 28,56);   check_pass_sc(21, "d=40 + hand separated",28,56);
        run_scene(22, "green d=60 + finger + hand", 0, 44,76);   check_pass_sc(22, "d=60 fingers: passes",44,76);
        run_scene(23, "green d=60 blue bg",         2, 44,76);   check_pass_sc(23, "d=60 colored bg: passes",44,76);
        run_scene(24, "green d=60 + hand 2-tone",   0, 44,76);   check_pass_sc(24, "d=60 two-tone + hand",44,76);
        run_scene(25, "red d=60 + hand",            2, 44,76);   check_pass_sc(25, "red d=60 + hand",44,76);
        run_scene(26, "green d=40 + hand blue bg",  2, 28,56);   check_pass_sc(26, "d=40 colored bg + hand",28,56);

        // ---- 红球漏检/复杂背景定向场景（短回归：SCENE_ONLY=27..33）----
        // 27 是原始硬边高光失败样例：保留真实结果，作为已知边界报告。
        run_scene(27, "hard-edge highlight (known limit)", 1, -1,0);
        limit_note(27, "hard-edge 50% highlight may fragment the ball");
        run_scene(28, "smooth highlight + contacting finger", 1, 186,216);
        check_ball_bbox_sc(28, "smooth highlight + finger: ball bbox");
        run_scene(29, "red ball + genuine background disc", 1, 100,220);
        check_sc(29, "background disc: a genuine circle is reported; ball-vs-disc not claimable",
                 last_pass==1 &&
                 ((last_minx<=560 && last_maxx>=720) ||      // 红球
                  (last_minx<=200 && last_maxx>=300)));      // 背景蓝盘
        limit_note(29, "two genuine circles: shape alone cannot choose the target");
        run_scene(30, "complex bg, no ball", 1, 0,0);
        check_sc(30, "complex bg no ball: no pass", last_pass==0);
        run_scene(31, "16 bg blobs + red ball", 1, -1,0);
        check_sc(31, "label exhaustion: O_ovf is asserted", last_ovf==1);
        limit_note(31, "15-label capacity exhausted: new target blurred; HUD ovf=1");
        run_scene(32, "red sphere gradient only", 1, 190,216);
        check_ball_bbox_sc(32, "gradient-only ball: bbox on ball");
        run_scene(33, "smooth specular highlight", 1, 190,216);
        check_ball_bbox_sc(33, "smooth highlight: ball bbox");

        // ---- FIFO 溢出注入与恢复（只在完整回归里跑）----
        if (only_sc < 0) begin
            $display("=== overflow injection (no blanking) ===");
            full0 = dbg_fullcnt;
            drive_overflow_frame(0);
            $display("  overflow frame: full_cycles=%0d (sticky drop=%0d)",
                     dbg_fullcnt-full0, dbg_drop);
            check("overflow injection exercised full", (dbg_fullcnt-full0) > 0);

            // 恢复：正常行结构的绿球帧必须重新检出
            run_scene(0, "recovery after overflow", 2, 190,216);
            check("recovery: ball detected after overflow",
                  last_pass==1 && last_w>=190 && last_w<=216);
        end

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
