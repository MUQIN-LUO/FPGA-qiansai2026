// ==========================================================================
// isp_ccl_circle 颜色一致性定向单测（直接给命中掩膜 + 每格颜色）
// --------------------------------------------------------------------------
// 目的：验证"接触但颜色不同 -> 分成两个连通域；同色接触 -> 仍然合并"。
// 不经过 isp_fg_mask / Sobel / 背景估计，只测 CCL 的颜色逻辑；
// 端到端行为由 tb_frontend 覆盖。
//
// 颜色输入与 isp_fg_mask.O_rgb 完全一致：每格 4 像素求平均后取 5:5:5。
// （只取中心像素会漏掉 50/50 混合格，导致红球+肤色手的边界被桥接。）
//
//   场景 0 绿球                       -> pass, bbox 约 204x201
//   场景 1 绿球 + 肤色手（相接）      -> pass, bbox 约 200
//   场景 2 绿球 + 绿色手（同色相接）  -> 合并，pass=0（证明不是几何硬切）
//   场景 3 红球 + 肤色手（相接）      -> pass（50/50 混合格边界）
//   场景 4 两个绿球相接（同色）       -> 合并，pass=0
//   场景 5 绿球 + 蓝手（异色相接）    -> pass
// ==========================================================================
`timescale 1ns/1ps
module tb_ccl_color;
    localparam integer W = 1280, H = 720;
    localparam integer GAP = 8;

    reg clk = 0;
    always #20 clk = ~clk;            // 25 MHz，与 CCL 域一致
    reg rst_n = 0, clr = 0, valid = 0;
    reg [3:0]  hit = 0;
    reg [14:0] rgb = 0;
    reg [11:0] x = 0, y = 0;

    wire o_evt, o_pass, o_cand, o_ovf, o_busy;
    wire [11:0] o_w, o_h, o_minx, o_maxx, o_miny, o_maxy;
    wire [23:0] o_cnt; wire [20:0] o_chord; wire [10:0] o_rows;
    wire [3:0] o_lab;

    isp_ccl_circle #(
        .IMG_WIDTH(W), .IMG_HEIGHT(H),
        .MIN_W_PX(12'd24), .MAX_W_PX(12'd720), .MIN_CELLS(12'd6)
    ) u_dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_clr(clr),
        .I_valid(valid), .I_hit(hit), .I_rgb(rgb), .I_x(x), .I_y(y),
        .O_evt(o_evt), .O_w(o_w), .O_h(o_h), .O_cnt(o_cnt), .O_chord(o_chord),
        .O_rows(o_rows), .O_minx(o_minx), .O_maxx(o_maxx),
        .O_miny(o_miny), .O_maxy(o_maxy),
        .O_pass(o_pass), .O_cand(o_cand),
        .O_ok_size(), .O_ok_asp(), .O_ok_rows(), .O_ok_chord(),
        .O_lab(o_lab), .O_ovf(o_ovf), .O_busy(o_busy)
    );

    localparam [23:0] C_GREEN = 24'h28b43c;
    localparam [23:0] C_SKIN  = 24'hc89678;
    localparam [23:0] C_RED   = 24'hd23c3c;
    localparam [23:0] C_BLUE  = 24'h2850c8;
    localparam [23:0] C_GRAY  = 24'h6e6e6e;

    function [23:0] pix;
        input integer sc, px, py;
        integer dx, dy, r2;
        begin
            dx = px - 640; dy = py - 360; r2 = dx*dx + dy*dy;
            case (sc)
            0: pix = (r2 <= 10000) ? C_GREEN : C_GRAY;
            1: pix = (r2 <= 10000) ? C_GREEN :
                     ((px >= 720 && px <= 980 && py >= 310 && py <= 410) ? C_SKIN : C_GRAY);
            2: pix = (r2 <= 10000) ? C_GREEN :
                     ((px >= 720 && px <= 980 && py >= 310 && py <= 410) ? C_GREEN : C_GRAY);
            3: pix = (r2 <= 10000) ? C_RED :
                     ((px >= 720 && px <= 980 && py >= 310 && py <= 410) ? C_SKIN : C_GRAY);
            4: begin // 两个同色绿球相接（圆心 560/720，半径 100）
                   pix = C_GRAY;
                   dx = px - 560; dy = py - 360; r2 = dx*dx + dy*dy;
                   if (r2 <= 10000) pix = C_GREEN;
                   dx = px - 720; r2 = dx*dx + dy*dy;
                   if (r2 <= 10000) pix = C_GREEN;
               end
            5: pix = (r2 <= 10000) ? C_GREEN :
                     ((px >= 720 && px <= 980 && py >= 310 && py <= 410) ? C_BLUE : C_GRAY);
            // ---- 小尺寸球 + 手（本轮板测关注的实际尺度）----
            6: begin // 红球 d=60 + 手（x=670 相接）
                   if (r2 <= 900) pix = C_RED;
                   else if (px >= 670 && px <= 830 && py >= 330 && py <= 390)
                       pix = C_SKIN;
                   else pix = C_GRAY;
               end
            7: begin // 绿球 d=60 + 手（对照）
                   if (r2 <= 900) pix = C_GREEN;
                   else if (px >= 670 && px <= 830 && py >= 330 && py <= 390)
                       pix = C_SKIN;
                   else pix = C_GRAY;
               end
            8: begin // 绿球 d=40 + 手（x=660 相接）
                   if (r2 <= 400) pix = C_GREEN;
                   else if (px >= 660 && px <= 800 && py >= 340 && py <= 380)
                       pix = C_SKIN;
                   else pix = C_GRAY;
               end
            default: pix = C_GRAY;
            endcase
        end
    endfunction

    function fg;
        input integer sc, px, py;
        begin fg = (pix(sc,px,py) != C_GRAY); end
    endfunction

    task drive_frame;
        input integer sc;
        integer yy, xx, k;
        reg [23:0] c0,c1,c2,c3;
        reg [9:0] sr,sg,sb;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge clk);
                    valid <= 1'b1;
                    x <= xx[11:0];
                    y <= yy[11:0];
                    hit <= { fg(sc,xx+3,yy), fg(sc,xx+2,yy), fg(sc,xx+1,yy), fg(sc,xx,yy) };
                    // 与 isp_fg_mask.O_rgb 一致：4 像素平均后取 5:5:5
                    c0 = pix(sc,xx,yy);   c1 = pix(sc,xx+1,yy);
                    c2 = pix(sc,xx+2,yy); c3 = pix(sc,xx+3,yy);
                    sr = c0[23:16]+c1[23:16]+c2[23:16]+c3[23:16];
                    sg = c0[15:8] +c1[15:8] +c2[15:8] +c3[15:8];
                    sb = c0[7:0]  +c1[7:0]  +c2[7:0]  +c3[7:0];
                    rgb <= { sr[9:5], sg[9:5], sb[9:5] };
                end
                @(negedge clk);
                valid <= 1'b0; hit <= 4'd0;
                repeat (GAP) @(negedge clk);
            end
        end
    endtask

    integer errors = 0;
    integer evt_n = 0, last_pass, last_w, last_h, last_minx, last_maxx, last_miny, last_maxy;
    always @(posedge clk) begin
        #1;
        if (o_evt) begin
            evt_n <= evt_n + 1;
            last_pass <= o_pass; last_w <= o_w; last_h <= o_h;
            last_minx <= o_minx; last_maxx <= o_maxx;
            last_miny <= o_miny; last_maxy <= o_maxy;
        end
    end

    task run_scene;
        input integer sc;
        input [1023:0] name;
        integer e0;
        begin
            e0 = evt_n;
            drive_frame(sc);
            repeat (200) @(negedge clk);
            $display("SCENE %0s: evt=%0d pass=%0d w=%0d h=%0d bbox=(%0d,%0d)-(%0d,%0d)",
                     name, evt_n-e0, last_pass, last_w, last_h,
                     last_minx, last_miny, last_maxx, last_maxy);
        end
    endtask

    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    initial begin
        repeat (20) @(negedge clk);
        rst_n = 1;
        repeat (20) @(negedge clk);

        run_scene(0, "green ball");
        check("green ball passes", last_pass==1 && last_w>=190 && last_w<=216);
        run_scene(1, "green ball + skin hand");
        check("different color: ball separated", last_pass==1 && last_w>=186 && last_w<=216 && last_maxx<=760);
        run_scene(2, "green ball + green hand");
        check("same color: still merged -> no pass", last_pass==0);
        run_scene(3, "red ball + skin hand");
        check("red vs skin: ball separated", last_pass==1 && last_w>=186 && last_w<=216);
        run_scene(4, "two green balls touching");
        check("same color balls merge -> no pass", last_pass==0);
        run_scene(5, "green ball + blue hand");
        check("green vs blue: ball separated", last_pass==1 && last_w>=186 && last_w<=216);
        run_scene(6, "red d=60 + hand");
        check("red d=60 + hand separated", last_pass==1 && last_w>=44 && last_w<=76);
        run_scene(7, "green d=60 + hand");
        check("green d=60 + hand separated", last_pass==1 && last_w>=44 && last_w<=76);
        run_scene(8, "green d=40 + hand");
        check("green d=40 + hand separated", last_pass==1 && last_w>=28 && last_w<=56);

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
