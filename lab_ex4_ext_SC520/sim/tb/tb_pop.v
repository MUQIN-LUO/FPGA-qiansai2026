// ==========================================================================
// isp_pop96 回归：亮度 4 档 / 套印 / 调色板 / 色相区分度 / 流水对齐
// --------------------------------------------------------------------------
// 本轮重点：
//   · HUE_AWARE=1 时同亮度档内暖/绿/冷有可见区分（4 调色板 × 4 档矩阵）；
//   · 模块已流水化（级 1 档位/色相，级 2 调色板+tint），只在 valid 拍推进；
//   · 套印（x-2）跨拍正确；行首/帧首不得使用上一行历史；
//   · 无效拍（valid=0）不得推进套印历史；
//   · 中性色（低色度）保持原调色板。
// 说明：以下为模块级 RTL 证据；isp_top 显示通路的模式切换对齐由
//       tb_isp_top_display.v 覆盖（真实 isp_top）。
// ==========================================================================
`timescale 1ns/1ps
module tb_pop;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg [95:0] tdata = 0; reg [1:0] pal = 0;
    reg tvalid = 0, tuser = 0, tlast = 0;
    wire [95:0] o0, o1;
    isp_pop96 #(.HUE_AWARE(1'b0), .USE_SHARED_Y(1'b0)) u0 (
        .I_clk(clk), .I_rst_n(rst_n), .I_tvalid(tvalid),
        .I_tuser(tuser), .I_tlast(tlast),
        .I_tdata(tdata), .I_y4(32'd0), .I_pal(pal), .O_tdata(o0));
    isp_pop96 #(.HUE_AWARE(1'b1), .USE_SHARED_Y(1'b0)) u1 (
        .I_clk(clk), .I_rst_n(rst_n), .I_tvalid(tvalid),
        .I_tuser(tuser), .I_tlast(tlast),
        .I_tdata(tdata), .I_y4(32'd0), .I_pal(pal), .O_tdata(o1));

    function [95:0] pack4;
        input [23:0] p0,p1,p2,p3;
        begin pack4 = {p0,p1,p2,p3}; end
    endfunction

    integer errors = 0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    function integer manh;
        input [23:0] a;
        input [23:0] b;
        integer dr, dg, db;
        begin
            dr = (a[23:16] > b[23:16]) ? (a[23:16]-b[23:16]) : (b[23:16]-a[23:16]);
            dg = (a[15:8]  > b[15:8])  ? (a[15:8] -b[15:8])  : (b[15:8] -a[15:8]);
            db = (a[7:0]   > b[7:0])   ? (a[7:0]  -b[7:0])   : (b[7:0]  -a[7:0]);
            manh = dr + dg + db;
        end
    endfunction

    // 单个有效拍；采样在下一个 posedge 之后（级 1 寄存完成，O_tdata 组合有效）
    task beat4;
        input [23:0] p0,p1,p2,p3;
        input [1:0]  pl;
        input        tu, tl;
        begin
            @(negedge clk);
            tdata = pack4(p0,p1,p2,p3);
            pal = pl; tuser = tu; tlast = tl; tvalid = 1'b1;
            @(posedge clk);
            #1;
        end
    endtask

    task beat;
        input [23:0] p;
        input [1:0]  pl;
        input        tu, tl;
        begin
            beat4(p,p,p,p, pl, tu, tl);
        end
    endtask

    task idle_beats;
        input integer n;
        integer k;
        begin
            @(negedge clk);
            tvalid = 1'b0; tuser = 0; tlast = 0;
            for (k = 0; k < n; k = k + 1) @(posedge clk);
            #1;
        end
    endtask

    // 每个亮度档的源色（kind: 0=neutral 1=warm 2=green 3=cool）
    function [23:0] src;
        input integer band;
        input integer kind;
        begin
            case (band)
            0: case (kind)
               0: src = 24'h0a0a0a;
               1: src = 24'h100800;
               2: src = 24'h001404;
               default: src = 24'h000410;
               endcase
            1: case (kind)
               0: src = 24'h282828;
               1: src = 24'h281808;
               2: src = 24'h082810;
               default: src = 24'h081028;
               endcase
            2: case (kind)
               0: src = 24'h646464;
               1: src = 24'ha06428;
               2: src = 24'h288c50;
               default: src = 24'h2850c8;
               endcase
            default: case (kind)
               0: src = 24'hc8c8c8;
               1: src = 24'hdcb478;
               2: src = 24'h78dc96;
               default: src = 24'h78a0dc;
               endcase
            endcase
        end
    endfunction

    function integer thresh;
        input integer band;
        begin
            case (band)
                0: thresh = 80;
                1: thresh = 150;
                default: thresh = 192;
            endcase
        end
    endfunction

    function integer thresh_cool;
        input integer band;
        begin
            case (band)
                0: thresh_cool = 48;
                1: thresh_cool = 96;
                default: thresh_cool = 160;
            endcase
        end
    endfunction

    reg [23:0] neu0 [0:3];
    reg [23:0] base_w, base_g, hw, hg, hc;
    reg [23:0] nsrc, wsrc, gsrc, csrc;
    integer p, bd, dist;

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        // ---- 1. 亮度阶梯（中性灰）：HUE_AWARE=0/1 输出一致 ----
        begin : ramp
            reg [23:0] e0c, e1c, e2c, e3c;
            beat(24'h0a0a0a, 0, 1, 0); e0c = o0[95:72];
            check("ramp band0 = 7b2d8e", o0 == {24'h7b2d8e,24'h7b2d8e,24'h7b2d8e,24'h7b2d8e});
            check("ramp neutral: hue-aware == baseline", o1 == o0);
            beat(24'h282828, 0, 1, 0); e1c = o0[95:72];
            check("ramp band1 = ff9ec4", o0 == {24'hff9ec4,24'hff9ec4,24'hff9ec4,24'hff9ec4});
            check("ramp band1 neutral: hue-aware == baseline", o1 == o0);
            beat(24'h646464, 0, 1, 0); e2c = o0[95:72];
            check("ramp band2 = ffd400", o0 == {24'hffd400,24'hffd400,24'hffd400,24'hffd400});
            check("ramp band2 neutral: hue-aware == baseline", o1 == o0);
            beat(24'hc8c8c8, 0, 1, 0); e3c = o0[95:72];
            check("ramp band3 = 00c8c8", o0 == {24'h00c8c8,24'h00c8c8,24'h00c8c8,24'h00c8c8});
            check("ramp band3 neutral: hue-aware == baseline", o1 == o0);
            check("ramp: 4 distinct colors", e0c!=e1c && e1c!=e2c && e2c!=e3c);
        end

        // ---- 2. 同亮度不同色相（本轮重点）----
        beat(24'ha06428, 0, 1, 0); base_w = o0[95:72]; hw = o1[95:72];
        beat(24'h288c50, 0, 1, 0); base_g = o0[95:72]; hg = o1[95:72];
        $display("same-Y band2: base=%h hue warm=%h green=%h dist=%0d",
                 base_w, hw, hg, manh(hw,hg));
        check("same Y -> same luminance-only block", base_w == base_g);
        check("hue-aware: warm != green", hw != hg);
        check("hue-aware: band2 dist >= 192", manh(hw,hg) >= 192);
        check("hue-aware: warm R>G", hw[23:16] > hw[15:8]);
        check("hue-aware: green G>R", hg[15:8] > hg[23:16]);

        // ---- 3. 4 调色板 × 4 亮度档 色相矩阵 ----
        for (p = 0; p < 4; p = p + 1) begin
            for (bd = 0; bd < 4; bd = bd + 1) begin : mat
                nsrc = src(bd,0); wsrc = src(bd,1); gsrc = src(bd,2); csrc = src(bd,3);
                beat(nsrc, p[1:0], 1, 0); neu0[bd] = o1[95:72];
                check("neutral unchanged (pal/band)", o1 == o0);
                beat(wsrc, p[1:0], 1, 0); base_w = o0[95:72]; hw = o1[95:72];
                beat(gsrc, p[1:0], 1, 0); base_g = o0[95:72]; hg = o1[95:72];
                beat(csrc, p[1:0], 1, 0); hc = o1[95:72];
                if (base_w != base_g) begin
                    $display("  [FAIL] pal%0d band%0d: warm/green not same band (%h vs %h)", p, bd, base_w, base_g);
                    errors = errors + 1;
                end
                dist = manh(hw, hg);
                if (hw == hg || dist < thresh(bd)) begin
                    $display("  [FAIL] pal%0d band%0d: warm=%h green=%h dist=%0d < %0d",
                             p, bd, hw, hg, dist, thresh(bd));
                    errors = errors + 1;
                end
                if (!(hw[23:16] > hw[15:8])) begin
                    $display("  [FAIL] pal%0d band%0d: warm not R-dominant", p, bd);
                    errors = errors + 1;
                end
                if (!(hg[15:8] > hg[23:16])) begin
                    $display("  [FAIL] pal%0d band%0d: green not G-dominant", p, bd);
                    errors = errors + 1;
                end
                if (!(hc[7:0] >= hc[23:16] && hc[7:0] >= hc[15:8])) begin
                    $display("  [FAIL] pal%0d band%0d: cool not B-dominant", p, bd);
                    errors = errors + 1;
                end
                if (manh(hw,hc) < thresh_cool(bd) || manh(hg,hc) < thresh_cool(bd)) begin
                    $display("  [FAIL] pal%0d band%0d: cool not distinct warm/green (%0d/%0d)",
                             p, bd, manh(hw,hc), manh(hg,hc));
                    errors = errors + 1;
                end
            end
            check("neutral 4 bands distinct (palette)",
                  neu0[0]!=neu0[1] && neu0[1]!=neu0[2] && neu0[2]!=neu0[3]);
        end
        $display("  [INFO] palette/band hue matrix done (4x4, warm/green/cool).");

        // ---- 4. 套印：上一有效拍 pixel2 档 2 -> 本拍 pixel0 用档 2 ----
        //   上一拍 4 像素档位 = {band3, band2, band0, band0}
        beat4(24'h0a0a0a, 24'h0a0a0a, 24'h646464, 24'hc8c8c8, 0, 1, 0);
        beat (24'hc8c8c8, 0, 0, 0);
        check("overprint: pixel0 uses x-2 band2", o0[95:72] == 24'hffd400);
        check("overprint: pixel1 stays band3", o0[71:48] == 24'h00c8c8);
        check("overprint: pixels2/3 stay band3", o0[47:24] == 24'h00c8c8 && o0[23:0] == 24'h00c8c8);
        check("overprint neutral: hue-aware == baseline", o1 == o0);

        // ---- 5. 行首：上一拍 tlast -> 本拍不得使用上一行 x-2 ----
        beat4(24'h0a0a0a, 24'h0a0a0a, 24'h646464, 24'hc8c8c8, 0, 1, 1); // tlast
        beat (24'hc8c8c8, 0, 0, 0);
        check("line start: no overprint from previous line", o0[95:72] == 24'h00c8c8);

        // ---- 6. 帧首：tuser -> 同样不得使用历史 ----
        beat4(24'h0a0a0a, 24'h0a0a0a, 24'h646464, 24'hc8c8c8, 0, 1, 0);
        beat (24'hc8c8c8, 0, 1, 0);
        check("frame start: no overprint from previous line", o0[95:72] == 24'h00c8c8);

        // ---- 7. 无效拍不得推进套印历史 ----
        beat4(24'h0a0a0a, 24'h0a0a0a, 24'h646464, 24'hc8c8c8, 0, 1, 0); // 历史 pixel2=2
        idle_beats(3);                                                  // valid=0 三拍
        beat (24'hc8c8c8, 0, 0, 0);
        check("invalid beats do not shift overprint history", o0[95:72] == 24'hffd400);

        // ---- 8. 调色板 1 的 4 档（中性）----
        begin : pal1
            reg [23:0] a0,a1,a2,a3;
            beat(24'h0a0a0a, 1, 1, 0); a0=o0[95:72];
            check("palette1 band0 = 00574b", o0 == {24'h00574b,24'h00574b,24'h00574b,24'h00574b});
            beat(24'h282828, 1, 1, 0); a1=o0[95:72];
            check("palette1 band1 = ffe0b2", o0 == {24'hffe0b2,24'hffe0b2,24'hffe0b2,24'hffe0b2});
            beat(24'h646464, 1, 1, 0); a2=o0[95:72];
            check("palette1 band2 = 00e5ff", o0 == {24'h00e5ff,24'h00e5ff,24'h00e5ff,24'h00e5ff});
            beat(24'hc8c8c8, 1, 1, 0); a3=o0[95:72];
            check("palette1 band3 = ff00aa", o0 == {24'hff00aa,24'hff00aa,24'hff00aa,24'hff00aa});
            check("palette1: 4 distinct colors", a0!=a1 && a1!=a2 && a2!=a3);
        end

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
