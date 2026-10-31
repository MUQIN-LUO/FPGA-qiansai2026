// ==========================================================================
// isp_ccl_bridge 回归：**直接实例化设计与综合使用的同一份 RTL**
//   mipi_clk 103.52 MHz 写掩膜；ccl_clk 25 MHz（系统时钟/2）。
//   场景：干净圆两帧、连续帧一致性、帧内强制溢出、满恰逢 tuser、
//         恢复帧检测、所有事件必须 pass=1。
// ==========================================================================
`timescale 1ns/1ps
module tb_bridge;

    reg mipi_clk = 0;
    reg ccl_clk  = 0;
    always #4.83 mipi_clk = ~mipi_clk;   // 103.52 MHz
    always #20.00 ccl_clk = ~ccl_clk;    // 25.00 MHz

    reg        rst_n = 0;
    reg        fg_valid = 0;
    reg [3:0]  fg_mask  = 0;
    reg [14:0] fg_rgb   = 0;      // 统一颜色：关闭颜色一致性，保持几何回归语义
    reg        fg_tuser = 0;
    reg        fg_tlast = 0;
    reg        det_mask = 0;

    wire        o_evt, o_pass, o_cand;
    wire        o_ok_size, o_ok_asp, o_ok_rows, o_ok_chord, o_ovf;
    wire [11:0] o_w, o_h, o_minx, o_maxx, o_miny, o_maxy;
    wire [23:0] o_cnt;
    wire [20:0] o_chord;
    wire [10:0] o_rows;
    wire [3:0]  o_lab;
    wire        dbg_full, dbg_drop;

    isp_ccl_bridge #(
        .IMG_WIDTH (1280), .IMG_HEIGHT(720),
        .MIN_W_PX  (12'd32), .MAX_W_PX (12'd720),
        .MIN_CELLS (12'd8)
    ) dut (
        .I_mipi_clk(mipi_clk), .I_rst_n(rst_n),
        .I_fg_valid(fg_valid), .I_fg_mask(fg_mask), .I_fg_rgb(fg_rgb),
        .I_fg_tuser(fg_tuser), .I_fg_tlast(fg_tlast),
        .I_det_mask(det_mask), .I_ccl_clk(ccl_clk),
        .O_evt(o_evt), .O_w(o_w), .O_h(o_h), .O_cnt(o_cnt), .O_chord(o_chord),
        .O_rows(o_rows), .O_minx(o_minx), .O_maxx(o_maxx), .O_miny(o_miny), .O_maxy(o_maxy),
        .O_pass(o_pass), .O_cand(o_cand),
        .O_ok_size(o_ok_size), .O_ok_asp(o_ok_asp), .O_ok_rows(o_ok_rows), .O_ok_chord(o_ok_chord),
        .O_lab(o_lab), .O_ovf(o_ovf),
        .O_dbg_full(dbg_full), .O_dbg_drop(dbg_drop)
    );

    // ---------------- 场景 ----------------
    function integer in_scene;
        input integer px, py;
        integer dx, dy;
        begin
            dx = px - 640;
            dy = py - 360;
            in_scene = (dx*dx + dy*dy <= 10000);
        end
    endfunction

    localparam integer LINE_CY = 2300;

    task write_frame;
        input integer vblank;
        integer yy, xx, t;
        begin
            repeat (vblank) @(negedge mipi_clk);
            for (yy = 0; yy < 720; yy = yy + 1) begin
                for (xx = 0; xx < 1280; xx = xx + 4) begin
                    @(negedge mipi_clk);
                    fg_valid <= 1'b1;
                    fg_mask  <= { in_scene(xx+3,yy), in_scene(xx+2,yy),
                                  in_scene(xx+1,yy), in_scene(xx,yy) };
                    fg_tuser <= (xx == 0 && yy == 0);
                    fg_tlast <= (xx == 1276);
                end
                @(negedge mipi_clk);
                fg_valid <= 1'b0;
                fg_mask  <= 4'd0;
                fg_tuser <= 1'b0;
                fg_tlast <= 1'b0;
                for (t = 320; t < LINE_CY; t = t + 1) @(negedge mipi_clk);
            end
        end
    endtask

    task prefill;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(negedge mipi_clk);
                fg_valid <= 1'b1;
                fg_mask  <= 4'h0;
                fg_tuser <= 1'b0;
                fg_tlast <= 1'b0;
            end
            @(negedge mipi_clk);
            fg_valid <= 1'b0;
        end
    endtask

    task chaos;
        integer n;
        begin
            for (n = 0; n < 230400; n = n + 1) begin
                @(negedge mipi_clk);
                fg_valid <= 1'b1;
                fg_mask  <= ((n % 5) == 0) ? 4'hF : 4'h0;
                fg_tuser <= (n == 0);
                fg_tlast <= 1'b0;
            end
            @(negedge mipi_clk);
            fg_valid <= 1'b0;
        end
    endtask

    // ---------------- 结果捕获与检查 ----------------
    integer errors = 0, evt_cnt = 0, bad_evt = 0, full_cycles = 0;
    integer g1_w, g1_h, g1_chord, g1_pass, g2_w, g2_chord;
    integer last_w, last_h, last_chord, last_pass;
    integer last2_w, last2_chord, last2_pass;

    always @(posedge mipi_clk) begin
        if (fg_valid && dbg_full) full_cycles <= full_cycles + 1;
    end

    always @(posedge mipi_clk) begin
        #1;
        if (o_evt) begin
            $display("EVT t=%0t n=%0d pass=%0d w=%0d h=%0d chord=%0d", $time, evt_cnt, o_pass, o_w, o_h, o_chord);
            if (o_pass !== 1'b1) bad_evt <= bad_evt + 1;
            if (evt_cnt == 0) begin g1_w <= o_w; g1_h <= o_h; g1_chord <= o_chord; g1_pass <= o_pass; end
            if (evt_cnt == 1) begin g2_w <= o_w; g2_chord <= o_chord; end
            last2_w <= last_w; last2_chord <= last_chord; last2_pass <= last_pass;
            last_w <= o_w; last_h <= o_h; last_chord <= o_chord; last_pass <= o_pass;
            evt_cnt <= evt_cnt + 1;
        end
    end

    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors = errors + 1; end
        end
    endtask

    initial begin
        repeat (20) @(negedge mipi_clk);
        rst_n = 1;
        repeat (10) @(negedge mipi_clk);

        // ---- 两帧干净圆 ----
        write_frame(0);
        write_frame(30*LINE_CY);
        repeat (100) @(negedge mipi_clk);
        check("frame1 detection (pass, 204x201)", g1_pass == 1 && g1_w >= 192 && g1_w <= 216 && g1_h >= 192 && g1_h <= 216);
        check("frame2 repeatability", g2_w == g1_w && g2_chord == g1_chord);
        check("event count = 2", evt_cnt == 2);
        check("FIFO not full in nominal frames", full_cycles == 0);

        // ---- 帧内强制溢出 ----
        $display("=== chaos ===");
        chaos();
        write_frame(30*LINE_CY);
        write_frame(30*LINE_CY);
        repeat (200) @(negedge mipi_clk);
        check("drop observed", dbg_drop == 1);
        check("chaos suppressed, two clean frames pass", evt_cnt == 4 && last_pass == 1 && last2_pass == 1);
        check("last two frames 204x201", last_w >= 192 && last_w <= 216 && last2_w >= 192 && last2_w <= 216);

        // ---- 满恰逢 tuser（丢 tuser）----
        $display("=== full at tuser ===");
        prefill(900);
        write_frame(0);
        repeat (500) @(negedge mipi_clk);
        write_frame(30*LINE_CY);
        write_frame(30*LINE_CY);
        repeat (200) @(negedge mipi_clk);
        check("lost-tuser segment suppressed, two recovery frames pass", evt_cnt == 6 && last_pass == 1 && last2_pass == 1);
        check("recovery frames 204x201", last_w >= 192 && last_w <= 216 && last2_w >= 192 && last2_w <= 216);
        check("all emitted events are valid detections", bad_evt == 0);

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end

endmodule
