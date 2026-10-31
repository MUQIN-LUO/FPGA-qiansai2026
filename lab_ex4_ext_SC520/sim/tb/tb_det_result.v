// ==========================================================================
// isp_det_result 定向回归：通过 -> 候选 -> 空帧 / 切换目标
// --------------------------------------------------------------------------
// 复现真实板测问题：
//   旧实现 O_det_valid（跨帧迟滞）与 S_box（每帧 M_size_ok 更新）共用一组
//   bbox。本帧出现"尺寸通过、形状未过"的候选时，bbox 被候选位置覆盖，
//   绿色 valid 仍来自上一帧的通过状态 -> 绿色框套在候选/杂物上。
//
// 本测试直接用真实 isp_det_result（isp_top 内同一份 RTL）：
//   1) 连续 3 帧 pass@A -> valid=1，box=A；
//   2) 1 帧 candidate@B（size_ok=1, pass=0）-> valid 仍 1，box 必须仍是 A，
//      cand=1（旧实现会变成 B）；
//   3) 2 帧空 -> valid 熄灭；候选框不亮绿；
//   4) 切换目标 pass@C -> box=C；
//   5) 复位/清空 -> valid=0/box=0。
// ==========================================================================
`timescale 1ns/1ps
module tb_det_result;
    reg clk = 0, rst_n = 0, clr = 0;
    reg evt = 0, pass = 0, size_ok = 0, cand = 0, frm = 0;
    reg [5:0] crit = 0;
    reg [9:0] cnt_in = 0;
    reg [11:0] bx0 = 0, bx1 = 0, by0 = 0, by1 = 0;
    wire valid, cand_o, pass_o, size_o, cand_d, evt_o;
    wire [11:0] ox0, ox1, oy0, oy1;
    wire [5:0] conf;
    wire [7:0] flags;
    wire [9:0] cnt_o;

    always #5 clk = ~clk;   // 100 MHz

    isp_det_result u_dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_clr(clr),
        .I_evt(evt), .I_frame(frm), .I_pass(pass), .I_size_ok(size_ok), .I_cand(cand),
        .I_crit(crit), .I_cnt(cnt_in),
        .I_minx(bx0), .I_maxx(bx1), .I_miny(by0), .I_maxy(by1),
        .O_valid(valid), .O_cand(cand_o),
        .O_minx(ox0), .O_maxx(ox1), .O_miny(oy0), .O_maxy(oy1),
        .O_conf(conf), .O_flags(flags), .O_cnt(cnt_o),
        .O_pass(pass_o), .O_size_ok(size_o),
        .O_cand_d(cand_d), .O_evt(evt_o)
    );

    integer errors = 0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    task frame;
        input        p, so, c;
        input [11:0] x0, x1, y0, y1;
        begin
            @(negedge clk);
            evt = 1; pass = p; size_ok = so; cand = c;
            bx0 = x0; bx1 = x1; by0 = y0; by1 = y1;
            @(negedge clk);
            evt = 0;
            repeat (5) @(negedge clk);
        end
    endtask

    // 帧起始但没有评估事件：坏帧/丢拍场景
    task frame_tick;
        begin
            @(negedge clk);
            frm = 1;
            @(negedge clk);
            frm = 0;
            repeat (4) @(negedge clk);
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        check("reset: valid=0", valid == 1'b0);
        check("reset: box=0", ox0 == 12'd0 && ox1 == 12'd0);

        // ---- 1. 三帧 pass@A ----
        crit = 6'b011111;   // {ovf=0,live=1,chord=1,rows=1,asp=1,size=1}
        cnt_in = 10'd123;
        frame(1,1,0, 100,199,100,199);
        frame(1,1,0, 100,199,100,199);
        frame(1,1,0, 100,199,100,199);
        $display("PASS A: valid=%b box=(%0d,%0d)-(%0d,%0d) conf=%0d flags=%b cnt=%0d",
                 valid, ox0, oy0, ox1, oy1, conf, flags, cnt_o);
        check("3 pass -> valid=1", valid == 1'b1);
        check("3 pass -> box=A", ox0==100 && ox1==199 && oy0==100 && oy1==199);
        check("conf >= 24", conf >= 24);
        check("flags atomic with pass (ovf/live/crit/pass/valid)",
              flags == 8'b01111111);
        check("cnt atomic with pass", cnt_o == 10'd123);

        // ---- 2. 候选@B：不能覆盖绿色框 ----
        crit = 6'b010111;   // live=1, chord=0, rows/asp/size=1
        cnt_in = 10'd45;
        frame(0,1,1, 300,399,300,399);
        $display("CAND B: valid=%b box=(%0d,%0d)-(%0d,%0d) cand=%b conf=%0d flags=%b cnt=%0d",
                 valid, ox0, oy0, ox1, oy1, cand_o, conf, flags, cnt_o);
        check("candidate: valid still 1", valid == 1'b1);
        check("candidate: green box stays A (not B)",
              ox0==100 && ox1==199 && oy0==100 && oy1==199);
        check("candidate: cand flag=1", cand_o == 1'b1);
        check("candidate: size_ok flag=1", size_o == 1'b1);
        check("candidate flags atomic (chord=0, pass=0, valid=1)",
              flags == 8'b01011101);
        check("candidate cnt atomic", cnt_o == 10'd45);

        // ---- 3. 两帧空：迟滞熄灭 ----
        crit = 6'b000000; cnt_in = 10'd0;
        frame(0,0,0, 0,0,0,0);
        frame(0,0,0, 0,0,0,0);
        $display("EMPTY: valid=%b box=(%0d,%0d)-(%0d,%0d) conf=%0d flags=%b",
                 valid, ox0, oy0, ox1, oy1, conf, flags);
        check("2 empty -> valid=0", valid == 1'b0);
        check("2 empty -> cand=0", cand_o == 1'b0);
        check("2 empty -> flags=0", flags == 8'b00000000);

        // ---- 4. 切换目标 pass@C ----
        frame(1,1,0, 500,599,400,499);
        frame(1,1,0, 500,599,400,499);
        frame(1,1,0, 500,599,400,499);
        $display("PASS C: valid=%b box=(%0d,%0d)-(%0d,%0d)", valid, ox0, oy0, ox1, oy1);
        check("switch target -> valid=1", valid == 1'b1);
        check("switch target -> box=C", ox0==500 && ox1==599 && oy0==400 && oy1==499);

        // ---- 4b. 坏帧丢事件：旧绿框不能无限保留 ----
        frame_tick();   // 本帧有事件，不应衰减
        check("frame tick after event: no decay", valid == 1'b1 && conf >= 24);
        frame_tick(); frame_tick(); frame_tick();
        $display("MISSING EVENTS: valid=%b conf=%0d flags=%b", valid, conf, flags);
        check("3 missing-event frames -> stale green box cleared", valid == 1'b0);
        check("missing-event flags clear pass/live", flags[6:0] == 7'd0);

        // ---- 5. 复位清零 ----
        clr = 1;
        @(negedge clk); clr = 0;
        repeat (3) @(negedge clk);
        check("clr -> valid=0", valid == 1'b0);
        check("clr -> box=0", ox0==0 && ox1==0 && oy0==0 && oy1==0);

        // ---- 6. 只有候选（无历史通过）：valid=0，候选框=B ----
        frame(0,1,1, 700,799,600,699);
        $display("CAND-only: valid=%b box=(%0d,%0d)-(%0d,%0d) cand=%b",
                 valid, ox0, oy0, ox1, oy1, cand_o);
        check("cand-only: valid=0", valid == 1'b0);
        check("cand-only: box=B", ox0==700 && ox1==799 && oy0==600 && oy1==699);
        check("cand-only: cand=1", cand_o == 1'b1);

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
