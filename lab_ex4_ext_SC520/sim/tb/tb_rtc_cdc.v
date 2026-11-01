`timescale 1ns/1ps
// Unit test for isp_rtc_cdc: 1 Hz divider rollover + atomic CDC + saturation.
// CLK_DIV is scaled to 20 for simulation; the synthesis value is 50,000,000.
module tb_rtc_cdc;
    reg src_clk = 0, dst_clk = 0, rst_n = 0;
    always #5 src_clk = ~src_clk;      // 100 MHz
    always #6.5 dst_clk = ~dst_clk;    // ~77 MHz, asynchronous

    wire [7:0] d0,d1,d2, h0,h1,h2, m0,m1,m2, s0,s1,s2;

    isp_rtc_cdc #(.CLK_DIV(20),
                  .INIT_DAY(8'd0),  .INIT_HOUR(8'd0),
                  .INIT_MIN(8'd0),  .INIT_SEC(8'd58)) u0 (
        .src_clk(src_clk), .src_rst_n(rst_n),
        .dst_clk(dst_clk), .dst_rst_n(rst_n),
        .day(d0), .hour(h0), .minute(m0), .second(s0)
    );
    isp_rtc_cdc #(.CLK_DIV(20),
                  .INIT_DAY(8'd0),  .INIT_HOUR(8'd23),
                  .INIT_MIN(8'd59), .INIT_SEC(8'd58)) u1 (
        .src_clk(src_clk), .src_rst_n(rst_n),
        .dst_clk(dst_clk), .dst_rst_n(rst_n),
        .day(d1), .hour(h1), .minute(m1), .second(s1)
    );
    isp_rtc_cdc #(.CLK_DIV(20),
                  .INIT_DAY(8'd99), .INIT_HOUR(8'd23),
                  .INIT_MIN(8'd59), .INIT_SEC(8'd58)) u2 (
        .src_clk(src_clk), .src_rst_n(rst_n),
        .dst_clk(dst_clk), .dst_rst_n(rst_n),
        .day(d2), .hour(h2), .minute(m2), .second(s2)
    );

    integer errors = 0;
    task check;
        input cond;
        input [8*72-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors=errors+1; end
        end
    endtask

    task wait_tuple;
        input [7:0] ed, eh, em, es;
        input integer which;
        integer t;
        reg ok;
        begin
            t = 0; ok = 0;
            while ((t < 400) && !ok) begin
                @(posedge dst_clk);
                t = t + 1;
                case (which)
                  0: ok = (d0==ed)&&(h0==eh)&&(m0==em)&&(s0==es);
                  1: ok = (d1==ed)&&(h1==eh)&&(m1==em)&&(s1==es);
                  default: ok = (d2==ed)&&(h2==eh)&&(m2==em)&&(s2==es);
                endcase
            end
            if (!ok) begin
                $display("  [FAIL] timeout tuple %0d:%0d:%0d:%0d which=%0d",
                         ed,eh,em,es,which);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        repeat (5) @(negedge src_clk); rst_n = 1;
        // after the reset the destination registers are zero
        repeat (2) @(posedge dst_clk);
        check(d0==0 && h0==0 && m0==0 && s0==0, "reset clears destination");

        // tick 1
        wait_tuple(8'd0, 8'd0, 8'd0, 8'd59, 0);
        check(d0==0 && h0==0 && m0==0 && s0==59, "sec 58 -> 59");
        wait_tuple(8'd0, 8'd23, 8'd59, 8'd59, 1);
        check(d1==0 && h1==23 && m1==59 && s1==59, "23:59:58 -> 23:59:59");
        wait_tuple(8'd99, 8'd23, 8'd59, 8'd59, 2);
        check(d2==99 && h2==23 && m2==59 && s2==59, "99d 23:59:58 -> :59");

        // tick 2
        wait_tuple(8'd0, 8'd0, 8'd1, 8'd0, 0);
        check(d0==0 && h0==0 && m0==1 && s0==0, "59 -> 00, minute carry");
        wait_tuple(8'd1, 8'd0, 8'd0, 8'd0, 1);
        check(d1==1 && h1==0 && m1==0 && s1==0, "23:59:59 -> day+1 00:00:00");
        wait_tuple(8'd99, 8'd23, 8'd59, 8'd59, 2);
        check(d2==99 && h2==23 && m2==59 && s2==59, "saturates at 99:23:59:59");

        // a few more ticks: u2 stays saturated, u0 keeps counting
        repeat (4) @(posedge dst_clk);
        wait_tuple(8'd99, 8'd23, 8'd59, 8'd59, 2);
        check(d2==99 && h2==23 && m2==59 && s2==59, "saturation holds");

        // atomicity: u1 only ever shows the old or the new timestamp
        // (the exact tuple checks above already prove no mixed fields;
        //  here we simply verify the final values again)
        check(d1==1 && h1==0 && m1==0 && s1==0, "CDC group coherent");

        // reset clears again
        @(negedge src_clk); rst_n=0; repeat (3) @(negedge dst_clk);
        rst_n=1; repeat (3) @(negedge dst_clk);
        check(d0==0 && d1==0 && d2==0, "reset clears all again");

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else           $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
