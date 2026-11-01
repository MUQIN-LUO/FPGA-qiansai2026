`timescale 1ns/1ps
// Short frame-handover test: frame 2 arrives while frame 1 is processing.
// The core must drop frame 2 without corrupting frame 1's result.
module tb_handoff;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg ev_valid = 0, frame_start = 0, frame_end = 0;
    reg [11:0] ev_x = 0, ev_y = 0;
    reg signed [9:0] ev_ux = 0, ev_uy = 0;
    reg [3:0] ev_sector = 0;
    wire O_evt, O_valid;
    wire [11:0] O_cx, O_cy, O_r, O_minx, O_maxx, O_miny, O_maxy;
    wire [7:0] O_flags;
    wire [15:0] O_score;
    wire [11:0] O_count_dbg;
    isp_circle_hough2 u (
        .clk(clk), .rst_n(rst_n),
        .ev_valid(ev_valid), .ev_x(ev_x), .ev_y(ev_y),
        .ev_ux(ev_ux), .ev_uy(ev_uy), .ev_sector(ev_sector),
        .frame_start(frame_start), .frame_end(frame_end),
        .O_evt(O_evt), .O_valid(O_valid),
        .O_cx(O_cx), .O_cy(O_cy), .O_r(O_r),
        .O_minx(O_minx), .O_maxx(O_maxx), .O_miny(O_miny), .O_maxy(O_maxy),
        .O_flags(O_flags), .O_score(O_score), .O_count_dbg(O_count_dbg)
    );
    reg [47:0] mem [0:4095];
    integer n, i, t, evts = 0, errors = 0;
    reg done = 0;
    always @(posedge clk) begin
        if (O_evt) begin evts = evts + 1; done = 1; end
    end
    task send_event;
        input integer idx;
        begin
            @(negedge clk);
            ev_valid = 1;
            ev_x = mem[idx][47:36]; ev_y = mem[idx][35:24];
            ev_ux = $signed(mem[idx][23:14]); ev_uy = $signed(mem[idx][13:4]);
            ev_sector = mem[idx][3:0];
        end
    endtask
    initial begin
        $readmemh("model/ev_r60.hex", mem);
        n = mem[0][31:0];
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);
        // frame 1: full event list
        @(negedge clk); frame_start = 1;
        @(negedge clk); frame_start = 0;
        for (i = 1; i <= n; i = i + 1) send_event(i);
        @(negedge clk); ev_valid = 0; frame_end = 1;
        @(negedge clk); frame_end = 0;
        // frame 2 arrives immediately while frame 1 is processing
        @(negedge clk); frame_start = 1;
        @(negedge clk); frame_start = 0;
        for (i = 1; i <= 10; i = i + 1) send_event(i);
        @(negedge clk); ev_valid = 0; frame_end = 1;
        @(negedge clk); frame_end = 0;

        for (t = 0; t < 3000000 && !done; t = t + 1) @(posedge clk);
        repeat (20) @(negedge clk);
        $display("HANDOFF done=%0d evts=%0d valid=%0d cx=%0d cy=%0d r=%0d flags=%02h",
                 done, evts, O_valid, O_cx, O_cy, O_r, O_flags);
        if (!done) errors=errors+1;
        if (evts != 1) begin $display("  [FAIL] O_evt count %0d != 1", evts); errors=errors+1; end
        if (O_valid !== 1'b1) begin $display("  [FAIL] frame1 lost"); errors=errors+1; end
        if (O_r < 56 || O_r > 63) begin $display("  [FAIL] r %0d", O_r); errors=errors+1; end
        if (O_flags[3] !== 1'b1) begin $display("  [FAIL] drop flag not visible"); errors=errors+1; end
        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
