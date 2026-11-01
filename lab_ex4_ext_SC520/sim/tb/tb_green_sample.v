`timescale 1ns/1ps
// Unit test for isp_green_sample: 16-beat interior grid + relative-green
// classifier + 2-sample hysteresis.
module tb_green_sample;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        beat_valid = 0;
    reg [11:0] beat_x = 0, beat_y = 0;
    reg [9:0]  beat_r = 0, beat_g = 0, beat_b = 0;
    reg [11:0] trk_cx = 600, trk_cy = 360, trk_r = 60;
    reg        trk_ok = 1;
    reg        id_chg = 0;
    wire       green;

    isp_green_sample #(.IMG_W(1280), .IMG_H(720)) u_dut (
        .clk(clk), .rst_n(rst_n),
        .beat_valid(beat_valid), .beat_x(beat_x), .beat_y(beat_y),
        .beat_r(beat_r), .beat_g(beat_g), .beat_b(beat_b),
        .trk_cx(trk_cx), .trk_cy(trk_cy), .trk_r(trk_r),
        .trk_ok(trk_ok), .id_chg(id_chg), .green(green)
    );

    integer xs [0:3];
    integer ys [0:3];
    integer errors = 0;

    task check;
        input cond;
        input [8*72-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    task beat;
        input integer x, y, r, g, b;
        begin
            @(negedge clk);
            beat_valid=1; beat_x=x; beat_y=y;
            beat_r=r; beat_g=g; beat_b=b;
            @(negedge clk); beat_valid=0;
        end
    endtask

    task pulse_id;
        begin
            @(negedge clk); id_chg=1'b1;
            @(negedge clk); id_chg=1'b0;
        end
    endtask

    task frame;
        input integer r, g, b;   // average pixel value; beat sum = 4x
        integer i, j;
        begin
            beat(0,0, r*4,g*4,b*4);
            for (i=0;i<4;i=i+1)
                for (j=0;j<4;j=j+1)
                    beat(xs[i], ys[j], r*4,g*4,b*4);
            beat(1276,719, r*4,g*4,b*4);
            repeat (2) @(posedge clk);
        end
    endtask

    initial begin
        // grid targets for trk (600,360,r60): offsets +/-30, +/-15, x->4px
        xs[0]=568; xs[1]=584; xs[2]=612; xs[3]=628;
        ys[0]=330; ys[1]=345; ys[2]=375; ys[3]=390;
        repeat (5) @(negedge clk); rst_n=1; repeat (3) @(negedge clk);
        check(green==0, "reset: not green");

        frame(12,23,17);                 // HUD sample 1
        check(green==0, "1 green sample: not yet (hysteresis)");
        frame(22,45,35);                 // HUD sample 2
        check(green==1, "2 green samples: GREEN");

        frame(10,10,10);                 // neutral grey
        check(green==1, "1 grey: GREEN retained (hysteresis)");
        frame(10,10,10);
        check(green==0, "2 grey: GREEN cleared");

        frame(10,13,10);                 // slight green-grey
        check(green==0, "grey with slight G: not GREEN");

        frame(15,30,22);                 // HUD sample 3
        frame(15,30,22);
        check(green==1, "HUD sample (15,30,22): GREEN");

        frame(10,20,25);                 // cyan-ish
        frame(10,20,25);
        check(green==0, "cyan (10,20,25): not GREEN");

        // ---- identity change clears old GREEN --------------------------
        frame(22,45,35); frame(22,45,35);
        check(green==1, "green before identity change");
        pulse_id;
        check(green==0, "identity change clears GREEN");
        frame(10,10,10);
        check(green==0, "red/grey after identity change: no stale GREEN");

        // ---- identity change in the middle of a sample -----------------
        frame(22,45,35); frame(22,45,35);
        check(green==1, "green before mid-sample switch");
        // fstart + 8 green hits, switch, 8 more hits, fend
        beat(0,0, 88,180,140);
        begin : mid_switch
            integer i, j, k;
            begin
                k = 0;
                for (i=0;i<4;i=i+1)
                    for (j=0;j<4;j=j+1) begin
                        if (k == 8) pulse_id;
                        beat(xs[i], ys[j], 88,180,140);
                        k = k + 1;
                    end
            end
        end
        beat(1276,719, 88,180,140);
        repeat (2) @(posedge clk);
        check(green==0, "mid-sample identity change aborts old sample");

        // ---- edge sampling invalid: no stale GREEN --------------------
        frame(22,45,35); frame(22,45,35);
        check(green==1, "green before edge invalid");
        trk_cx = 10; trk_cy = 360; trk_r = 60;
        frame(22,45,35);
        check(green==0, "edge grid invalid clears GREEN");
        trk_cx = 600; trk_cy = 360; trk_r = 60;

        frame(5,10,7);                   // dark green
        frame(5,10,7);
        check(green==1, "dark green (5,10,7): GREEN (relative)");

        frame(1,3,1);                    // near black
        frame(1,3,1);
        check(green==0, "near-black (1,3,1): not GREEN");

        // no track -> grid invalid; green must clear at the next frame start
        frame(22,45,35); frame(22,45,35);
        check(green==1, "green before track loss");
        trk_ok = 0;
        frame(22,45,35);
        check(green==0, "track lost: GREEN cleared");
        trk_ok = 1;

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else           $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
