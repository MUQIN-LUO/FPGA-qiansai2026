`timescale 1ns/1ps
// Continuous-frame pixel-level stability test (board-feedback fix).
// Drives 5 real-timing 1280x720 frames through isp_grad_event96 -> core:
//   f1 circle r60 (processed)    f2 circle (arrives while f1 is processing
//   -> must be DROPPED with no O_evt and no flicker)
//   f3..f5 background only (processed empty frames age the bounded hold)
// Expected O_evt sequence: f1 valid, f3 held, f4 held, f5 cleared.
// Also samples O_valid continuously and fails if it drops before the clear.
module tb_multiframe_e2e;
    localparam integer W = 1280, H = 720;
    localparam integer LINE_CYCLES = 2300;
    localparam integer ACTIVE = W/4;
    localparam integer HBLANK = LINE_CYCLES - ACTIVE - 1;
    localparam integer VBLANK = 20;
    localparam integer FRAMES = 5;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [31:0] tdata = 0;
    reg [11:0] ix = 0, iy = 0;

    wire        ev_valid, fstart, fend;
    wire [11:0] ev_x, ev_y;
    wire signed [9:0] ev_ux, ev_uy;
    wire [3:0]  ev_sector;

    isp_grad_event96 u_fe (
        .clk(clk), .rst_n(rst_n),
        .I_y4(tdata), .I_valid(tvalid), .I_tuser(tuser), .I_tlast(tlast),
        .I_x(ix), .I_y(iy),
        .O_ev_valid(ev_valid), .O_ev_x(ev_x), .O_ev_y(ev_y),
        .O_ev_ux(ev_ux), .O_ev_uy(ev_uy), .O_ev_sector(ev_sector),
        .O_frame_start(fstart), .O_frame_end(fend)
    );

    wire O_evt, O_valid;
    wire [11:0] O_cx, O_cy, O_r, O_minx, O_maxx, O_miny, O_maxy;
    wire [7:0]  O_flags;
    wire [15:0] O_score;
    wire [11:0] O_count_dbg;

    isp_circle_hough2 u_h (
        .clk(clk), .rst_n(rst_n),
        .ev_valid(ev_valid), .ev_x(ev_x), .ev_y(ev_y),
        .ev_ux(ev_ux), .ev_uy(ev_uy), .ev_sector(ev_sector),
        .frame_start(fstart), .frame_end(fend),
        .O_evt(O_evt), .O_valid(O_valid),
        .O_cx(O_cx), .O_cy(O_cy), .O_r(O_r),
        .O_minx(O_minx), .O_maxx(O_maxx), .O_miny(O_miny), .O_maxy(O_maxy),
        .O_flags(O_flags), .O_score(O_score), .O_count_dbg(O_count_dbg)
    );

    integer CX, CY, RR, INS, OUT;
    integer cur_frame;            // frame being driven (0-based)
    integer evtcnt = 0, errors = 0;
    integer first_evt_seen = 0;
    integer glitch = 0;           // O_valid dropped before the final clear
    reg     expect_clear = 0;

    always @(posedge clk) if (O_evt) begin
        evtcnt = evtcnt + 1;
        first_evt_seen = 1;
        $display("EVT#%0d frame=%0d valid=%b r=%0d flags=%02h", evtcnt, cur_frame, O_valid, O_r, O_flags);
        case (evtcnt)
          1: if (O_valid !== 1'b1) begin $display("  [FAIL] f1 evt invalid"); errors=errors+1; end
          2: if (O_valid !== 1'b1) begin $display("  [FAIL] f3 held invalid"); errors=errors+1; end
          3: if (O_valid !== 1'b1) begin $display("  [FAIL] f4 held invalid"); errors=errors+1; end
          4: if (O_valid !== 1'b0) begin $display("  [FAIL] f5 not cleared"); errors=errors+1; end
          default: ;
        endcase
        if (evtcnt == 4) expect_clear = 1;
    end
    // continuous valid monitor: after the first evt and before the expected
    // clear, O_valid must never drop
    always @(posedge clk) if (first_evt_seen && !expect_clear && O_valid === 1'b0)
        glitch = glitch + 1;

    function [7:0] pixel;
        input integer px, py;
        integer dx, dy;
        begin
            dx = px - CX; dy = py - CY;
            if (cur_frame < 2)
                pixel = ((dx*dx + dy*dy) <= (RR*RR)) ? INS[7:0] : OUT[7:0];
            else
                pixel = OUT[7:0];          // target left
        end
    endfunction

    task drive_line;
        input integer yy;
        input integer first;
        integer xx;
        begin
            for (xx = 0; xx < W; xx = xx + 4) begin
                @(negedge clk);
                tvalid = 1;
                tdata  = {pixel(xx,yy), pixel(xx+1,yy), pixel(xx+2,yy), pixel(xx+3,yy)};
                tuser  = (first && xx == 0);
                tlast  = (xx == 1276);
                ix = xx[11:0]; iy = yy[11:0];
            end
            @(negedge clk);
            tvalid = 0; tuser = 0; tlast = 0;
            repeat (HBLANK) @(negedge clk);
        end
    endtask

    initial begin
        CX = 640; CY = 360; RR = 60; INS = 200; OUT = 40;
        repeat (20) @(negedge clk);
        rst_n = 1;
        repeat (10) @(negedge clk);
        for (cur_frame = 0; cur_frame < FRAMES; cur_frame = cur_frame + 1) begin
            for (iy = 0; iy < H; iy = iy + 1) drive_line(iy, (iy == 0));
            repeat (VBLANK * LINE_CYCLES) @(negedge clk);
            $display("frame %0d done: valid=%b r=%0d evts=%0d", cur_frame, O_valid, O_r, evtcnt);
        end
        // allow the last processing to finish (it may start at the last frame end)
        repeat (2000000) @(posedge clk);
        expect_clear = 1;
        $display("FINAL evts=%0d valid=%b r=%0d glitch_samples=%0d", evtcnt, O_valid, O_r, glitch);
        if (evtcnt < 4) begin $display("  [FAIL] expected >=4 processed frames, got %0d", evtcnt); errors=errors+1; end
        if (glitch != 0) begin $display("  [FAIL] O_valid dropped %0d times before the clear", glitch); errors=errors+1; end
        if (O_valid !== 1'b0) begin $display("  [FAIL] box not cleared after target left"); errors=errors+1; end
        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
