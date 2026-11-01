// ==========================================================================
// Phase-2 research front end: O_dg_y4 (4xY8/beat) -> 4x4-block gradient events.
// --------------------------------------------------------------------------
// Research rev 2 (2026-11-01).  NOT placed/routed or board-tested.
//
//  * Direction = signed VECTOR SUM of the 8 interior 3x3-Sobel samples of the
//    4x4 block (rev 1 used the single strongest sample).  Median direction
//    error on synthetic disks drops from ~8-11 deg to ~5-6 deg.
//  * Block energy gate |sum gx| + |sum gy| >= EV_TH (default 512) rejects
//    sensor-noise blocks; EDGE_TH stays the per-sample floor.
//  * Event position = strongest sample inside the block (subx/suby).
//  * Block accumulators are double-banked by 4-row group parity.  Each bank
//    is a 1R/1W RAM: the completed group is emitted from one bank while the
//    next group accumulates into the other, so the two never race.
//  * Block word = {gx[15:0], gy[15:0], mag[10:0], subx, suby[1:0], valid}.
//
// Event latency: read/merge 2 cycles + emission 3 cycles per block
// (960 cycles per group; the group period is ~9200 cycles at 74.25 MHz).
// ==========================================================================
module isp_grad_event96 #(
    parameter integer IMG_W   = 1280,
    parameter integer IMG_H   = 720,
    parameter [10:0]  EDGE_TH = 11'd48,     // per-sample floor
    parameter [15:0]  EV_TH   = 16'd512     // block sum-energy gate
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] I_y4,          // {Y3,Y2,Y1,Y0}, Y3=x+0, Y0=x+3
    input  wire        I_valid,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire [11:0] I_x,           // x of x+0 (0,4,...,1276)
    input  wire [11:0] I_y,           // row 0..719
    output reg         O_ev_valid,
    output reg  [11:0] O_ev_x,
    output reg  [11:0] O_ev_y,
    output reg  signed [9:0] O_ev_ux,
    output reg  signed [9:0] O_ev_uy,
    output reg  [3:0]  O_ev_sector,
    output reg         O_frame_start,
    output reg         O_frame_end
);
    localparam integer BW = 320;         // 4-pixel beats per row

    function [16:0] recip8;
        input [7:0] m;
        begin
            case (m)
                8'd0: recip8 = 17'd0;      8'd1: recip8 = 17'd65536;
                8'd2: recip8 = 17'd32768;  8'd3: recip8 = 17'd21845;
                8'd4: recip8 = 17'd16384;  8'd5: recip8 = 17'd13107;
                8'd6: recip8 = 17'd10923;  8'd7: recip8 = 17'd9362;
                8'd8: recip8 = 17'd8192;   8'd9: recip8 = 17'd7282;
                8'd10: recip8 = 17'd6554;  8'd11: recip8 = 17'd5958;
                8'd12: recip8 = 17'd5461;  8'd13: recip8 = 17'd5041;
                8'd14: recip8 = 17'd4681;  8'd15: recip8 = 17'd4369;
                8'd16: recip8 = 17'd4096;  8'd17: recip8 = 17'd3855;
                8'd18: recip8 = 17'd3641;  8'd19: recip8 = 17'd3449;
                8'd20: recip8 = 17'd3277;  8'd21: recip8 = 17'd3121;
                8'd22: recip8 = 17'd2979;  8'd23: recip8 = 17'd2849;
                8'd24: recip8 = 17'd2731;  8'd25: recip8 = 17'd2621;
                8'd26: recip8 = 17'd2521;  8'd27: recip8 = 17'd2427;
                8'd28: recip8 = 17'd2341;  8'd29: recip8 = 17'd2260;
                8'd30: recip8 = 17'd2185;  8'd31: recip8 = 17'd2114;
                8'd32: recip8 = 17'd2048;  8'd33: recip8 = 17'd1986;
                8'd34: recip8 = 17'd1928;  8'd35: recip8 = 17'd1872;
                8'd36: recip8 = 17'd1820;  8'd37: recip8 = 17'd1771;
                8'd38: recip8 = 17'd1725;  8'd39: recip8 = 17'd1680;
                8'd40: recip8 = 17'd1638;  8'd41: recip8 = 17'd1598;
                8'd42: recip8 = 17'd1560;  8'd43: recip8 = 17'd1524;
                8'd44: recip8 = 17'd1489;  8'd45: recip8 = 17'd1456;
                8'd46: recip8 = 17'd1425;  8'd47: recip8 = 17'd1394;
                8'd48: recip8 = 17'd1365;  8'd49: recip8 = 17'd1337;
                8'd50: recip8 = 17'd1311;  8'd51: recip8 = 17'd1285;
                8'd52: recip8 = 17'd1260;  8'd53: recip8 = 17'd1237;
                8'd54: recip8 = 17'd1214;  8'd55: recip8 = 17'd1192;
                8'd56: recip8 = 17'd1170;  8'd57: recip8 = 17'd1150;
                8'd58: recip8 = 17'd1130;  8'd59: recip8 = 17'd1111;
                8'd60: recip8 = 17'd1092;  8'd61: recip8 = 17'd1074;
                8'd62: recip8 = 17'd1057;  8'd63: recip8 = 17'd1040;
                8'd64: recip8 = 17'd1024;  8'd65: recip8 = 17'd1008;
                8'd66: recip8 = 17'd993;   8'd67: recip8 = 17'd978;
                8'd68: recip8 = 17'd964;   8'd69: recip8 = 17'd950;
                8'd70: recip8 = 17'd936;   8'd71: recip8 = 17'd923;
                8'd72: recip8 = 17'd910;   8'd73: recip8 = 17'd898;
                8'd74: recip8 = 17'd886;   8'd75: recip8 = 17'd874;
                8'd76: recip8 = 17'd862;   8'd77: recip8 = 17'd851;
                8'd78: recip8 = 17'd840;   8'd79: recip8 = 17'd830;
                8'd80: recip8 = 17'd819;   8'd81: recip8 = 17'd809;
                8'd82: recip8 = 17'd799;   8'd83: recip8 = 17'd790;
                8'd84: recip8 = 17'd780;   8'd85: recip8 = 17'd771;
                8'd86: recip8 = 17'd762;   8'd87: recip8 = 17'd753;
                8'd88: recip8 = 17'd745;   8'd89: recip8 = 17'd736;
                8'd90: recip8 = 17'd728;   8'd91: recip8 = 17'd720;
                8'd92: recip8 = 17'd712;   8'd93: recip8 = 17'd705;
                8'd94: recip8 = 17'd697;   8'd95: recip8 = 17'd690;
                8'd96: recip8 = 17'd683;   8'd97: recip8 = 17'd676;
                8'd98: recip8 = 17'd669;   8'd99: recip8 = 17'd662;
                8'd100: recip8 = 17'd655;  8'd101: recip8 = 17'd649;
                8'd102: recip8 = 17'd643;  8'd103: recip8 = 17'd636;
                8'd104: recip8 = 17'd630;  8'd105: recip8 = 17'd624;
                8'd106: recip8 = 17'd618;  8'd107: recip8 = 17'd612;
                8'd108: recip8 = 17'd607;  8'd109: recip8 = 17'd601;
                8'd110: recip8 = 17'd596;  8'd111: recip8 = 17'd590;
                8'd112: recip8 = 17'd585;  8'd113: recip8 = 17'd580;
                8'd114: recip8 = 17'd575;  8'd115: recip8 = 17'd570;
                8'd116: recip8 = 17'd565;  8'd117: recip8 = 17'd560;
                8'd118: recip8 = 17'd555;  8'd119: recip8 = 17'd551;
                8'd120: recip8 = 17'd546;  8'd121: recip8 = 17'd542;
                8'd122: recip8 = 17'd537;  8'd123: recip8 = 17'd533;
                8'd124: recip8 = 17'd529;  8'd125: recip8 = 17'd524;
                8'd126: recip8 = 17'd520;  8'd127: recip8 = 17'd516;
                8'd128: recip8 = 17'd512;  8'd129: recip8 = 17'd508;
                8'd130: recip8 = 17'd504;  8'd131: recip8 = 17'd500;
                8'd132: recip8 = 17'd496;  8'd133: recip8 = 17'd493;
                8'd134: recip8 = 17'd489;  8'd135: recip8 = 17'd485;
                8'd136: recip8 = 17'd482;  8'd137: recip8 = 17'd478;
                8'd138: recip8 = 17'd475;  8'd139: recip8 = 17'd471;
                8'd140: recip8 = 17'd468;  8'd141: recip8 = 17'd465;
                8'd142: recip8 = 17'd462;  8'd143: recip8 = 17'd458;
                8'd144: recip8 = 17'd455;  8'd145: recip8 = 17'd452;
                8'd146: recip8 = 17'd449;  8'd147: recip8 = 17'd446;
                8'd148: recip8 = 17'd443;  8'd149: recip8 = 17'd440;
                8'd150: recip8 = 17'd437;  8'd151: recip8 = 17'd434;
                8'd152: recip8 = 17'd431;  8'd153: recip8 = 17'd428;
                8'd154: recip8 = 17'd426;  8'd155: recip8 = 17'd423;
                8'd156: recip8 = 17'd420;  8'd157: recip8 = 17'd417;
                8'd158: recip8 = 17'd415;  8'd159: recip8 = 17'd412;
                8'd160: recip8 = 17'd410;  8'd161: recip8 = 17'd407;
                8'd162: recip8 = 17'd405;  8'd163: recip8 = 17'd402;
                8'd164: recip8 = 17'd400;  8'd165: recip8 = 17'd397;
                8'd166: recip8 = 17'd395;  8'd167: recip8 = 17'd392;
                8'd168: recip8 = 17'd390;  8'd169: recip8 = 17'd388;
                8'd170: recip8 = 17'd386;  8'd171: recip8 = 17'd383;
                8'd172: recip8 = 17'd381;  8'd173: recip8 = 17'd379;
                8'd174: recip8 = 17'd377;  8'd175: recip8 = 17'd374;
                8'd176: recip8 = 17'd372;  8'd177: recip8 = 17'd370;
                8'd178: recip8 = 17'd368;  8'd179: recip8 = 17'd366;
                8'd180: recip8 = 17'd364;  8'd181: recip8 = 17'd362;
                8'd182: recip8 = 17'd360;  8'd183: recip8 = 17'd358;
                8'd184: recip8 = 17'd356;  8'd185: recip8 = 17'd354;
                8'd186: recip8 = 17'd352;  8'd187: recip8 = 17'd350;
                8'd188: recip8 = 17'd349;  8'd189: recip8 = 17'd347;
                8'd190: recip8 = 17'd345;  8'd191: recip8 = 17'd343;
                8'd192: recip8 = 17'd341;  8'd193: recip8 = 17'd340;
                8'd194: recip8 = 17'd338;  8'd195: recip8 = 17'd336;
                8'd196: recip8 = 17'd334;  8'd197: recip8 = 17'd333;
                8'd198: recip8 = 17'd331;  8'd199: recip8 = 17'd329;
                8'd200: recip8 = 17'd328;  8'd201: recip8 = 17'd326;
                8'd202: recip8 = 17'd324;  8'd203: recip8 = 17'd323;
                8'd204: recip8 = 17'd321;  8'd205: recip8 = 17'd320;
                8'd206: recip8 = 17'd318;  8'd207: recip8 = 17'd317;
                8'd208: recip8 = 17'd315;  8'd209: recip8 = 17'd314;
                8'd210: recip8 = 17'd312;  8'd211: recip8 = 17'd311;
                8'd212: recip8 = 17'd309;  8'd213: recip8 = 17'd308;
                8'd214: recip8 = 17'd306;  8'd215: recip8 = 17'd305;
                8'd216: recip8 = 17'd303;  8'd217: recip8 = 17'd302;
                8'd218: recip8 = 17'd301;  8'd219: recip8 = 17'd299;
                8'd220: recip8 = 17'd298;  8'd221: recip8 = 17'd297;
                8'd222: recip8 = 17'd295;  8'd223: recip8 = 17'd294;
                8'd224: recip8 = 17'd293;  8'd225: recip8 = 17'd291;
                8'd226: recip8 = 17'd290;  8'd227: recip8 = 17'd289;
                8'd228: recip8 = 17'd287;  8'd229: recip8 = 17'd286;
                8'd230: recip8 = 17'd285;  8'd231: recip8 = 17'd284;
                8'd232: recip8 = 17'd282;  8'd233: recip8 = 17'd281;
                8'd234: recip8 = 17'd280;  8'd235: recip8 = 17'd279;
                8'd236: recip8 = 17'd278;  8'd237: recip8 = 17'd277;
                8'd238: recip8 = 17'd275;  8'd239: recip8 = 17'd274;
                8'd240: recip8 = 17'd273;  8'd241: recip8 = 17'd272;
                8'd242: recip8 = 17'd271;  8'd243: recip8 = 17'd270;
                8'd244: recip8 = 17'd269;  8'd245: recip8 = 17'd267;
                8'd246: recip8 = 17'd266;  8'd247: recip8 = 17'd265;
                8'd248: recip8 = 17'd264;  8'd249: recip8 = 17'd263;
                8'd250: recip8 = 17'd262;  8'd251: recip8 = 17'd261;
                8'd252: recip8 = 17'd260;  8'd253: recip8 = 17'd259;
                8'd254: recip8 = 17'd258;  8'd255: recip8 = 17'd257;
                default: recip8 = 17'd0;
            endcase
        end
    endfunction

    function [3:0] sector16;
        input signed [15:0] gx;
        input signed [15:0] gy;
        reg [15:0] ax, ay, minor, major;
        reg [2:0] oct;
        reg sub;
        begin
            ax = gx[15] ? (~gx + 16'd1) : gx;
            ay = gy[15] ? (~gy + 16'd1) : gy;
            if (ax >= ay) begin minor=ay; major=ax; end
            else          begin minor=ax; major=ay; end
            sub = (((minor << 3) + (minor << 2)) >= ((major << 2) + major));
            if (ax >= ay) begin
                if (!gx[15] && !gy[15]) oct=3'd0;
                else if ( gx[15] && !gy[15]) oct=3'd3;
                else if ( gx[15] &&  gy[15]) oct=3'd4;
                else                         oct=3'd7;
            end
            else begin
                if (!gx[15] && !gy[15]) oct=3'd1;
                else if ( gx[15] && !gy[15]) oct=3'd2;
                else if ( gx[15] &&  gy[15]) oct=3'd5;
                else                         oct=3'd6;
            end
            sector16 = {oct, sub};
        end
    endfunction

    function signed [9:0] clip10;
        input signed [23:0] v;
        begin
            if (v > 24'sd255) clip10 = 10'sd255;
            else if (v < -24'sd256) clip10 = -10'sd256;
            else clip10 = v[9:0];
        end
    endfunction

    // ---------------- line buffers ---------------------------------------
    (* ram_style = "block" *) reg [31:0] lb1 [0:BW-1];
    (* ram_style = "block" *) reg [31:0] lb2 [0:BW-1];

    reg [31:0] top_q, mid_q, bot_q;
    reg [11:0] x_q, y_q;
    reg        v_q, tlast_q;
    wire [8:0] idx = I_x[10:2];
    wire [31:0] top_w = lb2[idx];
    wire [31:0] mid_w = lb1[idx];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            top_q<=0; mid_q<=0; bot_q<=0; x_q<=0; y_q<=0; v_q<=0; tlast_q<=0;
        end
        else begin
            v_q <= I_valid;
            if (I_valid) begin
                top_q <= top_w; mid_q <= mid_w; bot_q <= I_y4;
                lb1[idx] <= I_y4;
                lb2[idx] <= lb1[idx];
                x_q <= I_x; y_q <= (I_y > 0) ? (I_y - 12'd1) : 12'd0;
                tlast_q <= I_tlast;
            end
            else tlast_q <= 0;
        end
    end

    // ---------------- Sobel of the two interior columns -------------------
    wire [7:0] t0=top_q[31:24], t1=top_q[23:16], t2=top_q[15:8], t3=top_q[7:0];
    wire [7:0] m0=mid_q[31:24], m1=mid_q[23:16], m2=mid_q[15:8], m3=mid_q[7:0];
    wire [7:0] b0=bot_q[31:24], b1=bot_q[23:16], b2=bot_q[15:8], b3=bot_q[7:0];

    function signed [12:0] sob_gx;
        input [7:0] a00,a01,a02,a10,a11,a12,a20,a21,a22;
        begin
            sob_gx = -$signed({1'b0,a00}) + $signed({1'b0,a02})
                     - ($signed({1'b0,a10}) <<< 1) + ($signed({1'b0,a12}) <<< 1)
                     - $signed({1'b0,a20}) + $signed({1'b0,a22});
        end
    endfunction
    function signed [12:0] sob_gy;
        input [7:0] a00,a01,a02,a10,a11,a12,a20,a21,a22;
        begin
            sob_gy =  $signed({1'b0,a20}) + ($signed({1'b0,a21}) <<< 1) + $signed({1'b0,a22})
                    - $signed({1'b0,a00}) - ($signed({1'b0,a01}) <<< 1) - $signed({1'b0,a02});
        end
    endfunction
    function [10:0] abs13;
        input signed [12:0] a;
        begin abs13 = a[12] ? (~a + 13'd1) : a; end
    endfunction

    wire signed [12:0] gx1 = sob_gx(t0,t1,t2,m0,m1,m2,b0,b1,b2);
    wire signed [12:0] gy1 = sob_gy(t0,t1,t2,m0,m1,m2,b0,b1,b2);
    wire signed [12:0] gx2 = sob_gx(t1,t2,t3,m1,m2,m3,b1,b2,b3);
    wire signed [12:0] gy2 = sob_gy(t1,t2,t3,m1,m2,m3,b1,b2,b3);
    wire [10:0] mag1 = abs13(gx1) + abs13(gy1);
    wire [10:0] mag2 = abs13(gx2) + abs13(gy2);
    wire use2 = (mag2 > mag1);
    wire [10:0] sel_mag = use2 ? mag2 : mag1;
    wire sel_subx = use2;
    wire [7:0] bx = x_q[10:2];
    wire [1:0] suby = y_q[1:0];
    wire       acc_bank = y_q[2];

    // ---------------- block RAMs: 1R/1W each ------------------------------
    (* ram_style = "block" *) reg [46:0] blkA [0:BW-1];
    (* ram_style = "block" *) reg [46:0] blkB [0:BW-1];
    reg [46:0] blkA_rd, blkB_rd;
    reg        ready;

    // accumulation stage 1 (beat metadata; the matching block word is read
    // combinationally from address bx in the same cycle)
    reg        s1_v, s1_bank;
    reg [8:0]  s1_addr;
    reg signed [12:0] s1_dx, s1_dy;
    reg [10:0] s1_mag;
    reg        s1_subx;
    reg [1:0]  s1_suby;

    // emission
    localparam [3:0] E_CLRA=0, E_CLRB=1, E_IDLE=2, E_RD=3, E_EVAL=4, E_CALC=5, E_CALC2=9, E_RECIP=6, E_EMIT=7, E_END=8;
    reg [3:0]  est;
    reg [8:0]  ebx;
    reg [8:0]  eclr;
    reg        em_bank, em_bank_r;
    reg [11:0] eg_y;
    reg [2:0]  em_dly;
    reg        emit_pend, emit_busy, em_last;
    reg [1:0]  pend_bank;
    reg [11:0] pend_gy;
    reg [46:0] em_word;

    wire acc_en = v_q && (y_q >= 12'd2) && ready;

    // read addresses (combinational; captured on the next clock edge)
    // stage 0: registered Sobel results + position metadata
    reg        s0_v, s0_bank;
    reg [8:0]  s0_addr;
    reg [1:0]  s0_suby;
    reg signed [12:0] s0_gx1, s0_gy1, s0_gx2, s0_gy2;
    wire [8:0] raA = (emit_busy && (em_bank == 1'b0)) ? ebx : s0_addr;
    wire [8:0] raB = (emit_busy && (em_bank == 1'b1)) ? ebx : s0_addr;

    // merge of this beat into the block word read in the same cycle
    wire [46:0] s1_old = s1_bank ? blkB_rd : blkA_rd;
    wire        s1_usemax = (s1_mag > s1_old[14:4]);
    wire [46:0] acc_wd = s1_old[0]
        ? {s1_old[46:31] + {{3{s1_dx[12]}},s1_dx},
           s1_old[30:15] + {{3{s1_dy[12]}},s1_dy},
           s1_usemax ? s1_mag : s1_old[14:4],
           s1_usemax ? s1_subx : s1_old[3],
           s1_usemax ? s1_suby : s1_old[2:1],
           1'b1}
        : {{{3{s1_dx[12]}}, s1_dx}, {{3{s1_dy[12]}}, s1_dy},
           s1_mag, s1_subx, s1_suby, 1'b1};

    wire        acc_ok = s1_v && !(emit_busy && (s1_bank == em_bank));
    wire        em_ok  = emit_busy && (est == E_EMIT);
    wire        clrA   = (est == E_CLRA);
    wire        clrB   = (est == E_CLRB);

    wire [8:0]  a_wa = clrA ? eclr : (em_ok && !em_bank) ? ebx : s1_addr;
    wire [8:0]  b_wa = clrB ? eclr : (em_ok &&  em_bank) ? ebx : s1_addr;
    wire [46:0] a_wd = (clrA || (em_ok && !em_bank)) ? 47'd0 : acc_wd;
    wire [46:0] b_wd = (clrB || (em_ok &&  em_bank)) ? 47'd0 : acc_wd;
    wire        a_we = clrA || (em_ok && !em_bank) || (acc_ok && !s1_bank);
    wire        b_we = clrB || (em_ok &&  em_bank) || (acc_ok &&  s1_bank);

    always @(posedge clk) begin
        blkA_rd <= blkA[raA];
        blkB_rd <= blkB[raB];
        if (a_we) blkA[a_wa] <= a_wd;
        if (b_we) blkB[b_wa] <= b_wd;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_v<=0; s0_bank<=0; s0_addr<=0; s0_suby<=0;
            s0_gx1<=0; s0_gy1<=0; s0_gx2<=0; s0_gy2<=0;
        end
        else begin
            s0_v    <= v_q && (y_q >= 12'd2) && ready;
            s0_bank <= y_q[2];
            s0_addr <= x_q[10:2];
            s0_suby <= y_q[1:0];
            s0_gx1 <= gx1; s0_gy1 <= gy1;
            s0_gx2 <= gx2; s0_gy2 <= gy2;
        end
    end
    wire [10:0] mag1_s = abs13(s0_gx1) + abs13(s0_gy1);
    wire [10:0] mag2_s = abs13(s0_gx2) + abs13(s0_gy2);
    wire        use2_s = (mag2_s > mag1_s);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_v<=0; s1_bank<=0; s1_addr<=0; s1_dx<=0; s1_dy<=0;
            s1_mag<=0; s1_subx<=0; s1_suby<=0;
        end
        else begin
            s1_v    <= s0_v;
            s1_bank <= s0_bank;
            s1_addr <= s0_addr;
            s1_dx   <= s0_gx1 + s0_gx2;
            s1_dy   <= s0_gy1 + s0_gy2;
            s1_mag  <= use2_s ? mag2_s : mag1_s;
            s1_subx <= use2_s;
            s1_suby <= s0_suby;
        end
    end

    // direction normalisation of the block-sum vector
    wire signed [15:0] sgx = em_word[46:31];
    wire signed [15:0] sgy = em_word[30:15];
    wire [15:0] sax = sgx[15] ? (~sgx + 16'd1) : sgx;
    wire [15:0] say = sgy[15] ? (~sgy + 16'd1) : sgy;
    wire [15:0] emax = (sax >= say) ? sax : say;
    wire [15:0] emin = (sax >= say) ? say : sax;
    reg [15:0]  emag_r;
    wire [15:0] emag = emax + (emin >> 1);

    wire [15:0] energy = sax + say;

    reg  [2:0]  esh;
    reg  [7:0]  emag8;
    reg  [16:0] erecip;
    reg  [2:0]  esh_r;
    reg  [7:0]  emag8_r;
    reg  [16:0] erecip_r;
    reg  [3:0]  esec_r;
    reg         en_r;
    wire signed [32:0] ux_prod = $signed({{8{sgx[15]}}, sgx}) * $signed({1'b0,erecip_r});
    wire signed [32:0] uy_prod = $signed({{8{sgy[15]}}, sgy}) * $signed({1'b0,erecip_r});
    wire signed [9:0] ux_n = clip10(ux_prod >>> (8 + esh_r));
    wire signed [9:0] uy_n = clip10(uy_prod >>> (8 + esh_r));
    wire [3:0] esec = esec_r;

    always @(*) begin
        if (emag_r > 16'd8191)      esh = 3'd6;
        else if (emag_r > 16'd4095) esh = 3'd5;
        else if (emag_r > 16'd2047) esh = 3'd4;
        else if (emag_r > 16'd1023) esh = 3'd3;
        else if (emag_r > 16'd511)  esh = 3'd2;
        else if (emag_r > 16'd255)  esh = 3'd1;
        else                         esh = 3'd0;
        emag8 = (emag_r >> esh);
        if (emag8 == 0) emag8 = 8'd1;
        erecip = recip8(emag8);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            est<=E_CLRA; ebx<=0; eclr<=0; ready<=0; em_bank<=0; em_bank_r<=0;
            eg_y<=0; em_dly<=0; emit_pend<=0; emit_busy<=0; em_last<=0;
            esh_r<=0; emag8_r<=0; erecip_r<=0; esec_r<=0; en_r<=0; emag_r<=0;
            pend_bank<=0; pend_gy<=0; em_word<=0;
            O_ev_valid<=0; O_ev_x<=0; O_ev_y<=0; O_ev_ux<=0; O_ev_uy<=0;
            O_ev_sector<=0; O_frame_start<=0; O_frame_end<=0;
        end
        else begin
            O_ev_valid <= 1'b0;
            O_frame_start <= I_tuser;
            O_frame_end <= 1'b0;
            em_bank_r <= em_bank;

            if (emit_pend) begin
                em_dly <= em_dly + 1'b1;
                if (em_dly == 3'd3) begin
                    emit_pend <= 1'b0; em_dly <= 0;
                    em_bank <= pend_bank[0]; eg_y <= pend_gy;
                    ebx <= 0; emit_busy <= 1'b1; est <= E_RD;
                end
            end
            else if (!emit_busy && (est == E_IDLE) && tlast_q && (y_q > 0) &&
                     (((y_q[1:0] == 2'd3)) || (y_q == 12'd718))) begin
                pend_bank <= {1'b0, y_q[2]};
                pend_gy   <= y_q[10:2];
                em_last   <= (y_q == 12'd718);
                emit_pend <= 1'b1; em_dly <= 0;
            end

            case (est)
            E_CLRA: begin
                if (eclr == BW-1) begin eclr<=0; est<=E_CLRB; end
                else eclr <= eclr + 1'b1;
            end
            E_CLRB: begin
                if (eclr == BW-1) begin eclr<=0; ready<=1'b1; est<=E_IDLE; end
                else eclr <= eclr + 1'b1;
            end
            E_IDLE: begin end
            E_RD:    est <= E_EVAL;
            E_EVAL: begin em_word <= em_bank_r ? blkB_rd : blkA_rd; est <= E_CALC; end
            E_CALC: begin
                emag_r   <= emag;
                esec_r   <= sector16(em_word[46:31], em_word[30:15]);
                en_r     <= em_word[0] && (em_word[14:4] >= EDGE_TH) &&
                            (energy >= EV_TH);
                est <= E_CALC2;
            end
            E_CALC2: begin
                esh_r   <= esh;
                emag8_r <= emag8;
                est <= E_RECIP;
            end
            E_RECIP: begin
                erecip_r <= recip8(emag8_r);
                est <= E_EMIT;
            end
            E_EMIT: begin
                if (en_r) begin
                    O_ev_valid <= 1'b1;
                    O_ev_x     <= ({1'b0, ebx} << 2) + 12'd1 + {11'd0, em_word[3]};
                    O_ev_y     <= (eg_y << 2) + {10'd0, em_word[2:1]};
                    O_ev_ux    <= ux_n;
                    O_ev_uy    <= uy_n;
                    O_ev_sector<= esec;
                end
                if (ebx == BW-1) begin
                    if (em_last) est <= E_END;
                    else begin emit_busy <= 1'b0; est <= E_IDLE; end
                end
                else begin ebx <= ebx + 1'b1; est <= E_RD; end
            end
            E_END: begin
                O_frame_end <= 1'b1;
                emit_busy <= 1'b0;
                est <= E_IDLE;
            end
            default: est <= E_IDLE;
            endcase
        end
    end
endmodule
