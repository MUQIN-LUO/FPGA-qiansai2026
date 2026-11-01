// Simple dual-port RAM wrapper: one write port, one registered read port.
// Isolating the array in its own module is what lets TD infer ERAM for the
// large memories (the same pattern in a module with several arrays fell back
// to distributed LUT RAM).
module isp_dpram #(
    parameter integer DW = 8,
    parameter integer AW = 10,
    parameter integer DEPTH = 1024
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] wa,
    input  wire [DW-1:0] wd,
    input  wire [AW-1:0] ra,
    output reg  [DW-1:0] rd
);
    (* ram_style = "block" *) reg [DW-1:0] m [0:DEPTH-1];
    always @(posedge clk) begin
        if (we) m[wa] <= wd;
        rd <= m[ra];
    end
endmodule

// ==========================================================================
// Phase-2 research RTL: sparse-event coarse Hough -> median radius init ->
// radial Gauss-Newton fine centre -> 1px radial histogram + circularity.
// --------------------------------------------------------------------------
// Research rev 2 (2026-11-01).  NOT placed/routed or board-tested.
//
// Why a new fine stage: rev 1 projected every event along its own gradient
// normal.  On real raster input 13-20% of the normals are off by >14 deg
// (block-quantised staircase); at r=160 a 5 deg error displaces the projected
// line by 14 px.  The rev-1 fine centre therefore missed r100/r160.  This
// build uses only event POSITIONS for the fine stage:
//
//   init pass  : 1px radius histogram at the coarse (4px) centre, then the
//                MEDIAN radius r0 (the mean is pulled by background events).
//   4 x refine : Gauss-Newton on sum(|p-c|-r0)^2, scalar step
//                  d = 2 * sum((rho-r0)*d) / sum(rho^2) * r0
//                (the full 2x2 solve was verified in the model too, but it
//                 needs a ~50-bit determinant divider and was worse under
//                 noise; the scalar step passed 28/28 clean and 27-28/28 at
//                 sigma=6..10 in the pixel-level model).
//                Direction is used only as a loose 29-39 deg inlier gate.
//   final pass : 1px radial histogram + 16-sector map at the refined centre.
//   gates      : cov>=10/16, all four quadrants, peak>=12,
//                peak*4 >= 201*r/128 (fill>=0.25), aligned>=1/8 of events,
//                <=6 sector transitions in the +-3px band (an ellipse's band
//                crosses 4 disjoint arcs and fails this).
//
// Storage: no full-resolution edge map, no 3D accumulator.  ev_ram 1024x48,
// acc 320x180x8, hist/sect 145x27.  Every array has ONE write port and one
// registered read port and is written from ONE always block.
//
// Multipliers: one shared 24x24 (r^2, dot, two GN products).  Divider: one
// 48-bit restoring divider shared by the centre step, mean radius, fill.
// ==========================================================================
module isp_circle_hough2 #(
    parameter integer IMG_W  = 1280,
    parameter integer IMG_H  = 720,
    parameter integer EVMAX  = 1024,
    parameter integer ACC_W  = 320,
    parameter integer ACC_H  = 180,
    parameter integer R_MIN  = 26,
    parameter integer R_MAX  = 160,
    parameter integer TOPK   = 4,
    parameter integer ACCW   = 12,
    parameter integer HOLD   = 2,
    parameter integer ITERS  = 16,
    parameter integer SCORE_MARGIN = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ev_valid,
    input  wire [11:0] ev_x,
    input  wire [11:0] ev_y,
    input  wire signed [9:0] ev_ux,
    input  wire signed [9:0] ev_uy,
    input  wire [3:0]  ev_sector,
    input  wire        frame_start,
    input  wire        frame_end,
    output reg         O_evt,
    output reg         O_valid,
    output reg  [11:0] O_cx, O_cy, O_r,
    output reg  [11:0] O_minx, O_maxx, O_miny, O_maxy,
    output reg  [7:0]  O_flags,
    output reg  [15:0] O_score,
    output reg  [11:0] O_count_dbg
);
    localparam integer ACC_N = ACC_W * ACC_H;
    localparam integer B0    = 20;
    localparam integer RB    = 145;
    localparam integer EV_AW = 11;
    localparam [1:0] P_INIT = 2'd0, P_REF = 2'd1, P_FIN = 2'd2;

    // ---------------- FSM state / counters ------------------------------
    localparam [5:0]
      S_IDLE=0, S_CLR=1, S_VEV=2, S_VDEC=3, S_VRD=4, S_VWR=5,
      S_TOP=6, S_TOPRD=7, S_TOPEV=8,
      S_NEWC=9, S_HCLR=10,
      S_PFETCH=11, S_PD1=12, S_PD2=13, S_PD3=14, S_PD4=15, S_PD5=16,
      S_PSQ=17, S_PGATE=18, S_PG1=19, S_PG2=20, S_PNEXT=21,
      S_MED=22, S_MEDW=23, S_MEDC=24,
      S_RSOLVE=25, S_RWAITQ=26, S_RLOADY=27, S_RWAITY=28,
      S_RMULX=29, S_RMULY=30, S_RWAITM=31, S_RCAND=32,
      S_PEAK=33, S_PEAKW=34, S_PEAKC=35,
      S_COVB=36, S_COVW=37, S_COVC=38, S_FILL=39, S_CDONE=40,
      S_SCORE=41, S_DONE=42, S_GAP=43, S_SEL=44, S_PG0=45, S_FIN0=46;

    reg [5:0] st;
    reg [16:0] a;
    reg [11:0] ev_i;
    reg [5:0]  k;
    reg [2:0]  fan;
    reg        pol;
    reg [2:0]  cand;
    reg [3:0]  iter;
    reg [1:0]  pass;
    reg [8:0]  sx, sy, sx_d, sy_d;
    reg [11:0] ex, ey;
    reg signed [9:0] ux, uy;
    reg [3:0]  esec;
    reg signed [19:0] axp, ayp, axm, aym;
    reg [7:0]  scan_bin, med_bin, h_idx;
    reg [3:0]  ls_i;
    reg signed [12:0] ls_bx, ls_by;
    reg [15:0] ls_score;
    reg [11:0] ls_r, ls_cx, ls_cy;
    reg [4:0]  ls_cov;
    reg        ls_ok;
    reg [7:0]  h_ra, h_wa;
    reg [26:0] h_wd;
    reg        h_we;
    reg [7:0]  r0, rho;
    reg        gate_ok;
    reg [23:0] dot_abs_r;

    // ---------------- capture --------------------------------------------
    reg        ev_we;
    reg [EV_AW-1:0] ev_wa;
    reg [47:0] ev_wd;
    wire [47:0] ev_rd;
    wire [EV_AW-1:0] ev_ra = ev_i[EV_AW-1:0];

    reg [EV_AW-1:0] wr_ptr;
    reg [12:0] wr_cnt;
    reg        cap_active, cap_ovf, cap_drop;
    reg        proc_busy;
    reg [12:0] proc_count;
    reg        proc_ovf, proc_drop, proc_start, proc_done;
    reg        drop_seen;
    reg [1:0]  hold_cnt;
    reg [11:0] h_cx, h_cy, h_r, h_minx, h_maxx, h_miny, h_maxy;
    reg [15:0] h_score;

    // ---------------- coarse accumulator ---------------------------------
    wire [ACCW-1:0] acc_rd;

    // fan directions: +-12 deg and +-24 deg around the measured normal so
    // that a noisy normal still votes through the true centre (the coarse
    // vote is only a seed for the position-only radial refine).
    function signed [15:0] rot16;
        input signed [9:0] ux;
        input signed [9:0] uy;
        input [2:0] f;
        reg signed [17:0] sx, sy, a;
        begin
            sx = {{8{ux[9]}}, ux};
            sy = {{8{uy[9]}}, uy};
            case (f)
            3'd0: a = sx << 8;
            3'd1: a = (sx<<8) - (sx<<2) - (sx<<1)
                    - ((sy<<5) + (sy<<4) + (sy<<2) + sy);
            3'd2: a = (sx<<8) - (sx<<2) - (sx<<1)
                    + ((sy<<5) + (sy<<4) + (sy<<2) + sy);
            3'd3: a = (sx<<8) - (sx<<4) - (sx<<2) - (sx<<1)
                    - ((sy<<6) + (sy<<5) + (sy<<3));
            default: a = (sx<<8) - (sx<<4) - (sx<<2) - (sx<<1)
                    + ((sy<<6) + (sy<<5) + (sy<<3));
            endcase
            rot16 = {{6{a[17]}}, a[17:8]};
        end
    endfunction
    function signed [15:0] rot16b;
        input signed [9:0] ux;
        input signed [9:0] uy;
        input [2:0] f;
        reg signed [17:0] sx, sy, b;
        begin
            sx = {{8{ux[9]}}, ux};
            sy = {{8{uy[9]}}, uy};
            case (f)
            3'd0: b = sy << 8;
            3'd1: b = (sy<<8) - (sy<<2) - (sy<<1)
                    + ((sx<<5) + (sx<<4) + (sx<<2) + sx);
            3'd2: b = (sy<<8) - (sy<<2) - (sy<<1)
                    - ((sx<<5) + (sx<<4) + (sx<<2) + sx);
            3'd3: b = (sy<<8) - (sy<<4) - (sy<<2) - (sy<<1)
                    + ((sx<<6) + (sx<<5) + (sx<<3));
            default: b = (sy<<8) - (sy<<4) - (sy<<2) - (sy<<1)
                    - ((sx<<6) + (sx<<5) + (sx<<3));
            endcase
            rot16b = {{6{b[17]}}, b[17:8]};
        end
    endfunction
    reg signed [15:0] fan_ux, fan_uy;

    wire signed [9:0] vcx_p = axp[19:10];
    wire signed [9:0] vcy_p = ayp[19:10];
    wire signed [9:0] vcx_m = axm[19:10];
    wire signed [9:0] vcy_m = aym[19:10];
    wire [16:0] vaddr_p = ({7'd0, ayp[19:10]} << 8) + ({7'd0, ayp[19:10]} << 6) + {7'd0, axp[19:10]};
    wire [16:0] vaddr_m = ({7'd0, aym[19:10]} << 8) + ({7'd0, aym[19:10]} << 6) + {7'd0, axm[19:10]};
    wire vok_p = (vcx_p >= 0) && (vcy_p >= 0) && (vcx_p < ACC_W) && (vcy_p < ACC_H);
    wire vok_m = (vcx_m >= 0) && (vcy_m >= 0) && (vcx_m < ACC_W) && (vcy_m < ACC_H);
    wire [16:0] top_addr = ({8'd0, sy_d} << 8) + ({8'd0, sy_d} << 6) + {8'd0, sx_d};

    wire [16:0] acc_ra_w = (st == S_VRD)   ? (pol ? vaddr_m : vaddr_p) :
                           (st == S_TOPRD) ? top_addr : 17'd0;
    wire        acc_we_w = (st == S_CLR) ? 1'b1 :
                           (st == S_VWR) ? (pol ? vok_m : vok_p) :
                           (st == S_TOPEV) ? 1'b1 : 1'b0;
    wire [16:0] acc_wa_w = (st == S_CLR) ? a :
                           (st == S_VWR) ? (pol ? vaddr_m : vaddr_p) : top_addr;
    wire [ACCW-1:0] acc_wd_w = (st == S_CLR) ? {ACCW{1'b0}} :
                           (st == S_VWR) ? ((acc_rd == {ACCW{1'b1}}) ? {ACCW{1'b1}} : (acc_rd + 1'b1)) :
                           {ACCW{1'b0}};

    // ---------------- radial histogram / sector map ----------------------
    wire [26:0] h_rd;

    // ---------------- top-K / candidates ---------------------------------
    reg [ACCW-1:0] top_cnt [0:TOPK-1];
    reg [8:0]  top_x   [0:TOPK-1];
    reg [8:0]  top_y   [0:TOPK-1];
    reg [11:0] c_r [0:TOPK-1], c_cx [0:TOPK-1], c_cy [0:TOPK-1];
    reg [4:0]  c_cov [0:TOPK-1];
    reg [15:0] c_score [0:TOPK-1];
    reg        c_ok [0:TOPK-1];

    // ---------------- fine state -----------------------------------------
    reg signed [12:0] cx, cy;
    reg [15:0] n_ev, n_align;
    reg [24:0] trace;
    reg signed [25:0] gx_acc, gy_acc;
    reg [17:0] sr;

    reg signed [23:0] dxs, dys;
    reg [23:0] r2a, r2;
    reg signed [23:0] dotx, dot;
    reg signed [23:0] gxt, gyt, eval_;
    reg [7:0]  sqrt_root;
    reg [15:0] sqrt_rem;
    reg [2:0]  sqrt_i;

    reg signed [23:0] mul_a, mul_b;
    wire signed [47:0] mul_p = mul_a * mul_b;

    wire [15:0] sqrt_rem_sh = (sqrt_rem << 2) | ((r2[15:0] >> {sqrt_i,1'b0}) & 3);
    wire [15:0] sqrt_trial  = ({8'd0,sqrt_root} << 2) | 16'd1;
    wire        sqrt_ge     = (sqrt_rem_sh >= sqrt_trial);
    wire [15:0] sqrt_rem_n  = sqrt_ge ? (sqrt_rem_sh - sqrt_trial) : sqrt_rem_sh;
    wire [7:0]  sqrt_root_n = sqrt_ge ? (({7'd0,sqrt_root} << 1) | 1'b1) : (sqrt_root << 1);

    wire [15:0] gate_th = ({8'd0,rho} << 7) + ({8'd0,rho} << 6) + ({8'd0,rho} << 3);
    wire        dir_ok  = (dot_abs_r >= gate_th);
    wire [15:0] band_mul = ({8'd0,r0} << 6) + ({8'd0,r0} << 4) + ({8'd0,r0} << 2) + {8'd0,r0};
    wire [7:0]  bandv = (band_mul[15:8] < 8'd8) ? 8'd8 : band_mul[15:8];
    wire signed [8:0] rdiff = $signed({1'b0,rho}) - $signed({1'b0,r0});
    wire        band_ok = !((rdiff > $signed({1'b0,bandv})) ||
                            (rdiff < -$signed({1'b0,bandv})));
    wire        win_init = (rho >= 8'd22) && (rho <= 8'd200);
    wire        win_fin  = (rho >= R_MIN[7:0]) && (rho <= R_MAX[7:0]);
    wire        win_ok = (pass == P_INIT) ? win_init :
                         (pass == P_REF ) ? (band_ok && win_init) : win_fin;

    // ---------------- divider --------------------------------------------
    reg        dv_start, dv_busy, dv_done;
    reg [47:0] dv_num, dv_den, dv_quo, dv_rem, dv_q;
    reg [5:0]  dv_i;
    wire [48:0] dv_sh = {dv_rem[47:0], dv_num[dv_i]};
    wire        dv_ge = (dv_sh >= {1'b0, dv_den});
    wire [48:0] dv_sub = dv_sh - {1'b0, dv_den};
    wire [47:0] dv_quo_n = {dv_quo[46:0], dv_ge};

    // ---------------- scans / results ------------------------------------
    reg [12:0] cum;
    reg        med_found;
    reg [10:0] peak_v, peak_bin;
    reg [10:0] h_p2, h_p1;
    reg [7:0]  sv_i;
    reg [15:0] band16;
    reg [15:0] band_r, rot;
    reg [4:0]  maxr;
    reg [7:0]  gap_i;

    reg [4:0]  cov;
    reg [3:0]  ntr;
    reg        quad;
    reg [15:0] fill_q8;
    wire [15:0] band_final = band16 | h_rd[26:11];
    wire [7:0]  peak_r = peak_bin[7:0] + B0[7:0];
    wire [15:0] rexp = ({8'd0,peak_r} << 7) + ({8'd0,peak_r} << 6)
                     + ({8'd0,peak_r} << 3) + {8'd0,peak_r};
    wire [15:0] rexp_q = rexp >> 7;
    wire        ok_band = (cov >= 10) && (peak_bin >= 11'd6) && (maxr <= 5'd5) && (peak_v >= 12) &&
                          (({1'b0,peak_v} << 1) >= rexp_q) &&
                          (({2'b0,n_align} << 3) >= proc_count) && (ntr <= 4'd6);

    reg [15:0] best_score;
    reg [11:0] best_r, best_cx, best_cy;
    reg        best_ok;
    integer jj;

    reg signed [15:0] qx, qy;
    reg        sx_sign, sy_sign;
    reg signed [23:0] step_x, step_y;
    wire signed [23:0] step_raw = ($signed(mul_p[23:0]) >>> 11) >>> 1;
    wire [25:0] absgx = gx_acc[25] ? (~gx_acc + 26'd1) : gx_acc;
    wire [25:0] absgy = gy_acc[25] ? (~gy_acc + 26'd1) : gy_acc;

    wire signed [3:0] ls_ox = (ls_i == 4'd1) ? 4'sd1 : (ls_i == 4'd2) ? -4'sd1 :
                              (ls_i == 4'd5) ? 4'sd2 : (ls_i == 4'd6) ? -4'sd2 : 4'sd0;
    wire signed [3:0] ls_oy = (ls_i == 4'd3) ? 4'sd1 : (ls_i == 4'd4) ? -4'sd1 :
                              (ls_i == 4'd7) ? 4'sd2 : (ls_i == 4'd8) ? -4'sd2 : 4'sd0;
    reg        ls_small;
    reg [3:0]  ls_last;
    wire hist_wr = (st == S_PG2) && ((pass == P_INIT) || (pass == P_FIN));
    wire h_clr   = (st == S_HCLR);
    wire [7:0]  h_wa_w = h_clr ? scan_bin : h_idx;
    wire [26:0] h_wd_w = h_clr ? 27'd0 :
                         (pass == P_FIN)
                           ? {h_rd[26:11] | (16'd1 << esec), h_rd[10:0] + 1'b1}
                           : {h_rd[26:11], h_rd[10:0] + 1'b1};


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


    // trailing-zero count: with a rotating window this gives the length of
    // the zero run starting at each sector.  Used over 16 cycles so the
    // coverage gate never sits on a 22-level combinational path.
    function [4:0] tz16;
        input [15:0] v;
        begin
            if (v[0])       tz16 = 5'd0;
            else if (v[1])  tz16 = 5'd1;
            else if (v[2])  tz16 = 5'd2;
            else if (v[3])  tz16 = 5'd3;
            else if (v[4])  tz16 = 5'd4;
            else if (v[5])  tz16 = 5'd5;
            else if (v[6])  tz16 = 5'd6;
            else if (v[7])  tz16 = 5'd7;
            else if (v[8])  tz16 = 5'd8;
            else if (v[9])  tz16 = 5'd9;
            else if (v[10]) tz16 = 5'd10;
            else if (v[11]) tz16 = 5'd11;
            else if (v[12]) tz16 = 5'd12;
            else if (v[13]) tz16 = 5'd13;
            else if (v[14]) tz16 = 5'd14;
            else if (v[15]) tz16 = 5'd15;
            else            tz16 = 5'd16;
        end
    endfunction

    function [4:0] pop16;
        input [15:0] v;
        integer q;
        begin
            pop16 = 0;
            for (q = 0; q < 16; q = q + 1) pop16 = pop16 + v[q];
        end
    endfunction

    function [3:0] trans16;
        input [15:0] v;
        integer q;
        begin
            trans16 = 0;
            for (q = 0; q < 16; q = q + 1)
                if (v[q] != v[(q == 15) ? 0 : q+1]) trans16 = trans16 + 1'b1;
        end
    endfunction

    // =================== storage (single always) =========================
    isp_dpram #(.DW(ACCW), .AW(16), .DEPTH(ACC_N)) u_acc (
        .clk(clk), .we(acc_we_w), .wa(acc_wa_w[15:0]), .wd(acc_wd_w),
        .ra(acc_ra_w[15:0]), .rd(acc_rd));
    isp_dpram #(.DW(27), .AW(8), .DEPTH(RB)) u_hmem (
        .clk(clk), .we(h_we), .wa(h_wa), .wd(h_wd),
        .ra(h_ra), .rd(h_rd));
    isp_dpram #(.DW(48), .AW(10), .DEPTH(EVMAX)) u_evram (
        .clk(clk), .we(ev_we), .wa(ev_wa[9:0]), .wd(ev_wd),
        .ra(ev_ra[9:0]), .rd(ev_rd));

    // =================== divider =========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dv_busy<=0; dv_done<=0; dv_i<=0; dv_quo<=0; dv_rem<=0; dv_q<=0;
        end
        else begin
            dv_done <= 1'b0;
            if (dv_start && !dv_busy) begin
                dv_busy<=1; dv_quo<=0; dv_rem<=0; dv_i<=47;
            end
            else if (dv_busy) begin
                dv_rem <= dv_ge ? dv_sub[47:0] : dv_sh[47:0];
                dv_quo <= dv_quo_n;
                if (dv_i == 6'd0) begin
                    dv_busy<=0; dv_done<=1; dv_q<=dv_quo_n;
                end
                else dv_i <= dv_i - 1'b1;
            end
        end
    end

    // =================== capture =========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr<=0; wr_cnt<=0; cap_active<=0; cap_ovf<=0; cap_drop<=0;
            proc_busy<=0; proc_count<=0; proc_ovf<=0; proc_drop<=0;
            proc_start<=0; drop_seen<=0;
            ev_we<=0; ev_wa<=0; ev_wd<=0;
        end
        else begin
            proc_start <= 1'b0;
            ev_we <= 1'b0;
            if (frame_start) begin
                wr_ptr     <= 0;
                wr_cnt     <= 0;
                cap_ovf    <= 1'b0;
                cap_drop   <= proc_busy;
                cap_active <= !proc_busy;
            end
            if (ev_valid && cap_active) begin
                if (wr_ptr < EVMAX) begin
                    ev_we <= 1'b1;
                    ev_wa <= wr_ptr;
                    ev_wd <= {ev_x, ev_y, ev_ux, ev_uy, ev_sector};
                    wr_ptr <= wr_ptr + 1'b1;
                    wr_cnt <= wr_cnt + 1'b1;
                end
                else cap_ovf <= 1'b1;
            end
            if (frame_end) begin
                cap_active  <= 1'b0;
                O_count_dbg <= wr_cnt[11:0];
            end
            if (frame_end && proc_busy) drop_seen <= 1'b1;
            else if (proc_done)         drop_seen <= 1'b0;
            if (proc_done) proc_busy <= 1'b0;
            else if (frame_end && !proc_busy && !cap_drop) begin
                proc_count  <= wr_cnt;
                proc_ovf    <= cap_ovf;
                proc_drop   <= cap_drop;
                proc_start  <= 1'b1;
                proc_busy   <= 1'b1;
            end
        end
    end

    // =================== main FSM ========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=S_IDLE; O_evt<=0; O_valid<=0; O_flags<=0; O_score<=0; proc_done<=0;
            O_cx<=0; O_cy<=0; O_r<=0;
            O_minx<=0; O_maxx<=0; O_miny<=0; O_maxy<=0;
            a<=0; ev_i<=0; k<=1; pol<=0; cand<=0; iter<=0; pass<=P_INIT;
            sx<=0; sy<=0; sx_d<=0; sy_d<=0; ex<=0; ey<=0; ux<=0; uy<=0; esec<=0;
            axp<=0; ayp<=0; axm<=0; aym<=0; fan_ux<=0; fan_uy<=0;
            cx<=0; cy<=0; r0<=0; rho<=0; gate_ok<=0; dot_abs_r<=0; h_idx<=0; n_ev<=0; n_align<=0;
            trace<=0; gx_acc<=0; gy_acc<=0; sr<=0;
            dxs<=0; dys<=0; r2a<=0; r2<=0; dotx<=0; dot<=0;
            gxt<=0; gyt<=0; eval_<=0; sqrt_root<=0; sqrt_rem<=0; sqrt_i<=0;
            mul_a<=0; mul_b<=0;
            scan_bin<=0; med_bin<=0; cum<=0; med_found<=0;
            peak_v<=0; peak_bin<=11'd6; h_p2<=0; h_p1<=0;
            sv_i<=0; band16<=0; cov<=0; ntr<=0; quad<=0; fill_q8<=0;
            band_r<=0; rot<=0; maxr<=0; gap_i<=0;
            ls_i<=0; ls_bx<=0; ls_by<=0; ls_score<=0; ls_ok<=0; ls_small<=0; ls_last<=0;
            ls_r<=0; ls_cx<=0; ls_cy<=0; ls_cov<=0;
            dv_start<=0; dv_num<=0; dv_den<=0; qx<=0; qy<=0;
            sx_sign<=0; sy_sign<=0; step_x<=0; step_y<=0;
            best_score<=0; best_r<=0; best_cx<=0; best_cy<=0; best_ok<=0;
            hold_cnt<=0; h_cx<=0; h_cy<=0; h_r<=0; h_minx<=0; h_maxx<=0;
            h_miny<=0; h_maxy<=0; h_score<=0;
            h_ra<=0; h_wa<=0; h_wd<=0; h_we<=0;
            for (jj = 0; jj < TOPK; jj = jj + 1) begin
                top_cnt[jj]<=0; top_x[jj]<=0; top_y[jj]<=0;
                c_r[jj]<=0; c_cx[jj]<=0; c_cy[jj]<=0;
                c_cov[jj]<=0; c_score[jj]<=0; c_ok[jj]<=0;
            end
        end
        else begin
            O_evt <= 1'b0;
            proc_done <= 1'b0;
            dv_start <= 1'b0;
            h_we <= hist_wr | h_clr;
            h_wa <= h_wa_w;
            h_wd <= h_wd_w;

            case (st)
            S_IDLE: begin
                if (proc_start) begin
                    // clear the previous frame's selection BEFORE the
                    // short-circuit path: otherwise an empty/overflow frame
                    // would reuse stale best_ok/best_* and refresh the hold
                    best_ok <= 1'b0; best_r <= 12'd0; best_cx <= 12'd0;
                    best_cy <= 12'd0; best_score <= 16'd0;
                    if (proc_drop || proc_ovf || (proc_count == 0)) st <= S_DONE;
                    else begin a <= 0; st <= S_CLR; end
                end
            end

            S_CLR: begin
                if (a == ACC_N-1) begin
                    a <= 0;
                    top_cnt[0]<=0; top_x[0]<=0; top_y[0]<=0;
                    top_cnt[1]<=0; top_x[1]<=0; top_y[1]<=0;
                    top_cnt[2]<=0; top_x[2]<=0; top_y[2]<=0;
                    top_cnt[3]<=0; top_x[3]<=0; top_y[3]<=0;
                    ev_i <= 0; st <= S_VEV;
                end
                else a <= a + 1'b1;
            end

            S_VEV: begin
                if (ev_i >= proc_count) begin sx <= 0; sy <= 0; st <= S_TOP; end
                else st <= S_VDEC;
            end

            S_VDEC: begin
                ex <= ev_rd[47:36]; ey <= ev_rd[35:24];
                ux <= $signed(ev_rd[23:14]); uy <= $signed(ev_rd[13:4]);
                esec <= ev_rd[3:0];
                k <= 6'd1; fan <= 3'd0; pol <= 1'b0;
                fan_ux <= {{6{ev_rd[23]}}, ev_rd[23:14]};
                fan_uy <= {{6{ev_rd[13]}}, ev_rd[13:4]};
                axp <= {8'd0, ev_rd[47:36]} <<< 8;
                ayp <= {8'd0, ev_rd[35:24]} <<< 8;
                axm <= {8'd0, ev_rd[47:36]} <<< 8;
                aym <= {8'd0, ev_rd[35:24]} <<< 8;
                st <= S_VRD;
            end

            S_VRD: st <= S_VWR;

            S_VWR: begin
                if (pol == 1'b0) begin pol <= 1'b1; st <= S_VRD; end
                else begin
                    pol <= 1'b0;
                    axp <= axp + (fan_ux <<< 2);
                    ayp <= ayp + (fan_uy <<< 2);
                    axm <= axm - (fan_ux <<< 2);
                    aym <= aym - (fan_uy <<< 2);
                    if (k == 6'd40) begin
                        k <= 6'd1;
                        if (fan == 3'd0) begin fan <= 3'd0; ev_i <= ev_i + 1'b1; st <= S_VEV; end
                        else begin
                            fan <= fan + 1'b1;
                            fan_ux <= rot16(ux, uy, fan + 1'b1);
                            fan_uy <= rot16b(ux, uy, fan + 1'b1);
                            // restart the ray at the event for the new fan
                            axp <= {8'd0, ex} <<< 8;
                            ayp <= {8'd0, ey} <<< 8;
                            axm <= {8'd0, ex} <<< 8;
                            aym <= {8'd0, ey} <<< 8;
                            st <= S_VRD;
                        end
                    end
                    else begin k <= k + 1'b1; st <= S_VRD; end
                end
            end

            S_TOP: begin sx_d <= sx; sy_d <= sy; st <= S_TOPRD; end
            S_TOPRD: st <= S_TOPEV;
            S_TOPEV: begin
                if ((acc_rd > top_cnt[TOPK-1]) &&
                    ((top_cnt[0] == 0) || (sx_d >= top_x[0] + 9'd2) || (top_x[0] >= sx_d + 9'd2) ||
                                         (sy_d >= top_y[0] + 9'd2) || (top_y[0] >= sy_d + 9'd2)) &&
                    ((top_cnt[1] == 0) || (sx_d >= top_x[1] + 9'd2) || (top_x[1] >= sx_d + 9'd2) ||
                                         (sy_d >= top_y[1] + 9'd2) || (top_y[1] >= sy_d + 9'd2)) &&
                    ((top_cnt[2] == 0) || (sx_d >= top_x[2] + 9'd2) || (top_x[2] >= sx_d + 9'd2) ||
                                         (sy_d >= top_y[2] + 9'd2) || (top_y[2] >= sy_d + 9'd2)) &&
                    ((top_cnt[3] == 0) || (sx_d >= top_x[3] + 9'd2) || (top_x[3] >= sx_d + 9'd2) ||
                                         (sy_d >= top_y[3] + 9'd2) || (top_y[3] >= sy_d + 9'd2))) begin
                    if (acc_rd > top_cnt[0]) begin
                        top_cnt[3]<=top_cnt[2]; top_x[3]<=top_x[2]; top_y[3]<=top_y[2];
                        top_cnt[2]<=top_cnt[1]; top_x[2]<=top_x[1]; top_y[2]<=top_y[1];
                        top_cnt[1]<=top_cnt[0]; top_x[1]<=top_x[0]; top_y[1]<=top_y[0];
                        top_cnt[0]<=acc_rd; top_x[0]<=sx_d; top_y[0]<=sy_d;
                    end
                    else if (acc_rd > top_cnt[1]) begin
                        top_cnt[3]<=top_cnt[2]; top_x[3]<=top_x[2]; top_y[3]<=top_y[2];
                        top_cnt[2]<=top_cnt[1]; top_x[2]<=top_x[1]; top_y[2]<=top_y[1];
                        top_cnt[1]<=acc_rd; top_x[1]<=sx_d; top_y[1]<=sy_d;
                    end
                    else if (acc_rd > top_cnt[2]) begin
                        top_cnt[3]<=top_cnt[2]; top_x[3]<=top_x[2]; top_y[3]<=top_y[2];
                        top_cnt[2]<=acc_rd; top_x[2]<=sx_d; top_y[2]<=sy_d;
                    end
                    else begin
                        top_cnt[3]<=acc_rd; top_x[3]<=sx_d; top_y[3]<=sy_d;
                    end
                end
                if (sx == ACC_W-1) begin
                    sx <= 0;
                    if (sy == ACC_H-1) begin cand <= 0; a <= 0; st <= S_NEWC; end
                    else begin sy <= sy + 1'b1; st <= S_TOP; end
                end
                else begin sx <= sx + 1'b1; st <= S_TOP; end
            end

            // ---------------- candidate ----------------------------------
            S_NEWC: begin
                if (top_cnt[cand] == 0) begin
                    c_ok[cand] <= 1'b0; c_score[cand] <= 16'd0;
                    if (cand == TOPK-1) st <= S_SCORE;
                    else begin cand <= cand + 1'b1; st <= S_NEWC; end
                end
                else begin
                    cx <= $signed({1'b0, top_x[cand], 2'b10});
                    cy <= $signed({1'b0, top_y[cand], 2'b10});
                    ev_i <= 0; n_ev <= 0; sr <= 0; trace <= 0;
                    gx_acc <= 0; gy_acc <= 0;
                    scan_bin <= 0; med_bin <= 0; cum <= 0; med_found <= 0;
                    peak_v <= 0; peak_bin <= 11'd6; h_p2 <= 0; h_p1 <= 0;
                    sv_i <= 0; band16 <= 0; cov <= 0; ntr <= 0; quad <= 0;
                    pass <= P_INIT; st <= S_HCLR;
                end
            end

            S_HCLR: begin
                if (scan_bin == RB-1) begin
                    scan_bin <= 0; ev_i <= 0; n_ev <= 0; sr <= 0; st <= S_PFETCH;
                end
                else scan_bin <= scan_bin + 1'b1;
            end

            // ---------------- per-event pipeline -------------------------
            S_PFETCH: st <= S_PD1;
            S_PD1: begin
                ex <= ev_rd[47:36]; ey <= ev_rd[35:24];
                ux <= $signed(ev_rd[23:14]); uy <= $signed(ev_rd[13:4]);
                esec <= ev_rd[3:0];
                dxs <= $signed({1'b0,ev_rd[47:36]}) - cx;
                dys <= $signed({1'b0,ev_rd[35:24]}) - cy;
                mul_a <= $signed({1'b0,ev_rd[47:36]}) - cx;
                mul_b <= $signed({1'b0,ev_rd[47:36]}) - cx;
                st <= S_PD2;
            end
            S_PD2: begin
                r2a <= mul_p[23:0];
                mul_a <= dys; mul_b <= dys;
                st <= S_PD3;
            end
            S_PD3: begin
                r2 <= r2a + mul_p[23:0];
                if (((r2a + mul_p[23:0]) < 24'd484) || ((r2a + mul_p[23:0]) > 24'd40000))
                    st <= S_PNEXT;
                else begin
                    mul_a <= dxs; mul_b <= {{14{ux[9]}}, ux};
                    st <= S_PD4;
                end
            end
            S_PD4: begin
                dotx <= mul_p[23:0];
                mul_a <= dys; mul_b <= {{14{uy[9]}}, uy};
                st <= S_PD5;
            end
            S_PD5: begin
                dot <= dotx + mul_p[23:0];
                sqrt_rem <= 16'd0; sqrt_root <= 8'd0; sqrt_i <= 3'd7;
                st <= S_PSQ;
            end
            S_PSQ: begin
                if (sqrt_i == 3'd7)
                    dot_abs_r <= dot[23] ? (~dot + 24'd1) : dot;
                if (sqrt_i == 3'd0) begin rho <= sqrt_root_n; st <= S_PG0; end
                else begin
                    sqrt_rem <= sqrt_rem_n;
                    sqrt_root <= sqrt_root_n;
                    sqrt_i <= sqrt_i - 1'b1;
                end
            end
            S_PG0: begin
                // register the gate decision so the 24-bit |dot| adder and the
                // band compare are not on the multiplier-enable path
                gate_ok <= dir_ok && win_ok;
                esec    <= sector16(dxs[15:0], dys[15:0]);
                st <= S_PGATE;
            end
            S_PGATE: begin
                if (gate_ok) begin
                    eval_ <= $signed({1'b0,rho}) - $signed({1'b0,r0});
                    mul_a <= $signed({1'b0,rho}) - $signed({1'b0,r0});
                    mul_b <= dxs;
                    h_idx <= rho - B0[7:0];
                    h_ra  <= rho - B0[7:0];
                    st <= S_PG1;
                end
                else st <= S_PNEXT;
            end
            S_PG1: begin
                gxt <= mul_p[23:0];
                mul_a <= eval_; mul_b <= dys;
                st <= S_PG2;
            end
            S_PG2: begin
                gyt <= mul_p[23:0];
                if (pass == P_REF) begin
                    trace  <= trace + {1'b0, r2};
                    gx_acc <= gx_acc + gxt;
                    gy_acc <= gy_acc + gyt;
                    sr     <= sr + rho;
                    n_ev   <= n_ev + 1'b1;
                end
                else begin
                    if (pass == P_FIN) n_align <= n_align + 1'b1;
                    else               n_ev    <= n_ev + 1'b1;
                end
                st <= S_PNEXT;
            end
            S_PNEXT: begin
                if (({1'b0,ev_i} + 1'b1) >= proc_count) begin
                    if (pass == P_INIT) begin
                        med_bin <= 0; cum <= 0; med_found <= 1'b0; st <= S_MED;
                    end
                    else if (pass == P_REF) st <= S_RSOLVE;
                    else begin
                        scan_bin <= 0; peak_v <= 0; peak_bin <= 11'd6;
                        h_p2 <= 0; h_p1 <= 0; st <= S_PEAK;
                    end
                end
                else begin ev_i <= ev_i + 1'b1; st <= S_PFETCH; end
            end

            // ---------------- median radius -------------------------------
            S_MED: begin h_ra <= med_bin; st <= S_MEDW; end
            S_MEDW: st <= S_MEDC;
            S_MEDC: begin
                if (!med_found && ((({1'b0,cum} + {2'b0,h_rd[10:0]}) << 1) >= {2'b0,n_ev})) begin
                    r0 <= med_bin + B0[7:0]; med_found <= 1'b1;
                end
                cum <= cum + h_rd[10:0];
                if (med_bin == RB-1) begin
                    if (!med_found) r0 <= R_MIN[7:0];
                    iter <= 0; pass <= P_REF; ev_i <= 0; n_ev <= 0; sr <= 0;
                    trace <= 0; gx_acc <= 0; gy_acc <= 0;
                    st <= S_PFETCH;
                end
                else begin med_bin <= med_bin + 1'b1; st <= S_MED; end
            end

            // ---------------- Gauss-Newton --------------------------------
            S_RSOLVE: begin
                if ((n_ev < 16'd8) || (trace == 25'd0)) st <= S_RCAND;
                else begin
                    dv_num <= {10'd0, absgx, 12'd0};
                    dv_den <= {23'd0, trace};
                    dv_start <= 1'b1;
                    sx_sign <= gx_acc[25];
                    st <= S_RWAITQ;
                end
            end
            S_RWAITQ: if (dv_done) begin qx <= dv_q[15:0]; st <= S_RLOADY; end
            S_RLOADY: begin
                dv_num <= {10'd0, absgy, 12'd0};
                dv_den <= {23'd0, trace};
                dv_start <= 1'b1;
                sy_sign <= gy_acc[25];
                st <= S_RWAITY;
            end
            S_RWAITY: if (dv_done) begin
                qy <= dv_q[15:0];
                mul_a <= sx_sign ? -$signed({1'b0,qx}) : $signed({1'b0,qx});
                mul_b <= {16'd0, r0};
                st <= S_RMULX;
            end
            S_RMULX: begin
                if (step_raw > 24'sd12) step_x <= 24'sd12;
                else if (step_raw < -24'sd12) step_x <= -24'sd12;
                else step_x <= step_raw;
                mul_a <= sy_sign ? -$signed({1'b0,qy}) : $signed({1'b0,qy});
                mul_b <= {16'd0, r0};
                st <= S_RMULY;
            end
            S_RMULY: begin
                if (step_raw > 24'sd12) step_y <= 24'sd12;
                else if (step_raw < -24'sd12) step_y <= -24'sd12;
                else step_y <= step_raw;
                cx <= cx + step_x[12:0];
                cy <= cy + step_y[12:0];
                dv_num <= {30'd0, sr};
                dv_den <= {37'd0, n_ev};
                dv_start <= 1'b1;
                st <= S_RWAITM;
            end
            S_RWAITM: if (dv_done) begin r0 <= dv_q[7:0]; st <= S_RCAND; end
            S_RCAND: begin
                iter <= iter + 1'b1;
                ev_i <= 0; n_ev <= 0; sr <= 0; trace <= 0; gx_acc <= 0; gy_acc <= 0;
                if (iter == ITERS-1) begin
                    pass <= P_FIN; n_align <= 0;
                    ls_bx <= cx; ls_by <= cy;
                    ls_small <= (r0 <= 8'd40);
                    ls_last <= (r0 <= 8'd40) ? 4'd8 : 4'd0;
                    ls_i <= 4'd0; ls_score <= 16'd0; ls_ok <= 1'b0;
                    st <= S_FIN0;
                end
                else st <= S_PFETCH;
            end

            // -------- final pass, centre +-1 local search (5 points) -------
            S_FIN0: begin
                cx <= ls_bx + {{9{ls_ox[3]}}, ls_ox};
                cy <= ls_by + {{9{ls_oy[3]}}, ls_oy};
                n_align <= 0; scan_bin <= 0; peak_v <= 0; peak_bin <= 11'd6;
                h_p2 <= 0; h_p1 <= 0;
                st <= S_HCLR;
            end

            // ---------------- peak scan -----------------------------------
            S_PEAK: begin h_ra <= scan_bin; st <= S_PEAKW; end
            S_PEAKW: st <= S_PEAKC;
            S_PEAKC: begin
                if ((scan_bin >= 8'd7) && (scan_bin <= 8'd141) &&
                    ((h_p2 + h_p1 + h_rd[10:0]) > peak_v)) begin
                    peak_v <= h_p2 + h_p1 + h_rd[10:0];
                    peak_bin <= {2'd0, scan_bin} - 11'd1;
                end
                h_p2 <= h_p1; h_p1 <= h_rd[10:0];
                if (scan_bin == 8'd141) begin
                    sv_i <= 0; band16 <= 16'd0; st <= S_COVB;
                end
                else begin scan_bin <= scan_bin + 1'b1; st <= S_PEAK; end
            end

            // ---------------- coverage / transitions ----------------------
            S_COVB: begin
                h_ra <= ((peak_bin > 11'd3) ? (peak_bin[7:0] - 8'd3) : 8'd0) + sv_i;
                st <= S_COVW;
            end
            S_COVW: st <= S_COVC;
            S_COVC: begin
                band16 <= band16 | h_rd[26:11];
                if (sv_i == 8'd6) begin
                    band_r <= band_final;
                    rot    <= band_final;
                    gap_i  <= 8'd0; maxr <= 5'd0;
                    dv_num <= {peak_v, 8'd0};
                    dv_den <= {32'd0, rexp_q};
                    dv_start <= 1'b1;
                    st <= S_GAP;
                end
                else begin sv_i <= sv_i + 1'b1; st <= S_COVB; end
            end

            S_GAP: begin
                if (tz16(rot) > maxr) maxr <= tz16(rot);
                rot <= {rot[14:0], rot[15]};
                if (gap_i == 8'd15) begin
                    cov  <= pop16(band_r);
                    ntr  <= trans16(band_r);
                    quad <= (|band_r[3:0]) && (|band_r[7:4]) &&
                            (|band_r[11:8]) && (|band_r[15:12]);
                    st <= S_FILL;
                end
                else gap_i <= gap_i + 1'b1;
            end
            S_FILL: if (dv_done) begin
                fill_q8 <= (dv_q > 48'd255) ? 16'd255 : dv_q[15:0];
                st <= S_CDONE;
            end
            S_CDONE: begin
                if (ls_i == ls_last) begin
                    // final decision must include THIS pass's result (the
                    // running ls_* registers are one cycle behind)
                    if (ok_band && (!ls_ok || ((({11'd0,cov} << 8) + fill_q8) > ls_score))) begin
                        c_r[cand] <= peak_r; c_cx[cand] <= cx[11:0]; c_cy[cand] <= cy[11:0];
                        c_cov[cand] <= cov; c_score[cand] <= ({11'd0,cov} << 8) + fill_q8;
                        c_ok[cand] <= 1'b1;
                    end
                    else begin
                        c_r[cand] <= ls_r; c_cx[cand] <= ls_cx; c_cy[cand] <= ls_cy;
                        c_cov[cand] <= ls_cov; c_score[cand] <= ls_score;
                        c_ok[cand] <= ls_ok;
                    end
                    if (cand == TOPK-1) st <= S_SCORE;
                    else begin cand <= cand + 1'b1; st <= S_NEWC; end
                end
                else begin
                    if (ok_band && (!ls_ok || ((({11'd0,cov} << 8) + fill_q8) > ls_score))) begin
                        ls_score <= ({11'd0,cov} << 8) + fill_q8;
                        ls_r <= peak_r; ls_cx <= cx[11:0]; ls_cy <= cy[11:0];
                        ls_cov <= cov; ls_ok <= 1'b1;
                    end
                    ls_i <= ls_i + 1'b1; st <= S_FIN0;
                end
            end

            S_SCORE: begin
                best_score <= 16'd0; best_r <= 12'd0; best_cx <= 12'd0;
                best_cy <= 12'd0; best_ok <= 1'b0; cand <= 3'd0; st <= S_SEL;
            end
            S_SEL: begin
                // one candidate per cycle: keeps the 4-way best-of mux off
                // the critical path (gate timing showed -1.4 ns on it)
                // Prefer a smaller radius when its score is within SCORE_MARGIN
                // of the best (the larger circle otherwise wins on the extra
                // statistical sharpness of having more events).
                if (c_ok[cand] && (!best_ok ||
                    (c_score[cand] > (best_score + SCORE_MARGIN)) ||
                    (((c_score[cand] + SCORE_MARGIN) >= best_score) &&
                     (c_r[cand] < best_r)))) begin
                    best_score <= c_score[cand];
                    best_r     <= c_r[cand];
                    best_cx    <= c_cx[cand];
                    best_cy    <= c_cy[cand];
                    best_ok    <= 1'b1;
                end
                if (cand == TOPK-1) st <= S_DONE;
                else cand <= cand + 1'b1;
            end
            S_DONE: begin
                // Bounded hold: a processed frame without a valid circle keeps
                // the previous box for HOLD processed frames, then clears.
                // Skipped (cap_drop) frames never reach S_DONE, so they do not
                // age the hold and never write an invalid result.
                if (best_ok) begin
                    O_valid <= 1'b1;
                    O_cx <= best_cx; O_cy <= best_cy; O_r <= best_r;
                    O_minx <= (best_cx > best_r) ? (best_cx - best_r) : 0;
                    O_maxx <= (((best_cx + best_r) < IMG_W) ? (best_cx + best_r) : IMG_W-1);
                    O_miny <= (best_cy > best_r) ? (best_cy - best_r) : 0;
                    O_maxy <= (((best_cy + best_r) < IMG_H) ? (best_cy + best_r) : IMG_H-1);
                    O_score <= best_score;
                    hold_cnt <= HOLD[1:0];
                    h_cx <= best_cx; h_cy <= best_cy; h_r <= best_r;
                    h_minx <= (best_cx > best_r) ? (best_cx - best_r) : 0;
                    h_maxx <= (((best_cx + best_r) < IMG_W) ? (best_cx + best_r) : IMG_W-1);
                    h_miny <= (best_cy > best_r) ? (best_cy - best_r) : 0;
                    h_maxy <= (((best_cy + best_r) < IMG_H) ? (best_cy + best_r) : IMG_H-1);
                    h_score <= best_score;
                    O_flags <= {4'd0, drop_seen, proc_drop, proc_ovf, 1'b1};
                end
                else if (hold_cnt != 2'd0) begin
                    hold_cnt <= hold_cnt - 1'b1;
                    O_valid <= 1'b1;              // held, not a fresh pass
                    O_cx <= h_cx; O_cy <= h_cy; O_r <= h_r;
                    O_minx <= h_minx; O_maxx <= h_maxx;
                    O_miny <= h_miny; O_maxy <= h_maxy;
                    O_score <= h_score;
                    O_flags <= {4'd0, drop_seen, proc_drop, proc_ovf, 1'b0};
                end
                else begin
                    O_valid <= 1'b0;
                    O_cx <= 0; O_cy <= 0; O_r <= 0;
                    O_minx <= 0; O_maxx <= 0; O_miny <= 0; O_maxy <= 0;
                    O_score <= 0;
                    O_flags <= {4'd0, drop_seen, proc_drop, proc_ovf, 1'b0};
                end
                O_evt <= 1'b1;
                proc_done <= 1'b1;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
