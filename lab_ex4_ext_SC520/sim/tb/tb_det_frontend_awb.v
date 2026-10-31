// ==========================================================================
// 真实 AWB + 检测链路集成回归：提前帧首 / 圆形结果 / FIFO 溢出恢复
// --------------------------------------------------------------------------
// 背景（Codex 桥接实验）：
//   AWB 的 O_tuser 按原厂显示约定直通（I_tuser），比 O_tvalid/O_tdata 早
//   约 52 拍。检测链路若直接用它，tuser 落在 valid=0 的拍上，掩膜 FIFO
//   收不到帧首 → CCL 的 S_frame_open 永远为 0，强制完成事件也送不出。
//
// 本 TB 实例化真实 RTL：
//   awb（含 signal_delay/BRAM 行为模型）→ edge_detect_96 + isp_bg_est +
//   isp_fg_mask → isp_ccl_bridge（FIFO/CCL/CDC）。
// 检测链路使用 AWB 新增的 O_tuser_alg（=I_tuser_r2，与 O_tvalid 同拍），
// 显示约定不变。
//
// 覆盖：
//   1) AWB 原始 O_tuser 确实 off-valid（复现问题）；
//   2) O_tuser_alg 在 valid 拍上（修复生效）；
//   3) 绿球帧产生 pass=1 的 CCL 事件（帧首打开、mask/RGB 对齐）；
//   4) 注入无消隐连续流使 FIFO 溢出，随后正常帧恢复 pass=1。
// ==========================================================================
`timescale 1ns/1ps
module tb_det_frontend_awb;
    localparam integer W = 1280, H = 720;
    localparam integer LINE_CYCLES = 2300;
    localparam integer ACTIVE = W/4;
    localparam integer HBLANK = LINE_CYCLES - ACTIVE - 1;
    localparam integer VBLANK_LINES = 30;

    reg aclk = 0, cclk = 0;
    always #4.83 aclk = ~aclk;
    always #20.00 cclk = ~cclk;   // 25 MHz，直接给 CCL 桥

    reg rst_n = 0;
    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [95:0] tdata = 0;

    // ---------------- AWB ----------------
    wire        awb_tready_i, awb_tlast, awb_tuser, awb_tuser_alg;
    wire [95:0] awb_tdata;
    wire        awb_tvalid;
    wire [23:0] awb_meas;
    wire [19:0] awb_gr_ap, awb_gb_ap;
    wire        awb_gain_evt;

    awb #(
        .IMG_HEIGHT (H),
        .IMG_WIDTH  (W),
        .INIT_GAIN_R(20'd65536),   // 1.0x，保持颜色便于检测
        .INIT_GAIN_B(20'd65536)
    ) u_awb (
        .I_clk          (aclk),
        .I_rst_n        (rst_n),
        .I_tlast        (tlast),
        .I_tuser        (tuser),
        .I_tdata        (tdata),
        .I_tvalid       (tvalid),
        .I_tready       (awb_tready_i),
        .O_tlast        (awb_tlast),
        .O_tuser        (awb_tuser),       // 显示约定：早于数据
        .O_tuser_alg    (awb_tuser_alg),   // 检测专用：与 O_tvalid 同拍
        .O_tdata        (awb_tdata),
        .O_tvalid       (awb_tvalid),
        .O_tready       (1'b1),
        .I_gain_r       (20'd65536),
        .I_gain_b       (20'd65536),
        .O_meas_data    (awb_meas),
        .O_gain_r_ap    (awb_gr_ap),
        .O_gain_b_ap    (awb_gb_ap),
        .O_gain_evt     (awb_gain_evt),
        .I_gain_commit_en(1'b1)
    );

    // 与 isp_top 一致的检测输入寄存
    reg [95:0] din_q;
    reg        dv_q, dl_q, du_q;
    always @(posedge aclk or negedge rst_n) begin
        if (!rst_n) begin
            din_q <= 96'd0; dv_q <= 1'b0; dl_q <= 1'b0; du_q <= 1'b0;
        end
        else begin
            din_q <= awb_tdata;
            dv_q  <= awb_tvalid;
            dl_q  <= awb_tlast;
            du_q  <= awb_tuser_alg;   // 检测用同拍帧首
        end
    end

    wire [7:0] bg_r, bg_g, bg_b;
    isp_bg_est #(.IMG_WIDTH(W), .IMG_HEIGHT(H)) u_bg (
        .I_clk   (aclk),
        .I_rst_n (rst_n),
        .I_tvalid(awb_tvalid),
        .I_tuser (awb_tuser_alg),
        .I_tdata (awb_tdata),
        .O_bg_r  (bg_r), .O_bg_g(bg_g), .O_bg_b(bg_b)
    );

    wire [95:0] e_tdata; wire e_tvalid, e_tuser, e_tlast;
    edge_detect_96 #(.IMG_WIDTH(W), .IMG_HEIGHT(H), .EDGE_TH(11'd48)) u_edge (
        .I_clk(aclk), .I_rst_n(rst_n),
        .I_tlast(dl_q), .I_tuser(du_q), .I_tdata(din_q), .I_tvalid(dv_q),
        .I_tready(1'b1),
        .O_tlast(e_tlast), .O_tuser(e_tuser), .O_tdata(e_tdata),
        .O_tvalid(e_tvalid), .O_tready()
    );

    wire [3:0]  fg_mask; wire [14:0] fg_rgb;
    wire        fg_valid, fg_tlast, fg_tuser; wire [11:0] fg_x, fg_y;
    isp_fg_mask #(.IMG_WIDTH(W), .IMG_HEIGHT(H), .BG_TH(10'd60)) u_fg (
        .I_clk(aclk), .I_rst_n(rst_n),
        .I_tvalid(dv_q), .I_tlast(dl_q), .I_tuser(du_q), .I_tdata(din_q),
        .I_bg_r(bg_r), .I_bg_g(bg_g), .I_bg_b(bg_b),
        .I_edge_valid(e_tvalid), .I_edge_tlast(e_tlast),
        .I_edge_tuser(e_tuser), .I_edge_tdata(e_tdata),
        .O_mask(fg_mask), .O_rgb(fg_rgb), .O_valid(fg_valid), .O_tlast(fg_tlast),
        .O_tuser(fg_tuser), .O_x(fg_x), .O_y(fg_y)
    );

    wire        o_evt, o_pass, o_cand, o_ovf;
    wire [11:0] o_w, o_h, o_minx, o_maxx, o_miny, o_maxy;
    wire [23:0] o_cnt; wire [20:0] o_chord; wire [10:0] o_rows;
    wire [3:0]  o_lab; wire dbg_full, dbg_drop;
    isp_ccl_bridge #(
        .IMG_WIDTH(W), .IMG_HEIGHT(H),
        .MIN_W_PX(12'd24), .MAX_W_PX(12'd720), .MIN_CELLS(12'd6)
    ) u_ccl (
        .I_mipi_clk(aclk), .I_rst_n(rst_n),
        .I_fg_valid(fg_valid), .I_fg_mask(fg_mask), .I_fg_rgb(fg_rgb),
        .I_fg_tuser(fg_tuser), .I_fg_tlast(fg_tlast),
        .I_det_mask(1'b0), .I_ccl_clk(cclk),
        .O_evt(o_evt), .O_w(o_w), .O_h(o_h), .O_cnt(o_cnt), .O_chord(o_chord),
        .O_rows(o_rows), .O_minx(o_minx), .O_maxx(o_maxx),
        .O_miny(o_miny), .O_maxy(o_maxy),
        .O_pass(o_pass), .O_cand(o_cand),
        .O_ok_size(), .O_ok_asp(), .O_ok_rows(), .O_ok_chord(),
        .O_lab(o_lab), .O_ovf(o_ovf),
        .O_dbg_full(dbg_full), .O_dbg_drop(dbg_drop)
    );

    // ---------------- 场景 ----------------
    function [23:0] scene_pix;
        input integer px, py;
        integer dx, dy, r2;
        begin
            dx = px - 640; dy = py - 360; r2 = dx*dx + dy*dy;
            scene_pix = (r2 <= 10000) ? 24'h28b43c : 24'h6e6e6e;
        end
    endfunction

    task drive_frame;
        input integer overflow_mode;   // 1 = 行内无消隐，制造 FIFO 溢出
        integer yy, xx, k;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge aclk);
                    tvalid <= 1'b1;
                    tdata  <= { scene_pix(xx+3,yy), scene_pix(xx+2,yy),
                                scene_pix(xx+1,yy), scene_pix(xx,yy) };
                    tuser  <= (yy == 0 && xx == 0);
                    tlast  <= (xx == 1276);
                    // 帧首特征像素：首拍 0x808080，次拍回到背景 0x6e6e6e。
                    // 用于断言 O_tuser_alg 必须与"第一个有效输出像素"同拍，
                    // 而不是仅仅落在任意有效拍上。
                    if (yy == 0 && xx == 0)
                        tdata <= {24'h808080, 24'h808080, 24'h808080, 24'h808080};
                end
                @(negedge aclk);
                tvalid <= 1'b0;
                for (k = 0; k < HBLANK; k = k + 1) @(negedge aclk);
            end
            for (k = 0; k < VBLANK_LINES * LINE_CYCLES; k = k + 1)
                @(negedge aclk);
            tuser <= 0; tlast <= 0;
        end
    endtask

    task drive_overflow_frame;
        integer yy, xx;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge aclk);
                    tvalid <= 1'b1;
                    tdata  <= { scene_pix(xx+3,yy), scene_pix(xx+2,yy),
                                scene_pix(xx+1,yy), scene_pix(xx,yy) };
                    tuser  <= (yy == 0 && xx == 0);
                    tlast  <= (xx == 1276);
                    if (yy == 0 && xx == 0)
                        tdata <= {24'h808080, 24'h808080, 24'h808080, 24'h808080};
                end
                @(negedge aclk);
                tvalid <= 1'b0;   // 不插行消隐
            end
            repeat (4000) @(negedge aclk);
            tuser <= 0; tlast <= 0;
        end
    endtask

    // ---------------- 监测 ----------------
    integer u_cnt=0, u_on=0, u_off=0;
    integer ua_cnt=0, ua_on=0, ua_off=0;
    // 帧首特征像素断言：tuser_alg 当拍必须是首个有效输出像素（0x808080），
    // 下一有效拍必须是第二个输入拍（0x6e6e6e）。
    integer alg_n=0, alg_bad=0, next_bad=0;
    reg     pend=0;
    always @(posedge aclk) begin
        #1;
        if (awb_tuser_alg) begin
            alg_n = alg_n + 1;
            if (!awb_tvalid ||
                (awb_tdata !== {24'h808080,24'h808080,24'h808080,24'h808080}))
                alg_bad = alg_bad + 1;
            pend = 1;
        end
        else if (pend && awb_tvalid) begin
            pend = 0;
            if (awb_tdata !== {24'h6e6e6e,24'h6e6e6e,24'h6e6e6e,24'h6e6e6e})
                next_bad = next_bad + 1;
        end
    end
    always @(posedge aclk) begin
        if (awb_tuser) begin
            u_cnt = u_cnt + 1;
            if (awb_tvalid) u_on = u_on + 1; else u_off = u_off + 1;
        end
        if (awb_tuser_alg) begin
            ua_cnt = ua_cnt + 1;
            if (awb_tvalid) ua_on = ua_on + 1; else ua_off = ua_off + 1;
        end
    end

    integer evt_cnt=0, last_pass, last_w, last_h, full_cnt=0;
    // 逐拍波形：fg_tuser 前后 8 拍（din_q/du_q/edge/S1/S2/mask/fg RGB）
    reg [95:0] h_din [0:7];
    reg        h_du [0:7], h_ev [0:7], h_et [0:7], h_fgu [0:7];
    reg [14:0] h_s1 [0:7], h_s2 [0:7], h_fgrgb [0:7];
    reg [3:0]  h_mask [0:7];
    integer hi, dbg_tr = 0;
    always @(posedge aclk) begin
        for (hi = 7; hi > 0; hi = hi - 1) begin
            h_din[hi]  <= h_din[hi-1];
            h_du[hi]   <= h_du[hi-1];
            h_ev[hi]   <= h_ev[hi-1];
            h_et[hi]   <= h_et[hi-1];
            h_s1[hi]   <= h_s1[hi-1];
            h_s2[hi]   <= h_s2[hi-1];
            h_fgrgb[hi]<= h_fgrgb[hi-1];
            h_mask[hi] <= h_mask[hi-1];
            h_fgu[hi]  <= h_fgu[hi-1];
        end
        h_din[0]  <= din_q;
        h_du[0]   <= du_q;
        h_ev[0]   <= e_tvalid;
        h_et[0]   <= e_tuser;
        h_s1[0]   <= u_fg.S1_rgb;
        h_s2[0]   <= u_fg.S2_rgb;
        h_fgrgb[0]<= fg_rgb;
        h_mask[0] <= fg_mask;
        h_fgu[0]  <= fg_tuser;
        if (fg_tuser && dbg_tr < 2) begin
            dbg_tr = dbg_tr + 1;
            for (hi = 0; hi < 6; hi = hi + 1)
                $display("TR t-%0d din=%h du=%b ev=%b et=%b S1=%h S2=%h mask=%b fgu=%b fgrgb=%h",
                         hi, h_din[hi], h_du[hi], h_ev[hi], h_et[hi],
                         h_s1[hi], h_s2[hi], h_mask[hi], h_fgu[hi], h_fgrgb[hi]);
        end
    end

    integer last_minx, last_maxx, last_miny, last_maxy;
    // mask/RGB 逐拍对应：帧首第一拍 = marker，行首/行尾 = 背景灰
    integer fg_first_n=0, fg_first_bad=0, fg_line_n=0, fg_line_bad=0;
    integer fg_last_n=0, fg_last_bad=0;
    reg [14:0] fg_rgb_d; reg fg_valid_d;
    always @(posedge aclk) begin
        #1;
        fg_rgb_d   <= fg_rgb;
        fg_valid_d <= fg_valid;
        if (fg_valid) begin
            if (fg_tuser) begin
                fg_first_n = fg_first_n + 1;
                if (fg_first_n <= 4)
                    $display("FG_FIRST t=%0t fgrgb=%h din_q=%h S1=%h S2=%h e_u=%b",
                             $time, fg_rgb, din_q, u_fg.S1_rgb, u_fg.S2_rgb, e_tuser);
                if (fg_rgb !== 15'h4210)  // 0x808080 → 5:5:5 = (16,16,16)
                    fg_first_bad = fg_first_bad + 1;
            end
            if (fg_tlast && !fg_tuser) begin
                fg_line_n = fg_line_n + 1;
                if (fg_rgb !== 15'h35AD)  // 0x6e6e6e → (13,13,13)
                    fg_line_bad = fg_line_bad + 1;
            end
        end
        if (fg_valid_d && !fg_valid) begin
            fg_last_n = fg_last_n + 1;
            if (fg_rgb_d !== 15'h35AD)
                fg_last_bad = fg_last_bad + 1;
        end
    end
    always @(posedge aclk) begin
        #1;
        if (o_evt) begin
            evt_cnt = evt_cnt + 1;
            last_pass = o_pass; last_w = o_w; last_h = o_h;
            last_minx = o_minx; last_maxx = o_maxx;
            last_miny = o_miny; last_maxy = o_maxy;
        end
        if (dbg_full) full_cnt = full_cnt + 1;
    end

    integer errors = 0, full0, evt0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    initial begin
        repeat (20) @(negedge aclk);
        rst_n = 1;
        repeat (20) @(negedge aclk);

        // 帧 0：背景估计预热
        drive_frame(0);
        repeat (6000) @(negedge aclk);
        // 帧 1：评估
        evt0 = evt_cnt;
        drive_frame(0);
        repeat (6000) @(negedge aclk);
        $display("AWB tuser: total=%0d on_valid=%0d off_valid=%0d", u_cnt, u_on, u_off);
        $display("AWB tuser_alg: total=%0d on_valid=%0d off_valid=%0d", ua_cnt, ua_on, ua_off);
        $display("EVENTS=%0d last_pass=%0d w=%0d h=%0d full=%0d",
                 evt_cnt-evt0, last_pass, last_w, last_h, full_cnt);
        $display("FIRST-PIXEL: tuser_alg_pulses=%0d marker_bad=%0d next_bad=%0d",
                 alg_n, alg_bad, next_bad);
        $display("BBOX: (%0d,%0d)-(%0d,%0d) w=%0d h=%0d",
                 last_minx, last_miny, last_maxx, last_maxy, last_w, last_h);
        check("original O_tuser is off valid (reproduced)", u_off > 0);
        check("O_tuser_alg is on valid", ua_on > 0 && ua_off == 0);
        check("O_tuser_alg coincides with first valid pixel (marker)",
              alg_n > 0 && alg_bad == 0);
        check("second valid beat after tuser_alg is second input beat",
              next_bad == 0);
        check("circle detection passes through AWB", last_pass == 1 && last_w >= 186 && last_w <= 216);
        check("circle bbox x aligned (540..743 +/-2)",
              last_minx >= 538 && last_minx <= 542 &&
              last_maxx >= 741 && last_maxx <= 745);
        check("circle bbox y aligned (260..460 +/-2)",
              last_miny >= 258 && last_miny <= 262 &&
              last_maxy >= 458 && last_maxy <= 462);
        $display("FG ALIGN: first n=%0d bad=%0d | line-start n=%0d bad=%0d | line-end n=%0d bad=%0d",
                 fg_first_n, fg_first_bad, fg_line_n, fg_line_bad, fg_last_n, fg_last_bad);
        check("fg RGB at frame first beat = marker", fg_first_n > 0 && fg_first_bad == 0);
        check("fg RGB at line starts = background", fg_line_n > 0 && fg_line_bad == 0);
        check("fg RGB at line ends = background", fg_last_n > 0 && fg_last_bad == 0);

        // FIFO 溢出注入 + 恢复
        full0 = full_cnt;
        drive_overflow_frame();
        $display("overflow frame: full_cycles=%0d", full_cnt-full0);
        check("overflow injection exercised FIFO full", (full_cnt-full0) > 0);
        evt0 = evt_cnt;
        drive_frame(0);
        repeat (6000) @(negedge aclk);
        $display("recovery: events=%0d pass=%0d w=%0d", evt_cnt-evt0, last_pass, last_w);
        check("recovery after overflow passes", last_pass == 1 && last_w >= 186 && last_w <= 216);

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
