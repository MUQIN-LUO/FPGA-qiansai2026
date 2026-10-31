// ==========================================================================
// 真实 isp_top 显示通路定向测试（RAW -> BLC -> demosaic -> chroma -> AWB ->
//   RAW/GRAY/POP/EDGE 选择 -> gamma），不是纯组合函数测试。
// --------------------------------------------------------------------------
// 覆盖：
//   1) 320 拍/行连续突发 + 行/场消隐的真实视频时序；
//   2) POP/RAW/GRAY/EDGE 四种模式下，输出 valid/tuser/tlast 结构与
//      “每帧 230400 拍、720 个 tlast、1 个 tuser”一致；
//   3) 模式在帧边界切换、以及帧内行边界切换，结构不中断；
//   4) POP 模式行首：4 个像素必须同色（若套印历史跨行泄漏，首像素会
//      取到上一行的 x-2 档位而颜色不同）。输入为逐行均匀、8 行一块的
//      明暗交替图案，块边界正好使上一行与本行档位不同。
//
// 监测点：u_isp_top.sel_*（96bit 显示选择，gamma 前）与 gam_*（gamma 后）。
// 说明：真实 RAW 前端 + AWB 收敛，因此只断言“结构/对齐/同色”属性，
//       不比对绝对像素值；POP 观感仍需上板。
// ==========================================================================
`timescale 1ns/1ps
module tb_isp_top_display;
    localparam integer W = 1280, H = 720;
    localparam integer LINE_CYCLES = 2300;
    localparam integer ACTIVE = W/4;
    localparam integer HBLANK = LINE_CYCLES - ACTIVE - 1;
    localparam integer VBLANK_LINES = 30;
    localparam integer FRAMES = 10;

    reg aclk = 0, cclk = 0;
    always #4.83 aclk = ~aclk;
    always #10.0 cclk = ~cclk;

    reg rst_n = 0;
    reg        tvalid = 0, tuser = 0, tlast = 0;
    reg [39:0] tdata = 0;
    reg [9:0]  tdest = 0;
    reg [1:0]  mode = 3;

    wire [11:0] det_minx, det_maxx, det_miny, det_maxy;
    wire det_valid, det_cand;
    wire [9:0] det_cnt;
    wire [7:0] rgb_r, rgb_g, rgb_b, lum;
    wire lum_tgl;
    wire [9:0] gr100, gb100;
    wire [1:0] cal_st;
    wire frm_tgl;
    wire [11:0] red_minx, red_maxx, red_miny, red_maxy;
    wire red_valid, red_cand, red_update;
    wire [9:0] red_count;
    wire [127:0] o_tdata;
    wire o_tlast, o_tuser, o_tvalid, i_tready;

    isp_top u_isp_top (
        .axi4s_video_aclk(aclk),
        .I_ccl_src_clk   (cclk),
        .I_cal_mode      (1'b0),
        .I_rst_n         (rst_n),
        .I_tlast         (tlast),
        .I_tuser         (tuser),
        .I_tdata         (tdata),
        .I_tvalid        (tvalid),
        .I_tdest         (tdest),
        .I_mode          (mode),
        .I_pal           (2'd0),
        .O_det_minx(det_minx), .O_det_maxx(det_maxx),
        .O_det_miny(det_miny), .O_det_maxy(det_maxy),
        .O_det_valid(det_valid), .O_det_cand(det_cand), .O_det_cnt(det_cnt),
        .O_rgb_r(rgb_r), .O_rgb_g(rgb_g), .O_rgb_b(rgb_b),
        .O_lum(lum), .O_lum_tgl(lum_tgl),
        .O_gr100(gr100), .O_gb100(gb100),
        .I_cal_trig(1'b0), .O_cal_st(cal_st), .O_frm_tgl(frm_tgl),
        .O_red_minx(red_minx), .O_red_maxx(red_maxx),
        .O_red_miny(red_miny), .O_red_maxy(red_maxy),
        .O_red_valid(red_valid), .O_red_cand(red_cand),
        .O_red_update(red_update), .O_red_count(red_count),
        .O_tready(1'b1),
        .O_tdata(o_tdata), .O_tlast(o_tlast), .O_tuser(o_tuser),
        .O_tvalid(o_tvalid), .I_tready(i_tready)
    );

    // 真实 96bit 显示选择 / gamma 后（层次化观察）
    wire [95:0] d_sel = u_isp_top.sel_data;
    wire        v_sel = u_isp_top.sel_valid;
    wire        u_sel = u_isp_top.sel_user;
    wire        l_sel = u_isp_top.sel_last;
    wire [95:0] d_gam = u_isp_top.gam_data;

    // ---------------- 输入驱动 ----------------
    function [9:0] row_val;
        input integer yy;
        begin
            // 8 行一块明暗交替：块边界上一行与本行档位不同
            row_val = (((yy / 8) % 2) == 0) ? 10'd200 : 10'd800;
        end
    endfunction

    task drive_line;
        input integer yy;
        input integer frame_first;
        integer xx;
        reg [9:0] v;
        begin
            v = row_val(yy);
            for (xx = 0; xx < W; xx = xx + 4) begin
                @(negedge aclk);
                tvalid <= 1'b1;
                tdata  <= {v, v, v, v};
                tuser  <= (frame_first && xx == 0);
                tlast  <= (xx == 1276);
                // 尊重反压：tready 低则保持本拍
                while (i_tready !== 1'b1) @(negedge aclk);
            end
            @(negedge aclk);
            tvalid <= 1'b0; tuser <= 0; tlast <= 0;
            repeat (HBLANK) @(negedge aclk);
        end
    endtask

    task drive_frame;
        input integer mid_switch;   // 1=在第 360 行把 mode 从 3 切到 0
        integer yy;
        begin
            for (yy = 0; yy < H; yy = yy + 1) begin
                if (mid_switch && yy == 360) mode = 2'd0;
                drive_line(yy, (yy == 0));
            end
            repeat (VBLANK_LINES * LINE_CYCLES) @(negedge aclk);
        end
    endtask

    // ---------------- 结构 / 对齐监测 ----------------
    integer frames_seen = 0;
    integer frame_valid = 0, frame_tlast = 0, frame_tuser = 0;
    integer line_start_chk = 0, line_start_bad = 0;
    integer cnt_v = 0, cnt_u = 0, cnt_l = 0, cnt_u_off = 0;
    // 各级流水 valid 计数，用于定位输出为零发生在哪一级
    integer c_dem = 0, c_dn = 0, c_awb = 0, c_rgb = 0, c_rdy = 0, c_in = 0;
    always @(posedge aclk) begin
        if (tvalid) c_in = c_in + 1;
        if (i_tready) c_rdy = c_rdy + 1;
        if (u_isp_top.m_aixs_tvalid) c_dem = c_dem + 1;
        if (u_isp_top.S_dn_v2)       c_dn  = c_dn + 1;
        if (u_isp_top.S_awb_valid)   c_awb = c_awb + 1;
        if (u_isp_top.S_rgb_valid)   c_rgb = c_rgb + 1;
    end
    reg     prev_last = 0;
    integer max_diff, dr, dg, db, kk;
    integer errors = 0;

    always @(posedge aclk) begin
        #1;
        if (v_sel) begin
            cnt_v = cnt_v + 1;
            if (u_sel) cnt_u = cnt_u + 1;
            if (l_sel) cnt_l = cnt_l + 1;
            if (u_sel) begin
                if (frames_seen > 0) begin
                    $display("FRAME %0d valid=%0d tlast=%0d tuser=%0d (checked=%0d)",
                             frames_seen, frame_valid, frame_tlast, frame_tuser,
                             (frames_seen > 2));
                    if (frames_seen > 2) begin
                        if (frame_valid != 230400) begin
                            $display("  [FAIL] valid count %0d != 230400", frame_valid);
                            errors = errors + 1;
                        end
                        if (frame_tlast != 720) begin
                            $display("  [FAIL] tlast count %0d != 720", frame_tlast);
                            errors = errors + 1;
                        end
                    end
                end
                frames_seen = frames_seen + 1;
                frame_valid = 1;          // 本拍计入新帧
                frame_tlast = l_sel ? 1 : 0;
                frame_tuser = 1;
            end
            else begin
                frame_valid = frame_valid + 1;
                if (l_sel) frame_tlast = frame_tlast + 1;
            end

            // POP 行首：上一有效拍 tlast 或本拍 tuser；本拍 4 像素必须同色。
            // prev_last 只在有效拍更新（行消隐期间保持），否则真实行结构下
            // 行首永远看不到 tlast，检查会变成空过。
            if ((mode == 3) && (prev_last || u_sel)) begin
                line_start_chk = line_start_chk + 1;
                max_diff = 0;
                for (kk = 1; kk < 4; kk = kk + 1) begin
                    dr = (d_gam[95:88]  > d_gam[95-kk*24 -: 8]) ? (d_gam[95:88]  - d_gam[95-kk*24 -: 8]) : (d_gam[95-kk*24 -: 8] - d_gam[95:88]);
                    dg = (d_gam[87:80]  > d_gam[87-kk*24 -: 8]) ? (d_gam[87:80]  - d_gam[87-kk*24 -: 8]) : (d_gam[87-kk*24 -: 8] - d_gam[87:80]);
                    db = (d_gam[79:72]  > d_gam[79-kk*24 -: 8]) ? (d_gam[79:72]  - d_gam[79-kk*24 -: 8]) : (d_gam[79-kk*24 -: 8] - d_gam[79:72]);
                    if (dr > max_diff) max_diff = dr;
                    if (dg > max_diff) max_diff = dg;
                    if (db > max_diff) max_diff = db;
                end
                if (max_diff > 3) begin
                    line_start_bad = line_start_bad + 1;
                    if (line_start_bad <= 5)
                        $display("  [FAIL] POP line-start pixels differ: %h %h %h %h (max %0d)",
                                 d_gam[95:72], d_gam[71:48], d_gam[47:24], d_gam[23:0], max_diff);
                end
            end
            prev_last <= l_sel;      // 仅在有效拍更新，无效拍保持
        end
        else if (u_sel) begin
            cnt_u_off = cnt_u_off + 1;   // tuser 落在无效拍：结构不对齐
        end
        // 无效拍不改 prev_last：真实行消隐后，下一有效拍仍能看到上一行 tlast
    end

    // ---------------- 主流程 ----------------
    // ---------------- 短诊断探针（+TRACE=1）----------------
    // 抓 POP 模块输入侧：上一拍输入像素、当前 b_q/b_qq/first_q/line_start、
    // 以及对应输出 4 像素；用于区分上游数据不一致与 POP 逻辑不一致。
    integer trace_en, trace_n;
    reg        in_v_d, in_l_d, in_u_d;
    reg [95:0] in_d_d;
    integer tu_cnt, tu_on_valid, tu_off_valid;
    initial begin
        if (!$value$plusargs("TRACE=%d", trace_en)) trace_en = 0;
        trace_n = 0; tu_cnt = 0; tu_on_valid = 0; tu_off_valid = 0;
    end
    always @(posedge aclk) begin
        #1;
        if (u_isp_top.S_rgb_tuser) begin
            tu_cnt = tu_cnt + 1;
            if (u_isp_top.S_rgb_valid) tu_on_valid = tu_on_valid + 1;
            else                       tu_off_valid = tu_off_valid + 1;
        end
        if (trace_en && in_v_d && u_isp_top.u_pop96.first_q && trace_n < 12) begin
            trace_n = trace_n + 1;
            $display("TRACE LS in=%h %h %h %h | bq=%h bqq=%h ls=%b out=%h %h %h %h",
                     in_d_d[95:72], in_d_d[71:48], in_d_d[47:24], in_d_d[23:0],
                     u_isp_top.u_pop96.b_q, u_isp_top.u_pop96.b_qq,
                     u_isp_top.u_pop96.line_start,
                     u_isp_top.u_pop96.O_tdata[95:72], u_isp_top.u_pop96.O_tdata[71:48],
                     u_isp_top.u_pop96.O_tdata[47:24], u_isp_top.u_pop96.O_tdata[23:0]);
        end
        in_v_d <= u_isp_top.S_rgb_valid;
        in_l_d <= u_isp_top.S_rgb_tlast;
        in_u_d <= u_isp_top.S_rgb_tuser;
        in_d_d <= u_isp_top.S_rgb_data;
    end

    integer f, nframes;
    initial begin
        if (!$value$plusargs("FRAMES=%d", nframes)) nframes = FRAMES;
        repeat (20) @(negedge aclk);
        rst_n = 1;
        repeat (20) @(negedge aclk);

        // 帧 0-1: POP；帧 2-3: RAW；帧 4-5: GRAY；帧 6-7: EDGE；
        // 帧 8: POP 帧内切 RAW；帧 9: RAW
        for (f = 0; f < nframes; f = f + 1) begin
            case (f)
                0,1: mode = 2'd3;
                2,3: mode = 2'd0;
                4,5: mode = 2'd2;
                6,7: mode = 2'd1;
                8:   mode = 2'd3;
                default: mode = 2'd0;
            endcase
            drive_frame((f == 8) ? 1 : 0);
        end

        repeat (5000) @(negedge aclk);
        $display("COUNTS valid=%0d tuser_valid=%0d tuser_offvalid=%0d tlast_valid=%0d frames=%0d",
                 cnt_v, cnt_u, cnt_u_off, cnt_l, frames_seen);
        $display("TUSER_ALIGN total=%0d on_valid=%0d off_valid=%0d",
                 tu_cnt, tu_on_valid, tu_off_valid);
        $display("PIPE in=%0d tready_hi=%0d demosaic=%0d denoise=%0d awb=%0d rgb=%0d",
                 c_in, c_rdy, c_dem, c_dn, c_awb, c_rgb);
        $display("LINE_START_CHECKS=%0d BAD=%0d", line_start_chk, line_start_bad);
        if (cnt_v == 0) begin $display("  [FAIL] no valid output beats"); errors=errors+1; end
        if (cnt_u == 0) begin $display("  [FAIL] no tuser on valid beat"); errors=errors+1; end
        if (cnt_u_off != 0) begin $display("  [FAIL] tuser off valid beat: %0d", cnt_u_off); errors=errors+1; end
        if (line_start_chk == 0) begin $display("  [FAIL] line-start check never ran"); errors=errors+1; end
        if (line_start_bad != 0) errors = errors + 1;
        if (nframes >= 8 && frames_seen < 8) begin
            $display("  [FAIL] too few frames seen: %0d", frames_seen); errors=errors+1;
        end
        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
