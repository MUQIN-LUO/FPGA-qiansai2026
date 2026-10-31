// ==========================================================================
// AWB 实际应用集成回归
//   实例化工程实际使用的：isp_gray_world + isp_awb_arbiter + awb + signal_delay
//   仅 BRAM 原语用行为模型（sim/tb/blk_mem_gen_awb_delay_signal.v）。
//
//   检查：
//     1) 上电 INIT 生效（偏蓝），首帧后自动增益在下一帧首整对提交；
//     2) 蓝色场景多帧后收敛到 G/R、G/B（EMA 1/4），稳定；
//     3) 场景改红后增益方向正确；
//     4) 过暗/过曝样本不足时不提交，保持上一对；
//     5) AWB_MODE=0 时 CAL 提交不影响增益（K1 不标定）；
//        AWB_MODE=1 时 CAL 提交整对生效。
//   同时按 isp_top 的 OSD 公式 (g*25)>>14 打印实际应用值。
// ==========================================================================
`timescale 1ns/1ps
module tb_awb_apply;

    localparam integer W = 1280, H = 720;
    localparam integer BEATS = W/4;         // 320
    localparam integer VBLANK = 2000;

    reg clk = 0;
    always #4.83 clk = ~clk;                // 103.52 MHz

    reg rst_n = 0;
    reg        valid = 0;
    reg [95:0] tdata = 0;
    reg        tuser = 0, tlast = 0;

    // ---------------- 灰度世界 ----------------
    wire [19:0] gw_r, gw_b;
    wire        gw_commit;
    isp_gray_world #(.IMG_WIDTH(W), .IMG_HEIGHT(H)) u_gw (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_valid(valid), .I_tdata(tdata),
        .O_gain_r(gw_r), .O_gain_b(gw_b), .O_commit(gw_commit)
    );

    // ---------------- 仲裁器 ----------------
    reg  mode = 0;                 // 0 = 灰度世界，1 = 白纸标定
    reg  cal_commit = 0;
    reg  [19:0] cal_r = 20'd65536, cal_b = 20'd65536;
    reg  cal_en = 0;
    wire [19:0] wr, wb;
    wire        commit_en;
    wire        gain_evt;
    isp_awb_arbiter u_arb (
        .I_clk(clk), .I_rst_n(rst_n), .I_awb_mode(mode),
        .I_gw_commit(gw_commit), .I_gw_r(gw_r), .I_gw_b(gw_b),
        .I_cal_commit(cal_commit), .I_cal_r(cal_r), .I_cal_b(cal_b),
        .I_cal_commit_en(cal_en), .I_gain_evt(gain_evt),
        .O_wr(wr), .O_wb(wb), .O_commit_en(commit_en)
    );

    // ---------------- awb + 延迟线 ----------------
    wire [95:0] o_tdata;
    wire        o_tvalid, o_tlast, o_tuser;
    wire [19:0] ap_r, ap_b;
    awb #(.IMG_WIDTH(W), .IMG_HEIGHT(H)) u_awb (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_tlast(tlast), .I_tuser(tuser), .I_tdata(tdata), .I_tvalid(valid),
        .I_tready(), .O_tlast(o_tlast), .O_tuser(o_tuser),
        .O_tdata(o_tdata), .O_tvalid(o_tvalid), .O_tready(1'b1),
        .I_gain_r(wr), .I_gain_b(wb),
        .O_meas_data(),
        .O_gain_r_ap(ap_r), .O_gain_b_ap(ap_b),
        .O_gain_evt(gain_evt),
        .I_gain_commit_en(commit_en)
    );

    // ---------------- 场景 ----------------
    task drive_frame;
        input integer rr, gg, bb;
        input integer vblank;
        integer yy, xx, t;
        reg [23:0] pix;
        begin
            repeat (vblank) @(negedge clk);
            pix = {rr[7:0], gg[7:0], bb[7:0]};
            for (yy = 0; yy < H; yy = yy + 1) begin
                for (xx = 0; xx < W; xx = xx + 4) begin
                    @(negedge clk);
                    valid <= 1'b1;
                    tdata <= {pix, pix, pix, pix};
                    tuser <= (xx == 0 && yy == 0);
                    tlast <= (xx == 1276);
                end
            end
            @(negedge clk);
            valid <= 1'b0; tuser <= 0; tlast <= 0;
        end
    endtask

    // ---------------- 观测 ----------------
    integer evt_cnt = 0;
    integer ap_r_l, ap_b_l, osd_r_l, osd_b_l;
    always @(posedge clk) begin
        #1;
        if (gain_evt) begin
            evt_cnt <= evt_cnt + 1;
            ap_r_l <= ap_r; ap_b_l <= ap_b;
            osd_r_l <= (ap_r * 25) >> 14;
            osd_b_l <= (ap_b * 25) >> 14;
            $display("GAIN_EVT #%0d R=%0d(%0d.%02dx) B=%0d(%0d.%02dx) OSD R%0d B%0d",
                     evt_cnt+1, ap_r, ((ap_r*25)>>14)/100, ((ap_r*25)>>14)%100,
                     ap_b, ((ap_b*25)>>14)/100, ((ap_b*25)>>14)%100,
                     (ap_r*25)>>14, (ap_b*25)>>14);
        end
    end

    integer errors = 0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors = errors + 1; end
        end
    endtask

    integer i;
    initial begin
        $display("init gains: R=%0d B=%0d", 20'd76824, 20'd138731);
        repeat (20) @(negedge clk);
        rst_n = 1;
        repeat (20) @(negedge clk);
        check("reset applied = INIT", ap_r == 20'd76824 && ap_b == 20'd138731);

        // ---- 蓝色场景：首帧后自动提交，多帧收敛 ----
        drive_frame(60,120,200,0);                 // f1
        drive_frame(60,120,200,VBLANK);            // f2（本帧首整对提交）
        check("first auto commit happened", evt_cnt >= 1);
        check("first commit moves R up / B down",
              ap_r > 20'd76824 && ap_b < 20'd138731);
        $display("  after f2: R=%0d B=%0d (OSD %0d/%0d)", ap_r, ap_b, osd_r_l, osd_b_l);

        for (i = 0; i < 14; i = i + 1) drive_frame(60,120,200,VBLANK);
        check("blue converged R > 1.9x", ap_r > 20'd124518);
        check("blue converged B < 0.7x", ap_b < 20'd45875);
        $display("  after blue: R=%0d B=%0d (OSD %0d/%0d)", ap_r, ap_b, osd_r_l, osd_b_l);

        // 稳定：再两帧变化很小
        begin : stable
            integer r0, b0;
            r0 = ap_r; b0 = ap_b;
            drive_frame(60,120,200,VBLANK);
            drive_frame(60,120,200,VBLANK);
            check("stable: R change < 5%", (ap_r > r0-3200) && (ap_r < r0+3200));
            check("stable: B change < 5%", (ap_b > b0-2200) && (ap_b < b0+2200));
        end

        // ---- 红色场景 ----
        for (i = 0; i < 6; i = i + 1) drive_frame(200,100,50,VBLANK);
        check("red: R < 1.0x", ap_r < 20'd65536);
        check("red: B > 1.5x", ap_b > 20'd98304);
        $display("  after red: R=%0d B=%0d (OSD %0d/%0d)", ap_r, ap_b, osd_r_l, osd_b_l);

        // ---- 过暗/过曝：样本不足 -> 不提交，保持 ----
        begin : dark
            integer ev0, r0, b0;
            // 先排空上一帧（红色）尚未被 awb 提交的 pending：
            // 暗场首帧的首像素会把红场估计整对提交，之后才是稳定的旧值
            drive_frame(2,2,2,VBLANK);
            repeat (200) @(negedge clk);
            ev0 = evt_cnt; r0 = ap_r; b0 = ap_b;
            drive_frame(2,2,2,VBLANK);
            drive_frame(2,2,2,VBLANK);
            check("dark: no new commit", evt_cnt == ev0);
            check("dark: applied held", ap_r == r0 && ap_b == b0);
            drive_frame(255,255,255,VBLANK);
            drive_frame(255,255,255,VBLANK);
            check("overexposed: no new commit", evt_cnt == ev0);
            check("overexposed: applied held", ap_r == r0 && ap_b == b0);
        end

        // ---- AWB_MODE=0 下的 CAL 请求：不应生效 ----
        begin : cal_mode0
            integer ev0, r0, b0;
            ev0 = evt_cnt; r0 = ap_r; b0 = ap_b;
            cal_r = 20'd60000; cal_b = 20'd190000;   // 1.0x / 2.9x，明显区别
            @(negedge clk); cal_commit <= 1'b1; cal_en <= 1'b1;
            @(negedge clk); cal_commit <= 1'b0;
            drive_frame(2,2,2,VBLANK);               // 样本不足，不应自动提交
            @(negedge clk); cal_en <= 1'b0;
            drive_frame(2,2,2,VBLANK);
            check("mode0: CAL request not applied",
                  (ap_r != 20'd60000) && (ap_b != 20'd190000));
            check("mode0: applied unchanged", ap_r == r0 && ap_b == b0);
        end

        // ---- AWB_MODE=1：CAL 请求整对生效 ----
        begin : cal_mode1
            mode <= 1'b1;
            repeat (5) @(negedge clk);
            cal_r = 20'd85000; cal_b = 20'd120000;   // 1.30x / 1.83x
            @(negedge clk); cal_commit <= 1'b1; cal_en <= 1'b1;
            @(negedge clk); cal_commit <= 1'b0;
            drive_frame(2,2,2,VBLANK);
            drive_frame(2,2,2,VBLANK);
            @(negedge clk); cal_en <= 1'b0;
            check("mode1: CAL gains applied atomically",
                  ap_r == 20'd85000 && ap_b == 20'd120000);
            $display("  mode1 applied: R=%0d B=%0d (OSD %0d/%0d)", ap_r, ap_b,
                     (ap_r*25)>>14, (ap_b*25)>>14);
        end

        // ---- K1 再次长按：切回自动，灰度世界恢复提交 ----
        begin : cal_back_to_auto
            mode <= 1'b0;
            repeat (5) @(negedge clk);
            // 灰度世界保留自己的上一次估计（红色场景），切回自动后继续 EMA，
            // 因此不能拿标定值当起点；多帧后应向当前蓝色场景收敛。
            for (i = 0; i < 10; i = i + 1) drive_frame(60,120,200,VBLANK);
            check("back to auto: auto gains resume and converge to blue",
                  ap_r > 20'd120000 && ap_b < 20'd55000);
            check("back to auto: no longer the CAL gains",
                  !(ap_r == 20'd85000 && ap_b == 20'd120000));
            $display("  auto resumed: R=%0d B=%0d (OSD %0d/%0d)", ap_r, ap_b,
                     (ap_r*25)>>14, (ap_b*25)>>14);
        end

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end

endmodule
