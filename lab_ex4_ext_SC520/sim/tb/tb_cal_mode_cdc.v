// ==========================================================================
// K1 标定电平 CDC 定向验证
// --------------------------------------------------------------------------
// 被测模块：design_top_wrapper.v 中的 cal_mode_cdc（顶层实际实例化的同一份 RTL）
// 覆盖：
//   1) 上升沿：O_mode 在 2 个 CSI 沿后变 1；同一拍 O_trig=1；start 成立；
//   2) 下降沿：O_mode 变 0 的同一拍 O_trig=1；abort 成立、start 不成立；
//   3) 稳定电平：长时间无 O_trig；
//   4) 复位：CSI 复位期间 O_mode=0/O_trig=0；释放后源为 0 时无伪触发；
//   5) 每次 mode 变化恰好一个 trig，trig 不与 mode 变化分离。
// 说明：这是 CDC 逻辑/同拍关系验证；顶层实例化关系另由 TD elaboration
//       网表检查（u_isp_top.I_cal_mode 的 net 必须是 S_cal_t2）。
// ==========================================================================
`timescale 1ns/1ps
module tb_cal_mode_cdc;

    reg sys_clk = 0, csi_clk = 0;
    always #10.0 sys_clk = ~sys_clk;     // 50 MHz
    always #4.83 csi_clk = ~csi_clk;     // 103.52 MHz

    reg sys_rst_n = 0, csi_rst_n = 0;
    reg cal_tgl = 0;
    wire mode, trig;

    cal_mode_cdc u_dut (
        .I_csi_clk  (csi_clk),
        .I_csi_rst_n(csi_rst_n),
        .I_cal_tgl  (cal_tgl),
        .O_mode     (mode),
        .O_trig     (trig)
    );

    // 顶层 CAL 接受条件（与 isp_top 的 FSM 同拍语义）
    wire cal_start = trig &  mode;       // 上升沿：启动标定
    wire cal_abort = trig & ~mode;       // 下降沿：中止标定/回自动

    integer errors = 0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors = errors + 1; end
        end
    endtask

    // 统计：每次 mode 变化必须恰好一个 trig
    integer mode_edges = 0, trig_cnt = 0;
    reg mode_d = 0, trig_d = 0;
    reg trig_without_edge = 0;
    always @(posedge csi_clk) begin
        trig_d <= trig;
        if (mode != mode_d) mode_edges <= mode_edges + 1;
        if (trig && (!trig_d)) trig_cnt <= trig_cnt + 1;
        // 一个 trig 拍结束后的下一拍，mode 必须已经稳定且与 trig 同拍出现过变化
        if (trig && (mode == mode_d)) trig_without_edge <= 1'b1;
        mode_d <= mode;
    end

    // 在 CSI 域采样同拍关系（每个 CSI 上升沿检查 if trig then start|abort）
    integer same_cycle_bad = 0;
    always @(posedge csi_clk) begin
        #0.1;
        if (trig && !(cal_start || cal_abort)) same_cycle_bad <= same_cycle_bad + 1;
    end

    integer k, found;
    reg [1:0] rise_seen, fall_seen;
    reg rise_start, rise_abort, fall_start, fall_abort;

    initial begin
        repeat (5) @(negedge sys_clk);
        csi_rst_n = 1;
        repeat (5) @(negedge sys_clk);
        sys_rst_n = 1;
        repeat (20) @(negedge csi_clk);
        check("reset: mode=0", mode == 1'b0);
        check("reset: trig=0", trig == 1'b0);

        // ---- 上升沿 ----
        @(negedge sys_clk);
        cal_tgl = 1'b1;
        found = 0; rise_seen = 0;
        for (k = 0; k < 40; k = k + 1) begin
            @(posedge csi_clk);
            #0.1;
            if (!found && mode === 1'b1) begin
                found = 1;
                rise_seen  = (trig === 1'b1) ? 2'd1 : 2'd2;
                rise_start = cal_start;
                rise_abort = cal_abort;
                $display("MODE_RISE at CSI edge %0d: mode=%b trig=%b start=%b abort=%b",
                         k, mode, trig, cal_start, cal_abort);
            end
        end
        check("rise: mode=1", found == 1);
        check("rise: trig aligned with mode", rise_seen == 2'd1);
        check("rise: cal_start", rise_start == 1'b1);
        check("rise: cal_abort not asserted", rise_abort == 1'b0);

        // 再等一拍确认 trig 只持续一个 CSI 周期
        @(posedge csi_clk); #0.1;
        check("rise: trig is single-cycle", trig === 1'b0);
        check("rise: mode stays 1", mode === 1'b1);

        // ---- 稳定电平 ----
        repeat (200) @(negedge csi_clk);
        check("stable high: no trig for 200 cycles", trig == 1'b0 && mode == 1'b1);

        // ---- 下降沿 ----
        @(negedge sys_clk);
        cal_tgl = 1'b0;
        found = 0; fall_seen = 0;
        for (k = 0; k < 40; k = k + 1) begin
            @(posedge csi_clk);
            #0.1;
            if (!found && mode === 1'b0) begin
                found = 1;
                fall_seen  = (trig === 1'b1) ? 2'd1 : 2'd2;
                fall_start = cal_start;
                fall_abort = cal_abort;
                $display("MODE_FALL at CSI edge %0d: mode=%b trig=%b start=%b abort=%b",
                         k, mode, trig, cal_start, cal_abort);
            end
        end
        check("fall: mode=0", found == 1);
        check("fall: trig aligned with mode", fall_seen == 2'd1);
        check("fall: cal_abort", fall_abort == 1'b1);
        check("fall: cal_start not asserted", fall_start == 1'b0);
        @(posedge csi_clk); #0.1;
        check("fall: trig is single-cycle", trig === 1'b0);

        repeat (100) @(negedge csi_clk);
        check("stable low: no trig", trig == 1'b0 && mode == 1'b0);

        // ---- 计数一致性 ----
        check("mode edges = 2", mode_edges == 2);
        check("trig pulses = 2", trig_cnt == 2);
        check("no trig without mode change", trig_without_edge == 1'b0);
        check("every trig has start or abort in same cycle", same_cycle_bad == 0);

        // ---- CSI 复位：源保持 0 ----
        csi_rst_n = 0;
        repeat (3) @(negedge csi_clk);
        check("csi reset: mode=0", mode == 1'b0);
        check("csi reset: trig=0", trig == 1'b0);
        csi_rst_n = 1;
        repeat (10) @(negedge csi_clk);
        check("reset release (src=0): no spurious trig", trig == 1'b0 && mode == 1'b0);

        $display("\n================================================");
        if (errors == 0) $display("RESULT: PASS (0 errors)");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end

endmodule
