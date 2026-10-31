// ==========================================================================
// ae_ctrl 闭环/限幅/holdoff 短回归
// --------------------------------------------------------------------------
// 不模拟完整 I2C：用行为模型模拟 uicfgcs520 的 O_ae_cfg_done 握手
// （收到 req 后拉低若干拍，再拉高）。验证：
//   1) 上电初值无条件下发（AE_INIT/AG_INIT）；
//   2) 偏暗：曝光到 AE_MAX 后提增益，增益不超过 AG_MAX；
//   3) 偏亮：先降增益到 AG_MIN，再缩曝光，曝光不低于 AE_MIN；
//   4) 死区：|lum-target| <= dead 不再调整；
//   5) holdoff：两次调整至少隔 AE_HOLDOFF 帧。
// 说明：I2C 字节级时序由 uicfgcs520/uii2c 负责，本 TB 只验证 AE 控制器。
// ==========================================================================
`timescale 1ns/1ps
module tb_ae_ctrl;
    reg clk = 0;
    always #21 clk = ~clk;       // ~23.8 MHz

    reg rst = 1;
    reg cam_done = 0;
    reg ae_done = 1;
    reg [7:0] lum = 0;
    reg lum_upd = 0;

    wire ae_req;
    wire [15:0] ae, ag;

    ae_ctrl #(
        .AE_INIT   (16'd1991),
        .AG_INIT   (16'd16),
        .AG_MAX    (16'd96),     // 温和提亮后的顶层参数
        .AE_TARGET (8'd80)
    ) dut (
        .I_clk(clk), .I_rst(rst),
        .I_cam_cfg_done(cam_done), .I_ae_cfg_done(ae_done),
        .I_lum(lum), .I_lum_upd(lum_upd),
        .O_ae_req(ae_req), .O_ae(ae), .O_ag(ag)
    );

    // uicfgcs520 行为模型：req 上升 -> cfg_done 拉低 ~10 拍 -> 拉高
    reg [4:0] hs_cnt;
    reg req_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ae_done <= 1'b1; hs_cnt <= 5'd0; req_d <= 1'b0;
        end
        else begin
            req_d <= ae_req;
            if (ae_req && !req_d) begin
                ae_done <= 1'b0; hs_cnt <= 5'd0;
            end
            else if (!ae_done) begin
                if (hs_cnt == 5'd10) ae_done <= 1'b1;
                else hs_cnt <= hs_cnt + 5'd1;
            end
        end
    end

    integer errors = 0;
    integer req_cnt = 0;
    integer max_ag = 0, min_ag = 16'hffff;
    integer max_ae = 0, min_ae = 16'hffff;
    always @(posedge clk) begin
        if (ae_req && !req_d) req_cnt = req_cnt + 1;
        if (ag > max_ag) max_ag = ag;
        if (ag < min_ag) min_ag = ag;
        if (ae > max_ae) max_ae = ae;
        if (ae < min_ae) min_ae = ae;
    end

    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    task frame;
        input [7:0] l;
        begin
            lum = l;
            @(negedge clk); lum_upd = 1'b1;
            @(negedge clk); lum_upd = 1'b0;
            repeat (60) @(negedge clk);
        end
    endtask

    integer i, r0;
    initial begin
        repeat (5) @(negedge clk);
        rst = 0;
        repeat (5) @(negedge clk);
        cam_done = 1;
        repeat (50) @(negedge clk);

        // 上电初值下发
        check("init: AE=1991", ae == 16'd1991);
        check("init: AG=16",   ag == 16'd16);
        check("init: request issued", req_cnt >= 1);
        repeat (40) @(negedge clk);
        check("init: handshake returns idle (req low)", ae_req == 1'b0);

        // ---- 偏暗：加增益到 AG_MAX ----
        r0 = req_cnt;
        for (i = 0; i < 60; i = i + 1) frame(8'd30);
        $display("DARK: AE=%0d AG=%0d reqs=%0d", ae, ag, req_cnt-r0);
        check("dark: exposure stays AE_MAX", ae == 16'd1991);
        check("dark: gain reaches AG_MAX", ag == 16'd96);
        check("dark: gain never exceeds AG_MAX", max_ag <= 96);
        check("dark: adjustments respect holdoff (<=16 in 60 frames)",
              (req_cnt - r0) <= 16);

        // ---- 偏亮：先降增益，再缩曝光 ----
        r0 = req_cnt;
        for (i = 0; i < 60; i = i + 1) frame(8'd200);
        $display("BRIGHT: AE=%0d AG=%0d reqs=%0d", ae, ag, req_cnt-r0);
        check("bright: gain at AG_MIN", ag == 16'd16);
        check("bright: exposure reduced", ae < 16'd1991);
        check("bright: exposure >= AE_MIN", ae >= 16'd16);
        check("bright: gain never below AG_MIN", min_ag >= 16);

        // ---- 死区：目标附近不再调整 ----
        r0 = req_cnt;
        for (i = 0; i < 20; i = i + 1) frame(8'd80);   // 与 AE_TARGET 相同
        $display("DEADZONE: AE=%0d AG=%0d reqs=%0d", ae, ag, req_cnt-r0);
        check("deadzone: no adjustment at target", (req_cnt - r0) == 0);

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
