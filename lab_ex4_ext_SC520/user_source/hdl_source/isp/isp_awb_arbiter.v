// ==========================================================================
// AWB 增益仲裁/提交：请求增益的唯一 owner + 帧首整对提交许可
// --------------------------------------------------------------------------
// 设计与回归测试实例化同一份本模块。
//
// 背景（本轮板测根因）：
//   awb 只在“新帧首像素 && I_gain_commit_en”这一拍把请求增益整对锁存。
//   原 isp_top 的 I_gain_commit_en 只在 CAL_SHDW 打开；默认 AWB_MODE=0 时
//   灰度世界每帧更新请求 S_wbr/S_wbb，却从未给出提交许可 —— 实际生效的
//   增益永远停在上电 INIT（R 1.17x / B 2.12x），表现为画面长期偏蓝、
//   OSD 的 R/B 长期不变。长按 K1 进入 CAL_SHDW 只是让积压的自动请求
//   首次提交，并不是“白纸标定被应用/锁定”。
//
// 本模块：
//   · AWB_MODE=0（灰度世界）：gw_commit 更新请求，并置 pending；
//     commit_en 保持到 awb 回 gain_evt（完成帧首整对锁存）后撤销。
//   · AWB_MODE=1（白纸一键标定）：cal_commit 更新请求；
//     commit_en 直接来自 CAL_SHDW 窗口。
//   · 复位后生效增益 = INIT（awb 自身的 INIT 同值）。
// ==========================================================================
module isp_awb_arbiter #(
    parameter [19:0] INIT_R = 20'd76824,
    parameter [19:0] INIT_B = 20'd138731
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_awb_mode,       // 0 = 灰度世界（默认），1 = 白纸标定

    // 灰度世界请求 + 单拍提交脉冲
    input  wire        I_gw_commit,
    input  wire [19:0] I_gw_r,
    input  wire [19:0] I_gw_b,

    // 白纸标定请求 + 单拍提交脉冲
    input  wire        I_cal_commit,
    input  wire [19:0] I_cal_r,
    input  wire [19:0] I_cal_b,

    // CAL_SHDW 的提交窗口（仅 AWB_MODE=1 使用）
    input  wire        I_cal_commit_en,

    // awb 在帧首整对锁存后的确认脉冲
    input  wire        I_gain_evt,

    output reg  [19:0] O_wr,
    output reg  [19:0] O_wb,
    output wire        O_commit_en
);

    // 灰度世界提交 pending：从 O_commit 起保持到下个帧首 awb 确认
    reg gw_pend;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gw_pend <= 1'b0;
        end
        else begin
            if (I_awb_mode != 1'b0)
                gw_pend <= 1'b0;
            else if (I_gw_commit)
                gw_pend <= 1'b1;
            else if (I_gain_evt)
                gw_pend <= 1'b0;
        end
    end

    // 请求增益的唯一 owner
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_wr <= INIT_R;
            O_wb <= INIT_B;
        end
        else if (I_awb_mode == 1'b0) begin
            if (I_gw_commit) begin
                O_wr <= I_gw_r;
                O_wb <= I_gw_b;
            end
        end
        else begin
            if (I_cal_commit) begin
                O_wr <= I_cal_r;
                O_wb <= I_cal_b;
            end
        end
    end

    assign O_commit_en = (I_awb_mode == 1'b0) ? gw_pend : I_cal_commit_en;

endmodule
