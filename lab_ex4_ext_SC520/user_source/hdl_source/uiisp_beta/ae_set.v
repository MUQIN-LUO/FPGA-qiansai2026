`timescale 1ns / 1ps

/*********************************************************************
 * 文件名    : ae_set.v
 * 模块名    : ae_ctrl
 * ------------------------------------------------------------------
 * 本文件原本是原厂的 ae_set（4 个按键手动加减曝光/增益）。本工程把按键
 * 改作算法切换后，它被固定喂 4'b1111，AE 从此长期停在「曝光 1991（最大）
 * + 模拟增益 16x」，光线一变就完全失去调节能力。
 *
 * 现改为纯硬件闭环，模块名也一并改成 ae_ctrl 以免误解。
 * 模块名改了但不新建文件 —— TD 工程按【文件路径】登记源码，复用本文件
 * 可以避免新文件忘记 Add Source 而被当成黑盒。
 *
 * ------------------------------------------------------------------
 * 控制结构
 *
 *   ISP 每帧输出全帧 G 通道均值 I_lum（0~224，满量程≈全白）
 *        │  （O_lum_tgl 电平翻转跨到 24MHz 域）
 *        ▼
 *   与 AE_TARGET 比较 → 死区 → 乘性步进调整曝光 / 增益
 *        │
 *        ▼
 *   O_ae_req / O_ae / O_ag  →  uicfgcs520（组保持，整组原子生效）
 *
 * ------------------------------------------------------------------
 * 几个刻意的设计选择，以及为什么
 *
 *  1) 测光只用 G 通道
 *     G 的白平衡增益按定义恒为 1.0，所以 AWB 无论怎么调都不会影响 AE
 *     的输入。AE 与 AWB 是两个闭环，这样就从结构上切断了互相激励导致
 *     的振荡 —— 只剩单向耦合（AE 改亮度会改变 AWB 的统计窗口），
 *     而那是可以靠 AWB 只在"未饱和帧"上更新来处理的。
 *
 *  2) 乘性步进，不用乘法器也不用除法器
 *     步长取当前曝光值的 1/4（误差大）或 1/16（误差小），都是右移。
 *     曝光与亮度近似成正比，所以乘性步进 = 亮度的几何收敛，不会过冲
 *     振荡；而且误差越大步长越大，收敛快。
 *
 *  3) 先动曝光，曝光打头再动增益
 *     曝光时间长 = 收集的光子多 = 信噪比好；模拟增益是"事后放大"，
 *     噪声一起放大。所以只要曝光还有余量就不碰增益。
 *     不足时：曝光加到 AE_MAX，再提增益；
 *     过量时：先降增益到 AG_MIN，再缩短曝光。
 *
 *  4) 两次调整之间强制隔 AE_HOLDOFF 帧
 *     传感器改曝光后要 1~2 帧才体现到画面上，60Hz 连续追会把噪声
 *     当成误差来回抖。
 *
 *  5) 上电后无条件先下发一次初值
 *     初始化表把增益写成 1x（0x3e09=0x00/0x3e07=0x80），不主动下发
 *     初值的话画面会一直很暗，要等 AE 从最暗一路爬上来。
 *
 * ------------------------------------------------------------------
 * 量程（来自原厂注释，勿随意改）
 *   720p60 模式 VTS = 1000 行，曝光以【半行】为单位，
 *   最大安全曝光 = 2*VTS - 9 = 1991。
 *   增益索引 16..176 由 SC520GainTbl 映射为 1x..32x 模拟 + 数字细调。
 *********************************************************************/
module ae_ctrl #(
    parameter [15:0] AE_INIT    = 16'd1990,  // 上电初值，与原厂默认一致，避免画面跳变
    parameter [15:0] AE_MIN     = 16'd16,
    parameter [15:0] AE_MAX     = 16'd1991,  // = 2*VTS - 9，VTS=1000
    parameter [15:0] AG_INIT    = 16'd16 ,   // Start at unity gain; increase only when exposure is exhausted
    parameter [15:0] AG_MIN     = 16'd16,    // 1x
    parameter [15:0] AG_MAX     = 16'd80 ,   // 4x total gain cap
    parameter [7:0]  AE_TARGET  = 8'd96,     // 目标亮度（I_lum 单位，满量程 224）
    parameter [7:0]  AE_DEAD    = 8'd6,      // 死区，防抖
    parameter [7:0]  AE_HOLDOFF = 8'd4       // 两次调整之间至少隔几帧
)(
    input  wire        I_clk,           // 24 MHz（与 uicfgcs520 同域）
    input  wire        I_rst,           // 高有效复位
    input  wire        I_cam_cfg_done,  // 传感器初始化完成
    input  wire        I_ae_cfg_done,   // 上一组 AE 写入完成
    input  wire [7:0]  I_lum,           // 全帧 G 均值，0~224
    input  wire        I_lum_upd,       // 每帧一个脉冲
    output reg         O_ae_req,
    output reg  [15:0] O_ae,
    output reg  [15:0] O_ag
);

    // ------------------------------------------------------------------
    // 误差、死区、步长
    // ------------------------------------------------------------------
    wire        too_dark = (I_lum < AE_TARGET);
    wire [7:0]  diff     = too_dark ? (AE_TARGET - I_lum) : (I_lum - AE_TARGET);
    wire        in_dead  = (diff <= AE_DEAD);
    wire        big_err  = (diff > 8'd24);

    // 曝光乘性步长：1/4 或 1/32，最小 1，防止小曝光值时步长归零卡死
    //   （细调步长必须小于死区对应的相对量，否则会在死区边缘来回跳）
    wire [15:0] ae_step   = big_err ? (O_ae >> 2) : (O_ae >> 5);
    wire [15:0] ae_step_m = (ae_step == 16'd0) ? 16'd1 : ae_step;

    // 增益步长：索引单位。32 个索引 = 一个倍频程。
    // 大步长由 16 收到 8（半档），配合 4 帧 holdoff 让亮度过渡更平滑，
    // 避免低照度下整档跳变造成可见闪烁；小步长 2 用于死区边缘细调。
    wire [15:0] ag_step   = big_err ? 16'd8 : 16'd2;

    // ------------------------------------------------------------------
    // 计算下一组曝光/增益
    // ------------------------------------------------------------------
    reg [15:0] n_ae, n_ag;
    always @(*) begin
        n_ae = O_ae;
        n_ag = O_ag;

        if (!in_dead) begin
            if (too_dark) begin
                // 偏暗：先加曝光，曝光到顶再提增益
                if (O_ae < AE_MAX) begin
                    n_ae = ((O_ae + ae_step_m) > AE_MAX) ? AE_MAX : (O_ae + ae_step_m);
                end
                else if (O_ag < AG_MAX) begin
                    n_ag = ((O_ag + ag_step) > AG_MAX) ? AG_MAX : (O_ag + ag_step);
                end
            end
            else begin
                // Bright: remove excess gain before shortening exposure.
                // This avoids settling at short exposure and amplified noise.
                if (O_ag > AG_MIN) begin
                    n_ag = (O_ag < (AG_MIN + ag_step)) ? AG_MIN : (O_ag - ag_step);
                end
                else if (O_ae > AE_MIN) begin
                    n_ae = (O_ae < (AE_MIN + ae_step_m)) ? AE_MIN : (O_ae - ae_step_m);
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 握手状态机
    //   O_ae_req 采用【电平保持】而不是单拍脉冲：
    //   uicfgcs520 只在 ST_IDLE 里采样 I_ae_req，它可能正忙于别的写操作，
    //   单拍脉冲会被漏掉。保持到看见 O_ae_cfg_done 被拉低（序列已启动）
    //   再撤销，才可靠。
    // ------------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0,   // 空闲，等下一帧测光
                     S_REQ   = 2'd1,   // 拉高请求，等序列启动
                     S_WAIT  = 2'd2;   // 等序列完成

    reg [1:0]  state;
    reg [7:0]  hold;
    reg        init_done;
    reg [19:0] tmo;

    wire can_issue = I_cam_cfg_done && I_ae_cfg_done;
    wire need_adj  = ((n_ae != O_ae) || (n_ag != O_ag));

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            state     <= S_IDLE;
            O_ae_req  <= 1'b0;
            O_ae      <= AE_INIT;
            O_ag      <= AG_INIT;
            hold      <= 8'd0;
            init_done <= 1'b0;
            tmo       <= 20'd0;
        end
        else begin
            // hold 按【帧】递减，不是按时钟。
            // 传感器改曝光后要 1~2 帧才体现到画面上，如果按时钟计数（4 拍
            // 只有 0.17us），下一次调整会在画面还没更新时就拿旧读数再算一遍，
            // 变成过冲。按帧计数才能真正等到新画面。
            if (I_lum_upd && (hold != 8'd0))
                hold <= hold - 8'd1;

            case (state)
                S_IDLE: begin
                    O_ae_req <= 1'b0;

                    // 上电首次：无条件下发一次初值
                    if (!init_done && can_issue) begin
                        O_ae      <= AE_INIT;
                        O_ag      <= AG_INIT;
                        hold      <= AE_HOLDOFF;
                        O_ae_req  <= 1'b1;
                        init_done <= 1'b1;
                        tmo       <= 20'd0;
                        state     <= S_REQ;
                    end
                    // 之后：每帧测光更新一次，且受 hold-off 限制
                    else if (init_done && can_issue && I_lum_upd &&
                             (hold == 8'd0) && need_adj) begin
                        O_ae     <= n_ae;
                        O_ag     <= n_ag;
                        O_ae_req <= 1'b1;
                        hold     <= AE_HOLDOFF;
                        tmo      <= 20'd0;
                        state    <= S_REQ;
                    end
                end

                S_REQ: begin
                    // 保持请求直到看到 cfg_done 被拉低（序列已启动）
                    if (!I_ae_cfg_done) begin
                        O_ae_req <= 1'b0;
                        tmo      <= 20'd0;
                        state    <= S_WAIT;
                    end
                    else if (tmo >= 20'hF_FFFF) begin
                        // 超时兜底：请求没被接受也不能把整个 AE 卡死
                        O_ae_req <= 1'b0;
                        state    <= S_IDLE;
                    end
                    else begin
                        tmo <= tmo + 20'd1;
                    end
                end

                S_WAIT: begin
                    if (I_ae_cfg_done) begin
                        state <= S_IDLE;
                    end
                    else if (tmo >= 20'hF_FFFF) begin
                        state <= S_IDLE;
                    end
                    else begin
                        tmo <= tmo + 20'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
