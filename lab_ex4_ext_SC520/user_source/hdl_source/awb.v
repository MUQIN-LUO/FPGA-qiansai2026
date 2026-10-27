/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description:
The reference demo provided by Milianke is only used for learning.
We cannot ensure that the demo itself is free of bugs, so users
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan
@Time        :   2024/10
version:     :
@Description :   在原基础上删去了一些赘余的代码，并且优化了时序，加入BRAM IP核，
                 让从除法器出来的每一个数据都可以对上相应的像素数据进行增益，使
                 得它的适用行更加广泛了。

                 本工程改动：原设计每拍都用两个除法器实时求 G/R、G/B（灰度世界），
                 问题有两个 ——
                   1) 增益逐拍变化，画面颜色抖动/闪烁；
                   2) 背景大面积有色时把目标颜色一起拉偏，黄色被洗淡。
                 现改为固定增益的白平衡（乘法器退化成常数移位加法），
                 延迟线、valid/tuser 时序完全不变，颜色确定且可复现。
*****************************************************************/
module awb  #(
    parameter IMG_HEIGHT = 1080,   // 图像高度参数，默认值为1080
    parameter IMG_WIDTH  = 1920,   // 图像宽度参数，默认值为1920
    parameter [19:0] INIT_GAIN_R = 20'd76824,    // 上电默认 R 增益（Q16）
    parameter [19:0] INIT_GAIN_B = 20'd138731    // 上电默认 B 增益（Q16）
)
(
    input                   I_clk  ,    // 输入时钟信号
    input                   I_rst_n,    // 输入复位信号，低电平有效

    input                   I_tlast  ,  // 行结束信号
    input                   I_tuser  ,  // 帧开始信号
    input [95:0]            I_tdata  ,  // 输入数据，总共包含4个像素的RGB值，每个像素24位，4*24=96位
    input                   I_tvalid ,  // 输入数据有效信号
    output                  I_tready ,  // 输入数据准备好信号

    output                  O_tlast  ,  // 输出行结束信号
    output                  O_tuser  ,  // 输出帧开始信号
    output [95:0]           O_tdata  ,  // 输出数据
    output                  O_tvalid ,  // 输出数据有效信号
    input                   O_tready  , // 输出数据准备好信号

    // 白平衡增益（Q16 定点，65536 = 1.00x），G 通道恒为 1.0 作基准。
    // 由 isp_top 里的白点估计器每帧算出；上电初值是实验室标定的常数，
    // 所以即使估计器一次都没收敛（镜头盖着、或一直对着单色物体），
    // 画面颜色依然是对的。
    input      [19:0]       I_gain_r  ,
    input      [19:0]       I_gain_b,
    output reg [23:0] O_meas_data,// Pre-gain sample, aligned with O_tdata[23:0]

    // 实际生效的增益对 + “刚在帧边界整对锁存”的单拍事件
    output     [19:0] O_gain_r_ap,
    output     [19:0] O_gain_b_ap,
    output reg        O_gain_evt,

    // 提交许可 + 确认。只有 (帧首像素 && commit_en) 那一拍才整对锁存请求增益，
    // 并打一拍 O_gain_evt 作为"这一对已经生效"的确认（数值相同也确认）。
    input              I_gain_commit_en
);

/*****************************************************************
                          中间信号
*****************************************************************/

  reg         I_tvalid_r0,I_tvalid_r1;			// 输入有效信号的寄存
  reg  [95:0] I_tdata_d0,I_tdata_d1,I_tdata_d2; // 输入数据的寄存

  wire  [95:0]        data_d0 ;		// 延时后的数据
  wire                valid_d0;     // 延时后的有效信号
  wire                I_tuser_r0;   // 延时后的帧开始信号

  reg                 I_tuser_r1;   // 帧开始信号的寄存

  reg   [95:0]        data_d1 ;     // 再次延时后的数据
  reg                 valid_d1;     // 再次延时后的有效信号

  /*****************************************************************
                            输入流水线
  *****************************************************************/
  // 注意：d0 这一级即使数值已无下游使用（原用于统计），也必须保留，
  //       否则 I_tdata_d1 会提前一拍，整条通路的延迟对齐就变了。
  always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        I_tdata_d0  <= 0;
        I_tdata_d1  <= 0;
        I_tdata_d2  <= 0;
        {I_tvalid_r0,I_tvalid_r1} <= 0;
    end else begin
        I_tdata_d0  <= I_tdata   ;
        I_tdata_d1  <= I_tdata_d0;
        I_tdata_d2  <= I_tdata_d1;
        I_tvalid_r0 <= I_tvalid;
        I_tvalid_r1 <= I_tvalid_r0;
    end
  end

  /*****************************************************************
                          固定白平衡增益
  *****************************************************************/
  // ==============================================================
  // Q16 定点：65536 = 1.00x，G 通道固定为 1.00x 作参考。
  //
  // 标定方法（不需要色卡）：
  //   1. 在演示灯光下，把一张白纸/灰卡放到画面正中央的 64x64 标定框里；
  //   2. OSD 第三、四行会显示该框内 R/G/B 的平均值（0~255 满量程）；
  //   3. 记下读数 R, G, B，则
  //         WB_GAIN_R = 65536 * G / R
  //         WB_GAIN_B = 65536 * G / B
  //   4. 把算出来的两个数填回下面，重新综合即可。
  //
  // 为什么要标定而不是自适应：自适应（灰度世界）每帧都在改增益，画面会闪，
  // 而且背景大面积有色时会把目标颜色一起拉偏。固定增益一次标定、永久确定，
  // 演示时可复现，也不会和目标检测抢通道。
  // ==============================================================
  // 增益由外部端口驱动（白点估计器给出），上电默认值在 isp_top 里设置：
  //   实测标定值（2026-10-24，室内 LED，A4 白纸填满中心 64x64 框）
  //     标定时增益 1.00x，OSD 读数 R177 G218 B118（两次一致性 <1%）
  //     R = 65536 x 218 / 177 = 80717   (1.2316x)
  //     B = 65536 x 218 / 118 = 121075  (1.8475x)

	// 信号延迟模块，用于同步信号，延迟50个时钟周期
    signal_delay #(
        .IMG_HEIGHT   (IMG_HEIGHT),
        .IMG_WIDTH    (IMG_WIDTH ),
        .DATA_WIDTH   (96  ),	// 数据位宽为96位
        .DELAY_CYCLE  (50)      // 延迟50个时钟周期
    )signal_delay_d (
         .I_clk  (I_clk  ),     // 输入时钟信号
         .I_rst_n(I_rst_n),     // 输入复位信号
         .I_tuser(I_tuser),  	// 输入帧开始信号
         .I_valid(I_tvalid_r1), // 输入数据有效信号
         .I_data (I_tdata_d1 ), // 输入数据
         .O_valid(valid_d0   ), // 输出数据有效信号
         .O_data (data_d0    ), // 输出数据
         .O_tuser(I_tuser_r0 )  // 输出帧开始信号
    );

    // 信号同步，延迟一级
    always @(posedge I_clk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          valid_d1 <= 0;
          data_d1  <= 0;
          I_tuser_r1 <= 0;
      end else begin
          valid_d1 <= valid_d0;
          data_d1  <= data_d0 ;
          I_tuser_r1 <= I_tuser_r0;
      end
    end

/*****************************************************************
         ★ 增益的“整对原子提交点”（审查项 D）

  之前 isp_top 拿“下游检测流水给出的帧尾”当安全边界，那是错的：
  检测挂在 AWB 之后，等它看到帧尾时，AWB 早已把下一帧的头几个像素
  用旧增益乘完了 —— 同一帧里就混了两代增益。

  这里换成真正的边界：signal_delay 输出的 I_tuser_r0 与 data_d0 同拍，
  valid_d1/data_d1 比它晚一拍，因此

      cu_fpx = valid_d1 && I_tuser_r1   // 本拍乘进乘加器的正是新帧首像素

  恰好在“新帧首像素”这一拍用 I_gain_* 的当前值（组合 MUX），
  同一拍把 g_*_cur 更新成这一对；本帧其余像素一律用 g_*_cur。于是
      · 上一帧最后一个像素 -> 旧增益（此时 cu_fpx = 0）
      · 新帧第一个像素     -> 新增益（cu_fpx = 1，直接取 I_gain_*）
      · 新帧其余像素       -> g_*_cur = 新增益
  与 tvalid 是否连续、有没有消隐都无关。
*****************************************************************/
    reg  [19:0] g_r_cur, g_b_cur;
    // signal_delay 的 dob 是"组合读 + 地址已寄存"，因此 O_data 比 O_valid
    // 领先一拍：O_tuser 拉高的那一拍，乘加器实际拿到的还不是帧首像素。
    // 实测（tb_awb_atomic）增益跳变落在上一帧最后一拍，正是这一拍之差。
    // 这里把标记再打一拍，与"真正把帧首像素送进乘加器"的那一拍对齐。
    reg         tuser_g;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) tuser_g <= 1'b0;
        else          tuser_g <= I_tuser_r1;
    end
    wire        cu_fpx   = valid_d1 & tuser_g;              // 帧首像素
    wire        commit_px = cu_fpx & I_gain_commit_en;      // 真正允许提交的那一拍
    // ★ 首像素的 MUX 也必须用 commit_px：
    //   若用 cu_fpx，en=0 时首像素会偷用还没被批准的请求值。
    wire [19:0] g_r_eff = commit_px ? I_gain_r : g_r_cur;
    wire [19:0] g_b_eff = commit_px ? I_gain_b : g_b_cur;

    assign O_gain_r_ap = g_r_cur;
    assign O_gain_b_ap = g_b_cur;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            g_r_cur    <= INIT_GAIN_R;
            g_b_cur    <= INIT_GAIN_B;
            O_gain_evt <= 1'b0;
        end else begin
            O_gain_evt <= 1'b0;
            if (commit_px) begin
                g_r_cur <= I_gain_r;
                g_b_cur <= I_gain_b;
                // 无条件打一拍确认：与增益值有没有变化无关，
                // 所以"重新标定到同一对增益"也能被确认，不会假超时。
                O_gain_evt <= 1'b1;
            end
        end
    end

/*****************************************************************
                           像素值计算
*****************************************************************/

    reg [27:0] R_new_r1;
    reg [27:0] G_new_r1;
    reg [27:0] B_new_r1;

    reg [27:0] R_new_r2;
    reg [27:0] G_new_r2;
    reg [27:0] B_new_r2;

    reg [27:0] R_new_r3;
    reg [27:0] G_new_r3;
    reg [27:0] B_new_r3;

    reg [27:0] R_new_r4;
    reg [27:0] G_new_r4;
    reg [27:0] B_new_r4;


    wire   [7:0] rgb888_r_d1[0:3];
    wire   [7:0] rgb888_g_d1[0:3];
    wire   [7:0] rgb888_b_d1[0:3];

    assign  rgb888_r_d1[0] = data_d1[16+:8];
    assign  rgb888_g_d1[0] = data_d1[8+:8];
    assign  rgb888_b_d1[0] = data_d1[0+:8];

    assign  rgb888_r_d1[1] = data_d1[40+:8];
    assign  rgb888_g_d1[1] = data_d1[32+:8];
    assign  rgb888_b_d1[1] = data_d1[24+:8];

    assign  rgb888_r_d1[2] = data_d1[64+:8];
    assign  rgb888_g_d1[2] = data_d1[56+:8];
    assign  rgb888_b_d1[2] = data_d1[48+:8];

    assign  rgb888_r_d1[3] = data_d1[88+:8];
    assign  rgb888_g_d1[3] = data_d1[80+:8];
    assign  rgb888_b_d1[3] = data_d1[72+:8];

	// 计算新的RGB值，应用增益（增益为常数，综合后是多级移位加法，不占 DSP）
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            O_meas_data <= 24'd0;
            R_new_r1 <= 0;
            G_new_r1 <= 0;
            B_new_r1 <= 0;

            R_new_r2 <= 0;
            G_new_r2 <= 0;
            B_new_r2 <= 0;

            R_new_r3 <= 0;
            G_new_r3 <= 0;
            B_new_r3 <= 0;

            R_new_r4 <= 0;
            G_new_r4 <= 0;
            B_new_r4 <= 0;
        end
        else if(valid_d1) begin
            O_meas_data <= data_d1[23:0];
            R_new_r1 <= g_r_eff * rgb888_r_d1[0];
            G_new_r1 <=           rgb888_g_d1[0];
            B_new_r1 <= g_b_eff * rgb888_b_d1[0];

            R_new_r2 <= g_r_eff * rgb888_r_d1[1];
            G_new_r2 <=           rgb888_g_d1[1];
            B_new_r2 <= g_b_eff * rgb888_b_d1[1];

            R_new_r3 <= g_r_eff * rgb888_r_d1[2];
            G_new_r3 <=           rgb888_g_d1[2];
            B_new_r3 <= g_b_eff * rgb888_b_d1[2];

            R_new_r4 <= g_r_eff * rgb888_r_d1[3];
            G_new_r4 <=           rgb888_g_d1[3];
            B_new_r4 <= g_b_eff * rgb888_b_d1[3];
        end
    end

/*****************************************************************
                            像素值截断
*****************************************************************/

    wire [7:0] R_new_x1;
    wire [7:0] G_new_x1;
    wire [7:0] B_new_x1;

    wire [7:0] R_new_x2;
    wire [7:0] G_new_x2;
    wire [7:0] B_new_x2;

    wire [7:0] R_new_x3;
    wire [7:0] G_new_x3;
    wire [7:0] B_new_x3;

    wire [7:0] R_new_x4;
    wire [7:0] G_new_x4;
    wire [7:0] B_new_x4;

	// 对新的像素值进行截断，防止溢出
    assign R_new_x1 = (R_new_r1[27:24])?255:R_new_r1[23:16];
    assign G_new_x1 =  G_new_r1;
    assign B_new_x1 = (B_new_r1[27:24])?255:B_new_r1[23:16];

    assign R_new_x2 = (R_new_r2[27:24])?255:R_new_r2[23:16];
    assign G_new_x2 =  G_new_r2;
    assign B_new_x2 = (B_new_r2[27:24])?255:B_new_r2[23:16];

    assign R_new_x3 = (R_new_r3[27:24])?255:R_new_r3[23:16];
    assign G_new_x3 =  G_new_r3;
    assign B_new_x3 = (B_new_r3[27:24])?255:B_new_r3[23:16];

    assign R_new_x4 = (R_new_r4[27:24])?255:R_new_r4[23:16];
    assign G_new_x4 =  G_new_r4;
    assign B_new_x4 = (B_new_r4[27:24])?255:B_new_r4[23:16];
/*****************************************************************
                          同步信号
*****************************************************************/

    reg valid_d2,I_tuser_r2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            valid_d2 <= 0;
            I_tuser_r2 <= 0;
        end
        else begin
            valid_d2 <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
        end
    end

	// 输出信号的生成
    assign  O_tlast  = (valid_d1 == 0) && (valid_d2 == 1);   // 行结束信号
    // 帧起始信号【必须直通】，不能延时！
    //   数据经 signal_delay 延迟了 50 拍，但下游 video_in 是按"tuser 不延时、
    //   与延迟后的数据错开 50 拍"这个时序对齐的（原厂 hdl_source/awb.v 就是
    //    assign O_tuser = I_tuser）。这里若改成 I_tuser_r2，帧起点会晚 52 拍，
    //   表现为画面整体横向位移。
    assign  O_tuser  = I_tuser    ;                          // 帧开始信号
    assign  O_tvalid = valid_d2;                             // 输出有效信号

	// 输出数据，由四个像素的数据拼接而成
    assign  O_tdata = O_tvalid?{R_new_x4,G_new_x4,B_new_x4,
                       R_new_x3,G_new_x3,B_new_x3,
                       R_new_x2,G_new_x2,B_new_x2,
                       R_new_x1,G_new_x1,B_new_x1}:0;
	// 输入准备好信号，直接连接到输出准备好信号
    assign  I_tready = O_tready;

endmodule
