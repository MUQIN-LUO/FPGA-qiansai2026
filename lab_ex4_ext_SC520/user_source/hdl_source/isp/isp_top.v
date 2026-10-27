module isp_top #(
    // 标定墙钟超时（CSI 时钟拍数）。默认 5e8 ≈ 4.8 s @103.5 MHz，
    // 与原来完全一致；做成 parameter 只是为了仿真能缩短，综合默认值不变。
    parameter [29:0] CAL_TMO = 30'd500_000_000,
    // 只作用在 CAL_SHDW 的"等提交确认"窗口。生产默认 = CAL_TMO，行为不变；
    // 拆出来是为了能构造 deadline 边界测试（ACC 需要 4 个完整帧 ≈ 1.1e6 拍，
    // 用同一个定时器根本没法把 commit 卡在临近到期的位置）。
    parameter [29:0] CAL_COMMIT_TMO = 30'd500_000_000
) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input [1:0] 	I_mode			,   // 0=原图 1=边缘 2=灰度 3=波普
    input [1:0] 	I_pal			,   // 波普调色板编号
    output reg [11:0] O_det_minx	,   // 黄色目标外接框（图像坐标）
    output reg [11:0] O_det_maxx	,
    output reg [11:0] O_det_miny	,
    output reg [11:0] O_det_maxy	,
    output reg        O_det_valid	,   // 颜色+面积+形状 全部通过
    output reg        O_det_cand	,   // 颜色+面积通过（形状未过，供调参观察）
    output reg [9:0]  O_det_cnt		,   // 命中像素数 / 16（调试显示用）
    output reg [7:0]  O_rgb_r		,   // 中心 64x64 区域平均色（白平衡标定 / 验证用）
    output reg [7:0]  O_rgb_g		,
    output reg [7:0]  O_rgb_b		,
    output reg [7:0]  O_lum			,   // 全帧 G 通道均值（AE 测光），满量程 224
    output reg        O_lum_tgl		,   // 每帧翻转一次，供跨时钟域握手
    output wire [9:0] O_gr100		,   // 当前 R 增益 x100（OSD 显示用，如 116 = 1.16x）
    output wire [9:0] O_gb100		,   // 当前 B 增益 x100
    input  wire       I_cal_trig	,   // 一键标定请求（CSI 域，单拍电平事件）
    output reg  [1:0] O_cal_st		,   // 0=空闲 1=BUSY 2=OK 3=ERR
    output reg        O_frm_tgl		,   // 每帧尾翻转一次（供整组快照握手）
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep

  // 白平衡增益（Q16，G 恒为 1.0 基准）。本体在文件后半部分的白点估计器里，
  // 但这里必须先声明 —— 下面的 awb 实例化马上要用；若等本体处才声明，
  // Verilog 会先在端口连接处建一个隐式 1 位线网，和后面的 reg [19:0] 冲突。
  // ★ 唯一 owner：S_wbr / S_wbb 只在"标定 FSM"那一个 always 块里写，
  //   别处一律只读。旧动态 AWB 估计器已整体删除（本轮不用连续自适应），
  //   否则两个 always 同时写会造成多驱动（上次综合就是死在 HDL-8007）。
  reg  [19:0] S_wbr;       // 请求 R 增益（Q16）
  reg  [19:0] S_wbb;       // 请求 B 增益（Q16）

  // awb 内部真正生效的增益（它在自己首像素处整对锁存）与锁存事件
  wire [19:0] S_gain_r_ap;
  wire [19:0] S_gain_b_ap;
  wire        S_gain_evt;
  wire        S_gain_commit_en;   // 提交许可（定义在标定段，先声明供端口连接）

  // AWB 输出流。同样必须先声明：下面 awb 实例化的端口连接处就要用到，
  // 否则会先建出隐式线网（TD 会报 HDL-1007，且数据总线可能被截成 1 位）。
  wire [ 95:0] S_awb_data;
  wire         S_awb_valid, S_awb_tlast, S_awb_tuser;
  wire [23:0] S_awb_meas;

  wire         S_rgb_tlast;   // 先声明：第 79 行 O_tlast 就要用
  wire         awb_O_tready;  //synthesis keep

  wire         awb_O_tuser_128;  //synthesis keep
  wire         awb_O_tvalid_128;  //synthesis keep
  wire [127:0] awb_O_tdata_128;  //synthesis keep

  // 边缘检测输出（96bit 流）
  wire         edge_O_tlast;     //synthesis keep
  wire         edge_O_tuser;     //synthesis keep
  wire [ 95:0] edge_O_tdata;     //synthesis keep
  wire         edge_O_tvalid;    //synthesis keep
  wire         edge_O_tready;    //synthesis keep

  assign awb_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = S_rgb_tlast    ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
// ==================================================================
// 黑电平校正（BLC）
// ------------------------------------------------------------------
// 传感器输出的"黑"不是 0，而是有个固定基座。SC 系列 RAW10 的典型值
// 是 **64**（10bit 满量程 1023，约 6%）。这个基座若不去掉：
//   1) 暗部发灰 —— gamma 之后尤其明显：
//        gamma(16/255)=gamma(0.063)≈0.27 -> "纯黑"显示成中灰 69/255
//   2) 白平衡增益是按通道乘的，基座也被各自放大（B 乘得最多），
//      于是暗部会按增益比例偏色
//
// 位置：demosaic 之前、10bit 原始数据上扣 —— 必须在 >>2 转 8bit 之前，
// 否则精度已经丢了。
// 纯组合、不加流水，demosaic 的输入延迟和时序完全不变。
//
// 调参：BLC_LEVEL 是 10bit 域的值。
//   暗部还发灰 -> 调大；  暗部死黑、细节丢失 -> 调小。
// ==================================================================
// 【实测值，不要再猜】
//   方法：AWB 增益已知、BLC 关闭、盖住镜头读中心平均值 -> R018 G015 B020
//   反推：G 基座 = 15，R 基座 = 18/1.2316 = 14.6，B 基座 = 20/1.8475 = 10.8
//   取 8bit 约 15 -> 10bit 约 60。
//
// 为什么必须扣：
//   暗部的基座会被白平衡增益【按通道】放大。B 增益 1.85x 意味着
//   B 通道的"黑"被抬到 15x1.85=28，而 G 仍是 15 —— 暗部整体泛紫。
//   实测症状：暗场画面 3 通道本来是 15 左右，输出却变成 R18 G15 B20，
//   看起来就是一层紫雾。
localparam [9:0] BLC_LEVEL = 10'd60;

wire [39:0] S_blc_tdata;
assign S_blc_tdata[ 9: 0] = (I_tdata[ 9: 0] > BLC_LEVEL) ? (I_tdata[ 9: 0] - BLC_LEVEL) : 10'd0;
assign S_blc_tdata[19:10] = (I_tdata[19:10] > BLC_LEVEL) ? (I_tdata[19:10] - BLC_LEVEL) : 10'd0;
assign S_blc_tdata[29:20] = (I_tdata[29:20] > BLC_LEVEL) ? (I_tdata[29:20] - BLC_LEVEL) : 10'd0;
assign S_blc_tdata[39:30] = (I_tdata[39:30] > BLC_LEVEL) ? (I_tdata[39:30] - BLC_LEVEL) : 10'd0;

demosaic #(
    .IMG_HEIGHT          (720 ),  // 图像高度
    .IMG_WIDTH           (1280),  // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")   // 实测：teal 背包在 RGGB 下变橄榄绿（R/B 互换），判定 BGGR 正确
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(S_blc_tdata),  // AXI4-Stream视频数据（已扣黑电平）
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
// ==================================================================
// 色度降噪：水平 3 点中值，作用在色差平面 R-G / B-G
// ------------------------------------------------------------------
// 【噪声从哪来】
//   去马赛克时 R、B 是插值"猜"出来的，输入带噪声就会猜错，猜错就是
//   彩色斑点；接着被白平衡的 B 增益(1.97x)放大一遍，再被 gamma 放大
//   一遍 —— 三层叠加，所以特别刺眼。
//
// 【为什么在色差平面做】
//   R-G 和 B-G 承载的是"颜色"，G 本身承载亮度。只对色差取中值，
//   亮度细节（纹理、边缘锐度）完全不动。人眼对色度分辨率本来就不敏感，
//   这是所有 ISP 的标准做法。
//
// 【为什么用中值而不是平均】
//   这是椒盐噪声（孤立的错误值）。中值能把孤立异常点直接剔除，
//   平均只会把它摊薄、留一圈糊边。
//
// 【为什么是水平 3 点】
//   2D 3x3 需要两条行缓存（1280x10bit x2 x2 平面）。水平 3 点只需要
//   跨拍取邻居，零缓存、零 ERAM，成本差一个数量级。
//   代价：只压水平方向的斑点，竖直方向残留。先上这一级看效果，
//   不够再升级成 2D。
//
// 【数据对齐】
//   要处理第 T-1 拍的 4 个像素，neighbour 需要 T-2 拍和第 T 拍的像素，
//   所以打两拍。控制信号同步延迟两拍，保证和 AWB 的对齐不变。
// ==================================================================
wire [95:0] S_dn_out;
wire S_dn_v2, S_dn_u2, S_dn_l2;
isp_chroma_denoise #(.IMG_WIDTH(1280)) u_chroma_denoise (
    .axi4s_video_aclk(axi4s_video_aclk), .I_rst_n(I_rst_n),
    .m_aixs_tdata_96(m_aixs_tdata_96),
    .m_aixs_tvalid(m_aixs_tvalid), .m_aixs_tuser(m_aixs_tuser),
    .m_aixs_tlast(m_aixs_tlast),
    .S_dn_out(S_dn_out), .S_dn_v2(S_dn_v2),
    .S_dn_u2(S_dn_u2), .S_dn_l2(S_dn_l2)
);

data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT (720 ),
    .IMG_WIDTH  (1280),
    .INIT_GAIN_R(20'd76824 ),   // 上电默认 R（1.1722x）
    .INIT_GAIN_B(20'd138731)    // 上电默认 B（2.1169x）
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (S_dn_l2),        // Three-stage denoise: data and controls are aligned
    .I_tuser (S_dn_u2),
    .I_tdata (S_dn_out),       // 色度降噪之后的 96bit 数据
    .I_tvalid(S_dn_v2),
    .I_tready(m_aixs_tready),
    .O_tlast (S_awb_tlast ),
    .O_tuser (S_awb_tuser ),
    .O_tdata (S_awb_data ),
    .O_tvalid(S_awb_valid),
    .O_tready(awb_O_tready),
    .I_gain_r(S_wbr),          // 标定 FSM 给出的请求增益
    .O_meas_data(S_awb_meas), // Same pixel as S_awb_data[23:0], before gain
    .I_gain_b(S_wbb),
    .O_gain_r_ap(S_gain_r_ap), // awb 实际生效的增益（OSD 显示这一对）
    .O_gain_b_ap(S_gain_b_ap),
    .O_gain_evt (S_gain_evt ), // 在首像素处整对锁存时打一拍
    .I_gain_commit_en(S_gain_commit_en)
);

    // ==================================================================
    // AWB 旁路开关
    //   原厂 AWB 是灰度世界算法：R' = R x (G/R) = G，B' = B x (G/B) = G，
    //   即把 R、B 强行拉到与 G 相等。现场光照偏蓝时它会把整幅画面拉向
    //   中性，黄色目标被洗淡（实测整帧 G-B 恒为 0，颜色判据完全失效）。
    //   置 1 直接使用去马赛克后的原始 RGB，AWB 逻辑会被综合优化掉。
    // ==================================================================
    localparam BYPASS_AWB = 1'b0;   // 显示走 AWB（现为固定增益白平衡）

    // AWB 之前的原始样本（与 S_awb_data 同像素），标定和检测都用它
    wire [23:0] S_wb_meas = BYPASS_AWB ? m_aixs_tdata_96[23:0] : S_awb_meas;

    wire [95:0] S_rgb_data  = BYPASS_AWB ? m_aixs_tdata_96 : S_awb_data;
    wire        S_rgb_valid = BYPASS_AWB ? m_aixs_tvalid  : S_awb_valid;
    assign S_rgb_tlast = BYPASS_AWB ? m_aixs_tlast : S_awb_tlast;
    wire        S_rgb_tuser = BYPASS_AWB ? m_aixs_tuser   : S_awb_tuser;

    // 检测器 / 中心区域均值统计都取"白平衡之后"的数据。
    //   白平衡一旦标定好，这里的 R/G/B 就和肉眼看到的一致，
    //   黄色判据的阈值才有物理意义；OSD 上的读数也直接反映残余色偏，
    //   读数 R200 G218 B225 就说明还要把 R 加大、B 减小。
    wire [95:0] S_det_data  = S_rgb_data;
    wire        S_det_valid = S_rgb_valid;
    wire        S_det_tlast = S_rgb_tlast;
    wire        S_det_tuser = S_rgb_tuser;

    // 基础要求(2)：Sobel 边缘检测（灰度 -> 行缓存 -> Sobel -> 二值化）
    // 输入打一拍：AWB 乘法器输出直接进灰度的 3 级加法链会形成 14 级逻辑，
    // 在 103.52MHz 下违例 -1.221ns，这里隔开一级流水。
    reg [95:0] ed_tdata_d;
    reg        ed_tlast_d;
    reg        ed_tuser_d;
    reg        ed_tvalid_d;

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ed_tdata_d  <= 96'd0;
            ed_tlast_d  <= 1'b0;
            ed_tuser_d  <= 1'b0;
            ed_tvalid_d <= 1'b0;
        end
        else begin
            ed_tdata_d  <= S_rgb_data;
            ed_tlast_d  <= S_rgb_tlast;
            ed_tuser_d  <= S_rgb_tuser;
            ed_tvalid_d <= S_rgb_valid;
        end
    end

    // EDGE_TH 取值 0~2040（|Gx|+|Gy| 的满量程），越大越干净但弱边缘会丢掉。
    // 原厂默认 80 只有 3.9%，噪点会全部通过；128 约 6.3%。
    edge_detect_96 #(
        .IMG_WIDTH (1280),
        .IMG_HEIGHT(720 ),
        .EDGE_TH   (11'd128)
    ) u_edge_detect_96 (
        .I_clk   (axi4s_video_aclk),
        .I_rst_n (I_rst_n),
        .I_tlast (ed_tlast_d),
        .I_tuser (ed_tuser_d),
        .I_tdata (ed_tdata_d),
        .I_tvalid(ed_tvalid_d),
        .I_tready(awb_O_tready),
        .O_tlast (edge_O_tlast),
        .O_tuser (edge_O_tuser),
        .O_tdata (edge_O_tdata),
        .O_tvalid(edge_O_tvalid),
        .O_tready(edge_O_tready)
    );

    // ==================================================================
    // 多算法切换：原图 / 边缘 / 灰度
    //   边缘通路固有延迟 = 输入打拍(1) + 模块内输出寄存器(1) = 2 拍，
    //   原图与灰度通路各补两级寄存器与之对齐，切换时不会出现行列错位。
    // ==================================================================
    function [23:0] rgb_to_gray;
        input [23:0] px;
        reg   [7:0]  g;
        begin
            g = {2'b0,px[23:18]} + {1'b0,px[15:9]} + {2'b0,px[7:2]};
            rgb_to_gray = {g,g,g};
        end
    endfunction

    function [7:0] rgb_to_y8;
        input [23:0] px;
        begin
            rgb_to_y8 = {2'b0,px[23:18]} + {1'b0,px[15:9]} + {2'b0,px[7:2]};
        end
    endfunction

    wire [95:0] gray_comb = { rgb_to_gray(S_rgb_data[95:72]),
                              rgb_to_gray(S_rgb_data[71:48]),
                              rgb_to_gray(S_rgb_data[47:24]),
                              rgb_to_gray(S_rgb_data[23:0]) };

    // ==================================================================
    // 波普风格（扩展要求 1）：色调分离 + 假色彩
    //   灰度量化成 4 档，每档映射到一组高饱和配色，
    //   还原《玛丽莲·梦露》丝网印刷的平涂色块效果。
    //   4 组配色通过 S_pal 切换（后续接入自动轮换）。
    // ==================================================================
    wire [1:0] S_pal = I_pal;            // 配色方案，由顶层每 3 秒轮换

    function [23:0] pop_pal;
        input [1:0] idx;                 // 灰度档 0=最暗 .. 3=最亮
        input [1:0] pal;                 // 配色方案
        begin
            case (pal)
                2'd1: case (idx)
                    2'd0: pop_pal = 24'h00574b;
                    2'd1: pop_pal = 24'hffe0b2;
                    2'd2: pop_pal = 24'h00e5ff;
                    default: pop_pal = 24'hff00aa;
                endcase
                2'd2: case (idx)
                    2'd0: pop_pal = 24'hd32f2f;
                    2'd1: pop_pal = 24'hffc0cb;
                    2'd2: pop_pal = 24'h1e5aa8;
                    default: pop_pal = 24'hff7a00;
                endcase
                2'd3: case (idx)
                    2'd0: pop_pal = 24'h202020;
                    2'd1: pop_pal = 24'hffd400;
                    2'd2: pop_pal = 24'hff3b30;
                    default: pop_pal = 24'hf2f2f2;
                endcase
                default: case (idx)      // 方案 0：梦露经典冷暖配
                    2'd0: pop_pal = 24'h7b2d8e;   // 阴影 深紫
                    2'd1: pop_pal = 24'hff9ec4;   // 中间调 皮肤粉
                    2'd2: pop_pal = 24'hffd400;   // 亮部 头发黄
                    default: pop_pal = 24'h00c8c8; // 最亮 背景青
                endcase
            endcase
        end
    endfunction

    function [1:0] pop_band;             // 8bit 灰度 -> 4 档
        input [7:0] g;
        begin
            if      (g < 8'd64 ) pop_band = 2'd0;
            else if (g < 8'd128) pop_band = 2'd1;
            else if (g < 8'd192) pop_band = 2'd2;
            else                 pop_band = 2'd3;
        end
    endfunction

    // 每拍 4 个像素的灰度档位（像素 0 为最左）
    wire [1:0] b0 = pop_band(rgb_to_y8(S_rgb_data[95:72]));
    wire [1:0] b1 = pop_band(rgb_to_y8(S_rgb_data[71:48]));
    wire [1:0] b2 = pop_band(rgb_to_y8(S_rgb_data[47:24]));
    wire [1:0] b3 = pop_band(rgb_to_y8(S_rgb_data[23:0]));

    // 上一拍 4 个像素的档位，用来取 x-2 位置的档位
    reg [7:0] S_bands_d;

    wire [1:0] s0 = S_bands_d[5:4];      // x-2 落在上一拍第 3 个像素
    wire [1:0] s1 = S_bands_d[7:6];      // x-2 落在上一拍第 4 个像素
    wire [1:0] s2 = b0;
    wire [1:0] s3 = b1;

    // 套印错位：中间调色层（皮肤粉 / 头发黄）按 x-2 的档位着色，
    // 还原丝网印刷中套色没对齐、颜色向右溢出的效果；最暗/最亮层不动。
    wire [1:0] e0 = ((s0 == 2'd1) || (s0 == 2'd2)) ? s0 : b0;
    wire [1:0] e1 = ((s1 == 2'd1) || (s1 == 2'd2)) ? s1 : b1;
    wire [1:0] e2 = ((s2 == 2'd1) || (s2 == 2'd2)) ? s2 : b2;
    wire [1:0] e3 = ((s3 == 2'd1) || (s3 == 2'd2)) ? s3 : b3;

    wire [95:0] pop_comb = { pop_pal(e0, S_pal),
                             pop_pal(e1, S_pal),
                             pop_pal(e2, S_pal),
                             pop_pal(e3, S_pal) };

    // ==================================================================
    // 黄色目标检测（扩展要求 2）
    //   逐拍统计外接框与命中像素数，帧尾锁存上一帧结果；
    //   再叠一层"面积 + 填充率 + 长宽比"的形状校验，滤掉零散噪点。
    //   取的是白平衡之后的 RGB，与当前显示算法（RAW/EDGE/GRAY/POP）无关。
    // ==================================================================
    // 判据（作用在白平衡之后的数据上，R/G/B 已经和肉眼一致）：
    //   R、G 都高于 B；|R-G| 相对 G 足够小；G-B 相对 G 足够大。
    //
    //   每条各自排除什么：
    //     R > B, G > B   白/灰面 R≈G≈B 过不了；青/蓝/紫/品红/红 过不了
    //     |R-G| <= G/8   红绿物体过不了 —— 黄色必须 R≈G（相对尺度）
    //     G-B   >= G/8   饱和度太低的近灰物体过不了（相对尺度）
    //     4(G-B) >= 3(R-B)  原有比例判据，抑制 R 远高于 G 的橙色/肤色
    //     G >= MIN_LUM   绝对噪声地板，暗处噪点直接滤掉
    //
    //   ★ 本轮改动：原来 R>=B+16、G>=B+16、|R-G|<=20 全是【绝对】门限，
    //     亮度一变或白平衡留一点暖色残差就会被判非黄。现在改成相对量
    //     （G/8），只保留 G>=MIN_LUM 这一个绝对地板。全部用移位/加法实现，
    //     不引入乘法器，DSP 不增加。
    //
    //   注意：判据取的是**白平衡之后**的数据。之前走自适应 AWB 时
    //   R、B 被强行拉到等于 G，整帧 G-B 恒为 0，任何黄色判据都会失效；
    //   现在换成固定增益，颜色保真，阈值才有物理意义。
    localparam [7:0] YEL_MIN_LUM = 8'd40;       // 亮度地板，排除暗处噪声

    // 注意：这个 function 被 4 个 wire 并发调用，里面不能有局部变量
    // （静态局部会被 4 次调用共享，属于不该冒的险），全部写成纯组合表达式，
    // 每个比较都用 {1'b0,...} 显式扩到 9 位，减法不会回卷。
    // ------------------------------------------------------------------
    // 黄色判据：把两个【绝对门限】改成【相对门限】（移位即可，无乘法器）
    //   原判据 R>=B+16、G>=B+16、|R-G|<=20 都是绝对量：
    //   亮度一变、或白平衡留一点暖色残差，|R-G| 就会超过 20 而被判非黄 ——
    //   这是实板"大部分时间没框"最可能的原因之一。
    //   现在改为：
    //     |R-G| <= G/8      （R 与 G 的相对接近程度，尺度无关）
    //     (G-B) >= G/8      （黄色饱和度下限，尺度无关）
    //     4(G-B) >= 3(R-B)  （原有比例判据，本来就尺度无关，保留）
    //     G >= YEL_MIN_LUM  （绝对噪声地板，保留：暗处不可信）
    //     R > B 且 G > B    （取代原来的 +16 偏移）
    //   全部用移位/加法实现，不引入乘法器，DSP 不增加。
    // ------------------------------------------------------------------
    function yellow_hit;
        input [23:0] px;                                          // {R,G,B}
        reg [10:0] rg;                                            // |R-G|
        reg [10:0] gb;                                            // G-B
        reg [10:0] rb;                                            // R-B
        begin
            rg = (px[23:16] >= px[15:8]) ? ({3'b0,px[23:16]} - {3'b0,px[15:8]})
                                         : ({3'b0,px[15:8]}  - {3'b0,px[23:16]});
            gb = (px[15:8] >= px[7:0])   ? ({3'b0,px[15:8]}  - {3'b0,px[7:0]})
                                         : 11'd0;
            rb = (px[23:16] >= px[7:0])  ? ({3'b0,px[23:16]} - {3'b0,px[7:0]})
                                         : 11'd0;
            yellow_hit =
                (px[15:8] >= YEL_MIN_LUM) &&                // 绝对噪声地板
                (px[23:16] >  px[7:0])    &&                // R > B
                (px[15:8]  >  px[7:0])    &&                // G > B
                ((rg << 3) <= {3'b0, px[15:8]}) &&          // |R-G| <= G/8
                ((gb << 3) >= {3'b0, px[15:8]}) &&          // G-B  >= G/8
                ((gb << 2) >= (rb + rb + rb));              // 4(G-B) >= 3(R-B)
        end
    endfunction

    localparam [23:0] DET_MIN_PIX = 24'd800;

    // ==================================================================
    // 黄色圆形卷尺识别（颜色 + 形状）
    // ------------------------------------------------------------------
    // 【流水】原链 yellow_hit(4 份组合) -> det_cnt_beat -> det_any ->
    //   bbox 比较 -> 计数器 全挤在一拍，实测 -3.407ns。
    //   现在第 1 级把【颜色判据 + 本拍 x/y】一起寄存一拍，
    //   下游全部用寄存后的一组，长链被切断。
    //   坐标与命中严格同拍：S_xq/S_yq 与 S_h_q 在同一拍锁存。
    // ==================================================================
    wire h0 = yellow_hit(S_det_data[95:72]);
    wire h1 = yellow_hit(S_det_data[71:48]);
    wire h2 = yellow_hit(S_det_data[47:24]);
    wire h3 = yellow_hit(S_det_data[23:0]);

    // 拍内首个 / 末个命中像素的索引（用于逐行外轮廓跨度）
    wire [1:0] S_f_idx = h0 ? 2'd0 : h1 ? 2'd1 : h2 ? 2'd2 : 2'd3;
    wire [1:0] S_l_idx = h3 ? 2'd3 : h2 ? 2'd2 : h1 ? 2'd1 : 2'd0;

    // 第 1 级流水：命中 / 坐标 / valid / 预AWB样本 / 每拍RGB和 全部同级
    //   —— 第二级只消费这一组，流暂停时也能把最后一拍排出。
    reg  [3:0]  S_h_q;
    reg  [1:0]  S_fi_q, S_li_q;
    reg  [11:0] S_xq, S_yq;
    reg         S_det_vq;
    reg  [23:0] S_wbm_q;
    reg  [9:0]  S_beatq_r, S_beatq_g, S_beatq_b;
    reg  [11:0] S_det_x, S_det_y;

    // 本拍 4 个像素的 RGB 各自求和（供中心 ROI 均值 / AE 测光）
    wire [9:0] S_beat_r = S_det_data[95:88] + S_det_data[71:64]
                        + S_det_data[47:40] + S_det_data[23:16];
    wire [9:0] S_beat_g = S_det_data[87:80] + S_det_data[63:56]
                        + S_det_data[39:32] + S_det_data[15:8];
    wire [9:0] S_beat_b = S_det_data[79:72] + S_det_data[55:48]
                        + S_det_data[31:24] + S_det_data[7:0];


    // 自由运行的拍计数：每行 320 拍，x 到 1276 回卷并换行，y 到 719 回卷
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_h_q   <= 4'd0;   S_fi_q <= 2'd0;  S_li_q <= 2'd0;
            S_xq    <= 12'd0;  S_yq   <= 12'd0;
            S_det_vq<= 1'b0;
            S_wbm_q <= 24'd0;
            S_beatq_r <= 10'd0; S_beatq_g <= 10'd0; S_beatq_b <= 10'd0;
            S_det_x <= 12'd0;  S_det_y <= 12'd0;
        end
        else begin
            // ★ 每拍默认跟随当前 valid（延迟一拍与 S_h_q/S_xq 对齐）。
            //   旧写法只在 valid=1 时置 1、从不归零，于是首个有效拍之后
            //   S_det_vq 在所有空拍恒为 1，下游（ROI 取样 / 计数 / 帧事件 /
            //   AE）会把最后一拍的旧命中和旧坐标重复消费无数次。
            S_det_vq <= S_det_valid;
            if (S_det_valid) begin
                S_h_q   <= {h3, h2, h1, h0};
                S_fi_q  <= S_f_idx;
                S_li_q  <= S_l_idx;
                S_xq    <= S_det_x;
                S_yq    <= S_det_y;
                S_wbm_q <= S_wb_meas;
                S_beatq_r <= S_beat_r;
                S_beatq_g <= S_beat_g;
                S_beatq_b <= S_beat_b;
                if (S_det_x >= 12'd1276) begin
                    S_det_x <= 12'd0;
                    S_det_y <= (S_det_y >= 12'd719) ? 12'd0 : S_det_y + 12'd1;
                end
                else begin
                    S_det_x <= S_det_x + 12'd4;
                end
            end
        end
    end

    // ---- 第 2 级：逐拍统计（全部用寄存后的 S_h_q / S_xq / S_yq）----
    wire [2:0]  S_cnt_beat  = {2'b00,S_h_q[0]} + {2'b00,S_h_q[1]}
                            + {2'b00,S_h_q[2]} + {2'b00,S_h_q[3]};
    // 本拍至少 2 个像素命中才扩张外接框：沿 x 做一次腐蚀，孤立噪点不撑框
    wire        S_any_beat  = (S_cnt_beat >= 3'd2);
    wire [11:0] S_hit_minx  = S_xq + {10'd0, S_fi_q};
    wire [11:0] S_hit_maxx  = S_xq + {10'd0, S_li_q};
    wire        S_row_end   = (S_xq >= 12'd1276);
    wire        S_frm_end   = S_row_end && (S_yq >= 12'd719);
    // 与"检测屏蔽"无关的真实帧尾（同拍、未屏蔽）：标定与 AE 用它，
    // 不能用被 I_clr 掐掉的 u_span_acc.O_evt。
    wire        S_frm_evt_det = S_det_vq && (S_xq >= 12'd1276) &&
                                (S_yq >= 12'd719);

    // 帧尾快照（由 isp_span_acc 输出）+ 面积流水
    wire [11:0] S_eval_w, S_eval_h;
    wire [23:0] S_eval_cnt;
    wire [11:0] S_eval_minx, S_eval_maxx, S_eval_miny, S_eval_maxy;
    wire [20:0] S_eval_span;
    wire [10:0] S_eval_rows;
    wire        S_tuser_evt;

    reg  [23:0] S_area_r;
    reg         S_pend1, S_pend2;

    // ---- 标定状态：必须在这之前声明，因为下面的统计模块要吃 S_det_mask ----
    localparam [3:0] CAL_OFF   = 4'd0, CAL_ACC   = 4'd1,
                     CAL_DIVR  = 4'd2, CAL_DIVB  = 4'd3,
                     CAL_CHK   = 4'd4, CAL_APPLY = 4'd5,
                     CAL_SHDW  = 4'd6, CAL_OK    = 4'd7,
                     CAL_ERR   = 4'd8;
    reg  [3:0] S_cal_st;
    reg  [7:0] S_mask_hold;
    wire S_cal_busy = (S_cal_st != CAL_OFF) && (S_cal_st != CAL_OK) &&
                      (S_cal_st != CAL_ERR);
    wire S_det_mask;
    assign S_det_mask = S_cal_busy || (S_mask_hold != 8'd0);

    isp_span_acc u_span_acc (
        .I_clk   (axi4s_video_aclk),
        .I_rst_n (I_rst_n),
        .I_valid (S_det_vq),
        .I_clr   (S_det_mask),
        .I_x     (S_xq),
        .I_y     (S_yq),
        .I_h     (S_h_q),
        .O_w     (S_eval_w),
        .O_h     (S_eval_h),
        .O_cnt   (S_eval_cnt),
        .O_span  (S_eval_span),
        .O_rows  (S_eval_rows),
        .O_minx  (S_eval_minx),
        .O_maxx  (S_eval_maxx),
        .O_miny  (S_eval_miny),
        .O_maxy  (S_eval_maxy),
        .O_evt   (S_tuser_evt)
    );

    // ---- 中心 64x64 区域平均色：白平衡标定 / 单色目标取样 ----
    //   x 608~671（16 拍），y 328~391（64 行），共 4096 像素，平均 = 累加 >> 12。
    //   坐标用与命中间拍的 S_xq / S_yq，保证与检测看到的是同一批像素。
    // 坐标与数据都取第 2 级（S_xq/S_yq 与 S_beatq_* 同级），
    // 否则中心 OSD 会相对画面错开 1 拍 = 4 像素。
    wire S_in_center = (S_xq >= 12'd608) && (S_xq < 12'd672) &&
                       (S_yq >= 12'd328) && (S_yq < 12'd392);

    reg  [20:0] S_sum_r, S_sum_g, S_sum_b;
    reg         S_frm_d;


    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_sum_r <= 21'd0;
            S_sum_g <= 21'd0;
            S_sum_b <= 21'd0;
            S_frm_d <= 1'b0;
            O_rgb_r <= 8'd0;
            O_rgb_g <= 8'd0;
            O_rgb_b <= 8'd0;
        end
        else begin
            // 帧尾事件打一拍，等求和逻辑停下后再锁存，避免把帧尾那一拍算进去
            S_frm_d <= S_frm_evt_det;   // 与检测屏蔽无关的真实帧尾

            if (S_frm_d) begin
                O_rgb_r <= S_sum_r[19:12];
                O_rgb_g <= S_sum_g[19:12];
                O_rgb_b <= S_sum_b[19:12];
                S_sum_r <= 21'd0;
                S_sum_g <= 21'd0;
                S_sum_b <= 21'd0;
            end
            else if (S_det_vq && S_in_center) begin
                S_sum_r <= S_sum_r + {11'd0, S_beatq_r};
                S_sum_g <= S_sum_g + {11'd0, S_beatq_g};
                S_sum_b <= S_sum_b + {11'd0, S_beatq_b};
            end
        end
    end

    // ==================================================================
    // 一键白纸标定（KEY1 长按 1.5 s 触发；成功后锁定，直到下次长按重算）
    // ------------------------------------------------------------------
    // 【取样】AWB 乘法器之前的 S_wb_meas，与检测严格同拍同像素。
    //   只统计中心 64x64 ROI 内的有效像素；削顶 / 过暗一律剔除。
    // 【帧资格】必须"见过本帧首拍"才认这一帧 —— 触发点落在 ROI 当中时，
    //   那个残帧不会被当成完整帧凑数。
    // 【AE 已稳定】可观测代理判据：相邻两个合格帧的 ROI 样本数差 <= 16、
    //   G 累加值变化 <= 1/32（约 3.1%）。不依赖任何 I2C 曝光寄存器状态，
    //   也不需要在 24 MHz 域做握手。连续 4 个合格帧才进入除法。
    // 【增益】绝对比值 gR = (ΣG<<16)/ΣR、gB = (ΣG<<16)/ΣB（Q16），
    //   不是 AWB 之后的比值。
    // 【失败】样本不足 / 分母过小 / 越界 / 走廊不过 / 看门狗超时 -> CAL_ERR，
    //   保留上次成功值；越界绝不静默钳位成"成功"。
    // 【提交】CAL_CHK 通过 -> CAL_APPLY 写请求增益 -> CAL_SHDW 等 awb 的
    //   O_gain_evt。awb 在它自己的"新帧首像素"那一拍整对锁存，所以
    //   这里什么时候写都不会让同一帧里混用新旧增益（审查项 D）。
    // 【看门狗】S_cal_tmo 在【所有】非空闲状态累加：
    //   任何 BUSY 状态（ACC / DIVR / DIVB / CHK / SHDW）墙钟超时 -> CAL_ERR，
    //   并且【撤销待提交请求】（S_wbr/S_wbb 退回上次真正生效的一对）。
    //   绝不出现"报 OK 但请求还挂着、视频恢复后偷偷生效"。
    // ==================================================================
    localparam [19:0] WB_INIT_R = 20'd76824;    // 上电默认 R（Q16 1.1722x）
    localparam [19:0] WB_INIT_B = 20'd138731;   // 上电默认 B（Q16 2.1169x）

    localparam [19:0] CAL_R_MIN = 20'd52429;    // 0.80x
    localparam [19:0] CAL_R_MAX = 20'd124518;   // 1.90x
    localparam [19:0] CAL_B_MIN = 20'd85197;    // 1.30x
    localparam [19:0] CAL_B_MAX = 20'd170394;   // 2.60x
    localparam [20:0] CAL_N_MIN = 21'd900;      // 每帧 ROI 最少有效样本（满 1024）
    localparam [23:0] CAL_SUM_MIN = 24'd4096;   // ΣR / ΣB 下限（极小分母保护）
    localparam [2:0]  CAL_NFRM  = 3'd4;         // 需要 4 个稳定完整帧
    localparam [7:0]  CAL_MASK  = 8'd30;        // 成功后再屏蔽 30 帧检测

    // 坐标 / valid / 预 AWB 样本全部取第 2 级，与检测严格同像素
    wire S_cal_roi    = (S_xq >= 12'd608) && (S_xq < 12'd672) &&
                        (S_yq >= 12'd328) && (S_yq < 12'd392);
    wire [7:0] S_cal_r = S_wbm_q[23:16];
    wire [7:0] S_cal_g = S_wbm_q[15:8];
    wire [7:0] S_cal_b = S_wbm_q[7:0];
    // 饱和判据必须按【真实满量程】给：RAW10 最大 1023，减 BLC(60) 再 >>2
    // 之后满量程是 (1023-60)>>2 = 240。原来的 >=250 对真实数据永远不可达，
    // 等于没有削顶保护。这里取 236（低于满量程约 2%），BLC 本身不动。
    localparam [7:0] CAL_SAT = 8'd236;
    wire S_cal_bad    = (S_cal_r >= CAL_SAT) || (S_cal_g >= CAL_SAT) ||
                        (S_cal_b >= CAL_SAT) || (S_cal_g < 8'd40);
    wire S_cal_smp    = S_det_vq && S_cal_roi && !S_cal_bad;
    // ★ 帧首 / 帧尾都用【本级、同拍、未被屏蔽】的信号。
    //   S_tuser_evt 来自 u_span_acc.O_evt，而标定期间 I_clr=1 在 acc 里
    //   优先级高于帧结束 —— 标定一开就再也没有 O_evt，CAL_ACC 收不齐一帧、
    //   mask 也无法倒计时。所以这里独立产生，作为"这一帧真的走完"的唯一判据；
    //   检测结果好不好另外门控（那是 O_evt / S_tuser_evt 的事）。
    wire S_cal_fstart  = S_det_vq && (S_xq == 12'd0) && (S_yq == 12'd0);

    reg [2:0]  S_cal_nf;                 // 已接受的稳定帧数
    reg        S_cal_armed;              // 见过本帧首拍
    reg [23:0] S_fr_r, S_fr_g, S_fr_b, S_fr_n;   // 本帧 ROI 累加
    reg [23:0] S_pv_r, S_pv_g, S_pv_b, S_pv_n;   // 上一个合格帧（稳定性参照）
    reg [23:0] S_tt_r, S_tt_g, S_tt_b, S_tt_n;   // 已接受帧的跨帧累计
    reg [29:0] S_cal_tmo;

    // ★ 提交许可：纯组合、带墙钟上界。
    //   S_cal_tmo 在 CAL_APPLY 里清 0，进 SHDW 后逐拍 +1；
    //   一旦到 CAL_TMO 这一拍 en 已经是 0 —— 那一拍及以后都不可能再提交，
    //   所以"到期撤销 pending"不可能与"同一拍正在提交"冲突。
    assign S_gain_commit_en = (S_cal_st == CAL_SHDW) && (S_cal_tmo < CAL_COMMIT_TMO);
    reg [19:0] S_cal_gr, S_cal_gb;       // 算出的候选增益

    // 除法器（36 拍恢复余数法，每帧只跑一次，不在像素通路上）
    reg  [35:0] S_cnum;
    reg  [20:0] S_cden, S_crem;
    reg  [35:0] S_cquo;
    reg  [5:0]  S_ccnt;
    reg         S_cpass;                 // 0 = 算 R，1 = 算 B
    wire [21:0] S_crsh = {S_crem, S_cnum[35]};
    wire        S_cge  = (S_crsh >= {1'b0, S_cden});
    wire [21:0] S_crnx = S_cge ? (S_crsh - {1'b0, S_cden}) : S_crsh;
    // ★ 审查项 A：最后一拍商在同一拍才更新，采样必须用 next 商，
    //   否则丢掉最高位，结果约减半。
    wire [35:0] S_cquo_n = {S_cquo[34:0], S_cge};
    wire [19:0] S_cqs_n  = (|S_cquo_n[35:20]) ? 20'hFFFFF : S_cquo_n[19:0];

    // 帧间稳定性：样本数差 <= 16 且 G 累加值差 <= 1/32
    wire [23:0] S_st_n = (S_fr_n > S_pv_n) ? (S_fr_n - S_pv_n) : (S_pv_n - S_fr_n);
    wire [23:0] S_st_g = (S_fr_g > S_pv_g) ? (S_fr_g - S_pv_g) : (S_pv_g - S_fr_g);
    wire S_cal_stab = (S_tt_n == 24'd0) ||
                      ((S_st_n <= 24'd16) && (S_st_g <= (S_pv_g >> 5)));

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_cal_st    <= CAL_OFF;
            S_cal_nf    <= 3'd0;
            S_cal_armed <= 1'b0;
            S_fr_r <= 24'd0; S_fr_g <= 24'd0; S_fr_b <= 24'd0; S_fr_n <= 24'd0;
            S_pv_r <= 24'd0; S_pv_g <= 24'd0; S_pv_b <= 24'd0; S_pv_n <= 24'd0;
            S_tt_r <= 24'd0; S_tt_g <= 24'd0; S_tt_b <= 24'd0; S_tt_n <= 24'd0;
            S_cal_tmo   <= 30'd0;
            S_cal_gr    <= 20'd0; S_cal_gb <= 20'd0;
            S_mask_hold <= 8'd0;
            S_cnum <= 36'd0; S_cden <= 21'd0; S_crem <= 21'd0;
            S_cquo <= 36'd0; S_ccnt <= 6'd0;  S_cpass <= 1'b0;
            // ★ 审查项 B：S_wbr / S_wbb 的唯一默认值给出点
            S_wbr <= WB_INIT_R;
            S_wbb <= WB_INIT_B;
            O_cal_st <= 2'd0;
        end
        else begin
            case (S_cal_st)
                CAL_OFF, CAL_OK, CAL_ERR: begin
                    O_cal_st <= (S_cal_st == CAL_OK)  ? 2'd2 :
                                (S_cal_st == CAL_ERR) ? 2'd3 : 2'd0;
                    S_cal_tmo <= 30'd0;
                    if ((S_mask_hold != 8'd0) && S_frm_evt_det)
                        S_mask_hold <= S_mask_hold - 8'd1;
                    if (I_cal_trig) begin
                        S_cal_st    <= CAL_ACC;
                        S_cal_nf    <= 3'd0;
                        S_cal_armed <= 1'b0;
                        S_fr_r <= 24'd0; S_fr_g <= 24'd0;
                        S_fr_b <= 24'd0; S_fr_n <= 24'd0;
                        S_pv_r <= 24'd0; S_pv_g <= 24'd0;
                        S_pv_b <= 24'd0; S_pv_n <= 24'd0;
                        S_tt_r <= 24'd0; S_tt_g <= 24'd0;
                        S_tt_b <= 24'd0; S_tt_n <= 24'd0;
                        S_cal_tmo   <= 30'd0;
                        S_mask_hold <= 8'd0;
                    end
                end

                CAL_ACC: begin
                    O_cal_st  <= 2'd1;
                    S_cal_tmo <= S_cal_tmo + 30'd1;
                    if (S_cal_fstart) S_cal_armed <= 1'b1;
                    if (S_cal_smp) begin
                        S_fr_r <= S_fr_r + {16'd0, S_cal_r};
                        S_fr_g <= S_fr_g + {16'd0, S_cal_g};
                        S_fr_b <= S_fr_b + {16'd0, S_cal_b};
                        S_fr_n <= S_fr_n + 24'd1;
                    end
                    if (S_frm_evt_det) begin
                        if (S_cal_armed && (S_fr_n >= CAL_N_MIN) && S_cal_stab) begin
                            S_tt_r <= S_tt_r + S_fr_r;
                            S_tt_g <= S_tt_g + S_fr_g;
                            S_tt_b <= S_tt_b + S_fr_b;
                            S_tt_n <= S_tt_n + S_fr_n;
                            S_pv_r <= S_fr_r; S_pv_g <= S_fr_g;
                            S_pv_b <= S_fr_b; S_pv_n <= S_fr_n;
                            S_cal_nf <= S_cal_nf + 3'd1;
                            if (S_cal_nf == (CAL_NFRM - 3'd1)) begin
                                // 第 4 个合格帧。ΣG 上限 4x1024x255 = 1044480
                                // < 2^20，左移 16 位恰好落在 36 位除法器内。
                                S_cnum  <= {S_tt_g[19:0] + S_fr_g[19:0], 16'd0};
                                S_cden  <= S_tt_r[20:0] + S_fr_r[20:0];
                                S_cpass <= 1'b0;
                                S_ccnt  <= 6'd35;
                                S_crem  <= 21'd0;
                                S_cquo  <= 36'd0;
                                S_cal_tmo <= 30'd0;
                                S_cal_st  <= CAL_DIVR;
                            end
                        end
                        else begin
                            // 残帧 / 样本不足 / 帧间抖动：整轮重来，绝不拿残帧凑数
                            S_cal_nf <= 3'd0;
                            S_tt_r <= 24'd0; S_tt_g <= 24'd0;
                            S_tt_b <= 24'd0; S_tt_n <= 24'd0;
                            S_pv_r <= 24'd0; S_pv_g <= 24'd0;
                            S_pv_b <= 24'd0; S_pv_n <= 24'd0;
                        end
                        S_fr_r <= 24'd0; S_fr_g <= 24'd0;
                        S_fr_b <= 24'd0; S_fr_n <= 24'd0;
                        // 帧尾拍必在 (1276,719)，帧首拍必在 (0,0)，
                        // 两者不可能是同一拍；这里仍然按"同拍优先保留"
                        // 来写，免得将来把帧尾换成延迟版本时被悄悄覆盖。
                        S_cal_armed <= S_cal_fstart;
                    end
                    if (S_cal_tmo >= CAL_TMO) S_cal_st <= CAL_ERR;
                end

                CAL_DIVR, CAL_DIVB: begin
                    O_cal_st  <= 2'd1;
                    S_cal_tmo <= S_cal_tmo + 30'd1;
                    S_crem <= S_crnx[20:0];
                    S_cquo <= S_cquo_n;
                    S_cnum <= {S_cnum[34:0], 1'b0};
                    S_ccnt <= S_ccnt - 6'd1;
                    if (S_ccnt == 6'd0) begin
                        if (!S_cpass) begin
                            S_cal_gr <= S_cqs_n;          // ★ next 商
                            S_cpass  <= 1'b1;
                            S_cnum   <= {S_tt_g[19:0], 16'd0};
                            S_cden   <= S_tt_b[20:0];
                            S_ccnt   <= 6'd35;
                            S_crem   <= 21'd0;
                            S_cquo   <= 36'd0;
                            S_cal_st <= CAL_DIVB;
                        end
                        else begin
                            S_cal_gb  <= S_cqs_n;         // ★ next 商
                            S_cal_tmo <= 30'd0;
                            S_cal_st  <= CAL_CHK;
                        end
                    end
                    if (S_cal_tmo >= CAL_TMO) S_cal_st <= CAL_ERR;
                end

                CAL_CHK: begin
                    O_cal_st  <= 2'd1;
                    S_cal_tmo <= S_cal_tmo + 30'd1;
                    if ((S_tt_r < CAL_SUM_MIN) || (S_tt_b < CAL_SUM_MIN) ||
                        (S_cal_gr < CAL_R_MIN) || (S_cal_gr > CAL_R_MAX) ||
                        (S_cal_gb < CAL_B_MIN) || (S_cal_gb > CAL_B_MAX) ||
                        (S_cal_gr * S_cal_gb < 40'd6871947674) ||
                        (S_cal_gr * S_cal_gb > 40'd13743895347))
                        S_cal_st <= CAL_ERR;      // 越界即失败，不静默钳位
                    else begin
                        S_cal_tmo <= 30'd0;
                        S_cal_st  <= CAL_APPLY;
                    end
                    if (S_cal_tmo >= CAL_TMO) S_cal_st <= CAL_ERR;
                end

                CAL_APPLY: begin
                    O_cal_st    <= 2'd1;
                    S_cal_tmo   <= S_cal_tmo + 30'd1;
                    // 写请求。awb 会在它自己的新帧首像素处整对锁存，
                    // 所以这里随时写都不会造成同帧新旧混用。
                    S_wbr       <= S_cal_gr;
                    S_wbb       <= S_cal_gb;
                    S_mask_hold <= CAL_MASK;
                    S_cal_tmo   <= 30'd0;      // 墙钟计时从 0 起，en 下一拍即有效
                    S_cal_st    <= CAL_SHDW;
                end

                CAL_SHDW: begin
                    O_cal_st  <= 2'd1;
                    // 优先接收上一拍的真实确认（它一定来自 en=1 的那一拍）
                    if (S_gain_evt && (S_gain_r_ap == S_cal_gr) &&
                                      (S_gain_b_ap == S_cal_gb))
                        S_cal_st <= CAL_OK;
                    else if (S_cal_tmo >= CAL_COMMIT_TMO) begin
                        // 到这一拍 S_gain_commit_en 已经是 0，awb 之后不可能
                        // 再锁存请求值，所以撤销 pending 是绝对安全的：
                        // 把请求拉回"上次真正生效"的那一对，报 ERR。
                        S_wbr    <= S_gain_r_ap;
                        S_wbb    <= S_gain_b_ap;
                        S_cal_st <= CAL_ERR;
                    end
                    else
                        S_cal_tmo <= S_cal_tmo + 30'd1;
                end

                default: S_cal_st <= CAL_OFF;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 检测整组就绪事件（供跨域快照握手）
    //   bbox / valid / cand / cnt 走 S_tuser_evt -> S_pend1 -> S_pend2 -> O_det_*,
    //   比帧尾拍本身晚 4 拍（S_tuser_evt 自己比帧尾晚 1 拍，再加 3 拍寄存器）。
    //   O_frm_tgl 必须在这个时刻翻转，否则 wrapper 在边沿采样会读到
    //   "新的 cand/valid + 旧的 bbox"这种半新半旧组。
    //
    //   源必须用【未被标定屏蔽的】S_frm_evt_det，不能用 S_tuser_evt ——
    //   后者在标定 BUSY/冷却期被 I_clr 掐掉，那样 ROI/增益/CAL 状态在
    //   标定期间就再也不会随帧上报了。
    // ------------------------------------------------------------------
    reg [3:0] S_frmt_d;
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) S_frmt_d <= 4'd0;
        else          S_frmt_d <= {S_frmt_d[2:0], S_frm_evt_det};
    end
    wire S_det_rdy = S_frmt_d[3];

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) O_frm_tgl <= 1'b0;
        else if (S_det_rdy) O_frm_tgl <= ~O_frm_tgl;
    end

    // ==================================================================
    // AE 测光：全帧 G 通道累加
    //   刻意只用 G：
    //     G 的白平衡增益按定义恒为 1.0，所以 AWB 怎么调都不影响 AE，
    //     两个闭环之间的耦合被从结构上切断。取的是白平衡之后的 G，
    //     和肉眼看到的亮度一致。
    //
    //   量程：每拍 4 个 G 之和最大 1020，整帧 320x720 = 230400 拍，
    //         累加上限 235,008,000 < 2^28，不会溢出。
    //   取 [27:20]（右移 20）得到 0~224 的读数，满量程对应全白。
    //   注意这不是严格 0~255：921600 像素 / 2^20 = 0.879，
    //   所以 O_lum = 真实均值 x 0.879，AE 的目标值按这个单位给即可。
    // ==================================================================
    reg [27:0] S_lum_acc;
    // 帧尾那一拍本身也是数据：锁存值必须用"含当前拍"的 next 和，
    // 否则每帧漏掉最后 4 个 G（全白画面少 4/921600，只在"仅尾拍亮"
    // 这种极端用例下才看得出来，但语义必须是完整的）。
    wire [27:0] S_lum_nx = S_det_vq ? (S_lum_acc + {18'd0, S_beatq_g})
                                    : S_lum_acc;

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_lum_acc <= 28'd0;
            O_lum     <= 8'd0;
            O_lum_tgl <= 1'b0;
        end
        else if (S_frm_evt_det) begin
            // 帧尾：锁存本帧均值（含尾拍），并翻转握手电平通知 24MHz 域
            O_lum     <= (S_lum_nx[27:20] > 8'd224) ? 8'd224 : S_lum_nx[27:20];
            O_lum_tgl <= ~O_lum_tgl;
            S_lum_acc <= 28'd0;
        end
        else if (S_det_vq) begin
            S_lum_acc <= S_lum_nx;
        end
    end

    // ==================================================================
    // 【已移除】原"动态白平衡"估计器（白点筛选 / 每帧除法 / 走廊 / EMA）
    // ------------------------------------------------------------------
    // 本轮产品逻辑是"一键白纸标定后锁定"，不需要连续自适应：
    //   · 连续灰度世界会把黄色卷尺一起洗淡，检测判据直接失效（实测过）；
    //   · 估计器每帧都写 S_wbr/S_wbb，与标定 FSM 形成多驱动（HDL-8007）。
    // 因此整段删除。S_wbr/S_wbb 的唯一 owner 现为下面的标定 FSM，
    // 上电默认值 76824 / 138731 也在它的复位分支里统一给出。
    // ==================================================================

    // 增益 x100 供 OSD 显示：100 / 65536 = 25 / 16384，所以 (g x 25) >> 14。
    // 显示 awb 实际生效的那一对（不是请求值）：标定后要等 awb 在帧边界
    // 锁存才真正乘到像素上，两者可能差一帧，显示生效值才与画面一致。
    wire [24:0] S_gain_r100 = S_gain_r_ap * 5'd25;
    wire [24:0] S_gain_b100 = S_gain_b_ap * 5'd25;
    assign O_gr100 = S_gain_r100[23:14];
    assign O_gb100 = S_gain_b100[23:14];

    // ---- 求值流水 + 时间滞后 ----
    //   单帧判据在阈值附近会反复越界，表现为外接框一闪一闪。
    //   这里用"置信度积分 + 迟滞"稳定输出：
    //     每帧检出 +8，未检出 -8（0~63 饱和）
    //     置信度 >= 24（约连续 3 帧）才点亮，< 8（约连续 3 帧丢失）才熄灭
    //   熄灭前的保持期里框停在最后一次检出的位置，不会乱跳。
    reg [5:0]  S_det_conf;
    reg        S_det_sticky;
    reg [11:0] S_box_minx, S_box_maxx, S_box_miny, S_box_maxy;

    // ------------------------------------------------------------------
    // 形状判据（颜色 + 形状，尺度稳定）
    //   主判据是【逐行外轮廓跨度累计 / 外接框面积】：
    //       S_span = Σ 每行 (最右命中像素x − 最左命中像素x + 1)
    //     完整圆形：Σ 弦长 = πD²/4，bbox = D²  →  比值 ≈ π/4 ≈ 0.785
    //     实心方块/矩形：Σ 跨度 ≈ w·h          →  比值 ≈ 1.0
    //     黑色内芯 / 少量缺口：外轮廓跨度不变，**不会抹掉这个特征**
    //     多个分散黄块合框：中间有空隙，跨度覆盖不到，比值明显偏低 → 判歧义
    //   与"部分命中拍占比"相比，这个量不随 1/D 变化，也不受 x mod 4 影响。
    //
    //   其余为辅助判据：
    //     - 命中像素数、实际填充率下限：只做噪声地板（黑内芯会让填充率变低，
    //       所以不能像以前那样拿填充率当主判据）
    //     - 宽高比 3:4 ~ 4:3：圆的透视投影仍接近 1，长条被排除
    //     - 有效行连续性 >= 90%：排除交错/断续的稀疏图案
    // ------------------------------------------------------------------
    wire S_ok_cnt   = (S_eval_cnt >= DET_MIN_PIX);
    wire S_ok_fill  = ((S_eval_cnt * 100) >= (S_area_r * 30));           // 噪声地板 0.30
    wire S_ok_asp   = ((S_eval_w * 4) >= (S_eval_h * 3)) &&              // w/h >= 0.75
                      ((S_eval_h * 4) >= (S_eval_w * 3));                // w/h <= 1.33
    wire S_ok_rows  = ((S_eval_rows * 10) >= (S_eval_h * 9));            // >= 90% 行有命中
    // 外轮廓跨度比 ∈ [0.65, 0.90]（100 倍定点）
    wire S_ok_span  = ((S_eval_span * 100) >= (S_area_r * 65)) &&
                      ((S_eval_span * 100) <= (S_area_r * 90));

    wire S_pass_now = S_ok_cnt && S_ok_fill && S_ok_asp && S_ok_rows && S_ok_span;

    wire [5:0] S_conf_nxt = S_pass_now ? ((S_det_conf > 6'd55) ? 6'd63 : S_det_conf + 6'd8)
                                       : ((S_det_conf < 6'd8 ) ? 6'd0  : S_det_conf - 6'd8);

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_pend1     <= 1'b0;
            S_pend2     <= 1'b0;
            S_area_r    <= 24'd0;
            S_det_conf  <= 6'd0;
            S_det_sticky<= 1'b0;
            S_box_minx  <= 12'd0;  S_box_maxx <= 12'd0;
            S_box_miny  <= 12'd0;  S_box_maxy <= 12'd0;
            O_det_minx  <= 12'd0;  O_det_maxx <= 12'd0;
            O_det_miny  <= 12'd0;  O_det_maxy <= 12'd0;
            O_det_valid <= 1'b0;
            O_det_cand  <= 1'b0;
            O_det_cnt   <= 10'd0;
        end
        else if (S_det_mask) begin
            // ★ 标定 BUSY / 冷却期：识别结果整组清零。
            //   屏蔽时 u_span_acc 被 I_clr 清空、不再产生 S_tuser_evt，
            //   下面那条评估流水整段不动 —— 如果不清，旧外接框会一直挂在
            //   画面上，看起来像"标定期间还在识别"。退出屏蔽后从完整帧
            //   重新积置信度（S_det_conf 已归零，需要连续 3 帧才重新点亮）。
            S_pend1     <= 1'b0;
            S_pend2     <= 1'b0;
            S_area_r    <= 24'd0;
            S_det_conf  <= 6'd0;
            S_det_sticky<= 1'b0;
            S_box_minx  <= 12'd0;  S_box_maxx <= 12'd0;
            S_box_miny  <= 12'd0;  S_box_maxy <= 12'd0;
            O_det_minx  <= 12'd0;  O_det_maxx <= 12'd0;
            O_det_miny  <= 12'd0;  O_det_maxy <= 12'd0;
            O_det_valid <= 1'b0;
            O_det_cand  <= 1'b0;
            O_det_cnt   <= 10'd0;
        end
        else begin
            S_pend1 <= S_tuser_evt;
            S_pend2 <= S_pend1;

            if (S_pend1)
                S_area_r <= S_eval_w * S_eval_h;

            if (S_pend2) begin
                S_det_conf <= S_conf_nxt;

                // 迟滞：累积到 24 才点亮，跌破 8 才熄灭
                if (!S_det_sticky && (S_conf_nxt >= 6'd24))
                    S_det_sticky <= 1'b1;
                else if (S_det_sticky && (S_conf_nxt < 6'd8))
                    S_det_sticky <= 1'b0;

                // 框位置在"颜色+面积通过"时就更新，不等形状校验。
                // 否则形状不通过时框坐标根本没有更新过，橙色候选框永远画不出来
                // ——"出框/不出框"就退化成一根没有诊断信息的 0/1 开关。
                if (S_eval_cnt >= DET_MIN_PIX) begin
                    S_box_minx <= S_eval_minx;
                    S_box_maxx <= S_eval_maxx;
                    S_box_miny <= S_eval_miny;
                    S_box_maxy <= S_eval_maxy;
                end

                O_det_cand <= (S_eval_cnt >= DET_MIN_PIX);
                // 调试显示：命中像素数 / 16（上限 999，即约 16000 像素）
                O_det_cnt  <= (S_eval_cnt > 24'd15984) ? 10'd999 : S_eval_cnt[13:4];
            end

            O_det_minx  <= S_box_minx;
            O_det_maxx  <= S_box_maxx;
            O_det_miny  <= S_box_miny;
            O_det_maxy  <= S_box_maxy;
            O_det_valid <= S_det_sticky;
        end
    end

    reg [95:0] raw_d1, raw_d2, gray_d1, gray_d2, pop_d1, pop_d2;
    reg        raw_v1, raw_v2, raw_u1, raw_u2, raw_l1, raw_l2;

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            raw_d1  <= 96'd0; raw_d2  <= 96'd0;
            gray_d1 <= 96'd0; gray_d2 <= 96'd0;
            pop_d1  <= 96'd0; pop_d2  <= 96'd0;
            raw_v1 <= 1'b0; raw_v2 <= 1'b0;
            raw_u1 <= 1'b0; raw_u2 <= 1'b0;
            raw_l1 <= 1'b0; raw_l2 <= 1'b0;
        end
        else begin
            raw_d1  <= S_rgb_data;   raw_d2  <= raw_d1;
            gray_d1 <= gray_comb;     gray_d2 <= gray_d1;
            pop_d1  <= pop_comb;      pop_d2  <= pop_d1;
            S_bands_d <= {b3, b2, b1, b0};
            raw_v1  <= S_rgb_valid;  raw_v2  <= raw_v1;
            raw_u1  <= S_rgb_tuser;   raw_u2  <= raw_u1;
            raw_l1  <= S_rgb_tlast;   raw_l2  <= raw_l1;
        end
    end

    reg [95:0] sel_data;
    reg        sel_valid, sel_user, sel_last;

    always @(*) begin
        case (I_mode)
            2'd1: begin
                sel_data  = edge_O_tdata;
                sel_valid = edge_O_tvalid;
                sel_user  = edge_O_tuser;
                sel_last  = edge_O_tlast;
            end
            2'd2: begin
                sel_data  = gray_d2;
                sel_valid = raw_v2;
                sel_user  = raw_u2;
                sel_last  = raw_l2;
            end
            2'd3: begin
                sel_data  = pop_d2;
                sel_valid = raw_v2;
                sel_user  = raw_u2;
                sel_last  = raw_l2;
            end
            default: begin
                sel_data  = raw_d2;
                sel_valid = raw_v2;
                sel_user  = raw_u2;
                sel_last  = raw_l2;
            end
        endcase
    end

    // ==================================================================
    // Gamma 2.2 编码（线性光 -> sRGB）
    // ------------------------------------------------------------------
    // 【为什么需要】
    //   传感器输出的是**线性光**：亮度 = 光子数。显示器按 sRGB 解释，
    //   其传递函数接近 2.2 次方。直接把线性值当 sRGB 显示，结果是
    //   中间调被压暗、色彩对比被拉大 —— 观感"发闷、发灰"。
    //   这就是之前画面反复看着"偏色/怪怪的"的放大因素之一。
    //
    // 【为什么不用 ROM】
    //   本工程每拍 4 个像素 x 3 通道 = 12 次查表/时钟。用 ERAM 做表要
    //   12 块存储体（96→128 那条路只有单口），代价和布线都不小。
    //   这里用**8 段折线**：每段 y = base + ((x低5位 x k) >> 5)，
    //   系数全是小常数，每通道一次 5x7 乘法 + 一次加法，共 12 份。
    //   实测最大误差约 4 个码值，肉眼不可辨。
    //
    // 【摆在哪里】
    //   放在算法选择之后、96->128 打包之前，所以 RAW/GRAY/EDGE/POP 四种
    //   输出都被校正（EDGE 是二值图，gamma(0)=0、gamma(255)=255，不受影响）。
    //   黄色检测仍然取 gamma 之前的数据，判据阈值一个都不用改。
    // ==================================================================
    // 分段表（x 的 [7:5] 选段，段内用 x 的低 5 位）
    //   段   x 范围     base  k     输出范围
    //   0   0  ~ 31       0   100   0   ~ 96
    //   1   32 ~ 63     100    33   100 ~ 131
    //   2   64 ~ 95     135    25   135 ~ 159
    //   3   96 ~127     159    25   159 ~ 183
    //   4   128~159     184    20   184 ~ 203
    //   5   160~191     204    17   204 ~ 220
    //   6   192~223     221    16   221 ~ 236
    //   7   224~255     239    17   239 ~ 255
    wire [95:0] gam_comb;

    genvar gj;
    generate
        for (gj = 0; gj < 12; gj = gj + 1) begin : g_gamma
            isp_gamma8 u_gamma (
                .I_data(sel_data[gj*8 +: 8]),
                .O_data(gam_comb[gj*8 +: 8])
            );
        end
    endgenerate

    // 打一拍：gamma 是纯组合，插一级流水切断路径，
    // frame_start / valid 同步延迟，保证打包时仍然对齐
    // 置 0 可旁路 gamma，便于现场对比效果（线性 vs sRGB 一眼就能看出差别）
    localparam GAMMA_EN = 1'b1;

    reg [95:0] gam_data;
    reg        gam_valid, gam_user;

    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gam_data  <= 96'd0;
            gam_valid <= 1'b0;
            gam_user  <= 1'b0;
        end
        else begin
            gam_data  <= GAMMA_EN ? gam_comb : sel_data;
            gam_valid <= sel_valid;
            gam_user  <= sel_user;
        end
    end

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),

        .I_96b_frame_start  ( gam_user          ),
        .I_96b_valid        ( gam_valid         ),
        .I_96b_data         ( gam_data          ),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(S_rgb_data),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule


// Four-pixel RGB888 gamma stage.  The 16-segment curve approximates
// y = 255 * (x / 255)^0.7.  It lifts shadows and mid-tones while preserving
// black and white and adds one registered pipeline cycle.
module axis_gamma_rgb96 (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid
);

    function [7:0] gamma07;
        input [7:0] value;
        reg [7:0] base;
        reg [7:0] delta;
        reg [11:0] interpolation;
        begin
            case (value[7:4])
                4'h0: begin base = 8'd0;   delta = 8'd37; end
                4'h1: begin base = 8'd37;  delta = 8'd23; end
                4'h2: begin base = 8'd60;  delta = 8'd19; end
                4'h3: begin base = 8'd79;  delta = 8'd18; end
                4'h4: begin base = 8'd97;  delta = 8'd16; end
                4'h5: begin base = 8'd113; delta = 8'd16; end
                4'h6: begin base = 8'd129; delta = 8'd14; end
                4'h7: begin base = 8'd143; delta = 8'd14; end
                4'h8: begin base = 8'd157; delta = 8'd14; end
                4'h9: begin base = 8'd171; delta = 8'd13; end
                4'ha: begin base = 8'd184; delta = 8'd13; end
                4'hb: begin base = 8'd197; delta = 8'd12; end
                4'hc: begin base = 8'd209; delta = 8'd12; end
                4'hd: begin base = 8'd221; delta = 8'd12; end
                4'he: begin base = 8'd233; delta = 8'd11; end
                default: begin base = 8'd244; delta = 8'd11; end
            endcase

            interpolation = delta * value[3:0] + 8;
            if (value == 8'hff)
                gamma07 = 8'hff;
            else
                gamma07 = base + (interpolation >> 4);
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
        end else begin
            O_tlast  <= I_tlast;
            O_tuser  <= I_tuser;
            O_tvalid <= I_tvalid;
            if (I_tvalid) begin
                O_tdata <= {
                    gamma07(I_tdata[95:88]), gamma07(I_tdata[87:80]), gamma07(I_tdata[79:72]),
                    gamma07(I_tdata[71:64]), gamma07(I_tdata[63:56]), gamma07(I_tdata[55:48]),
                    gamma07(I_tdata[47:40]), gamma07(I_tdata[39:32]), gamma07(I_tdata[31:24]),
                    gamma07(I_tdata[23:16]), gamma07(I_tdata[15:8]),  gamma07(I_tdata[7:0])
                };
            end else begin
                O_tdata <= 96'd0;
            end
        end
    end

endmodule


// Centre crop for the ISP's four-pixel-wide RGB888 stream.  Frame start is
// intentionally forwarded at the original frame boundary so video_in can
// reset its asynchronous FIFO before the first selected row arrives.
module axis_crop_rgb96 #(
    parameter integer IN_WIDTH   = 1792,
    parameter integer IN_HEIGHT  = 1080,
    parameter integer OUT_WIDTH  = 1600,
    parameter integer OUT_HEIGHT = 900,
    parameter integer X_START    = 96,
    parameter integer Y_START    = 90
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [95:0] O_tdata,
    output wire        O_tvalid
);

    localparam integer X_START_BEAT = X_START / 4;
    localparam integer OUT_BEATS    = OUT_WIDTH / 4;
    localparam integer X_END_BEAT   = X_START_BEAT + OUT_BEATS;

    reg [9:0]  S_hcnt;
    reg [10:0] S_vcnt;

    wire [9:0]  S_current_h = I_tuser ? 10'd0 : S_hcnt;
    wire [10:0] S_current_v = I_tuser ? 11'd0 : S_vcnt;
    wire S_inside = (S_current_h >= X_START_BEAT) &&
                    (S_current_h < X_END_BEAT) &&
                    (S_current_v >= Y_START) &&
                    (S_current_v < (Y_START + OUT_HEIGHT));

    assign O_tdata  = I_tdata;
    assign O_tvalid = I_tvalid && S_inside;
    assign O_tlast  = O_tvalid && (S_current_h == (X_END_BEAT - 1));
    assign O_tuser  = I_tuser;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_hcnt <= 10'd0;
            S_vcnt <= 11'd0;
        end
        else if(I_tuser) begin
            S_hcnt <= (I_tvalid && !I_tlast) ? 10'd1 : 10'd0;
            S_vcnt <= (I_tvalid && I_tlast) ? 11'd1 : 11'd0;
        end
        else if(I_tvalid) begin
            if(I_tlast) begin
                S_hcnt <= 10'd0;
                S_vcnt <= (S_vcnt == IN_HEIGHT - 1) ? 11'd0 : S_vcnt + 1'b1;
            end
            else begin
                S_hcnt <= S_hcnt + 1'b1;
            end
        end
    end

endmodule

module isp_chroma_denoise #(
    parameter integer IMG_WIDTH = 1280,
    parameter DEN_EN = 1'b1
)(
    input wire axi4s_video_aclk, I_rst_n,
    input wire [95:0] m_aixs_tdata_96,
    input wire m_aixs_tvalid, m_aixs_tuser, m_aixs_tlast,
    output reg [95:0] S_dn_out,
    output reg S_dn_v2, S_dn_u2, S_dn_l2
);
// Counts accepted video groups; tuser is the existing early frame pulse.
reg [11:0] beat_x;
reg first_d1, last_d1;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        beat_x <= 0;
        first_d1 <= 1'b1;
        last_d1 <= 1'b0;
    end else begin
        if (m_aixs_tuser) beat_x <= 0;
        if (m_aixs_tvalid) begin
            first_d1 <= m_aixs_tuser || (beat_x == 0);
            last_d1 <= (beat_x == IMG_WIDTH/4-1);
            beat_x <= (beat_x == IMG_WIDTH/4-1) ? 0 : beat_x + 1'b1;
        end
    end
end
// 【流水拆分】中值+重建实测 10.5ns / 14 级逻辑，超过 9.66ns 预算，拆成两级：
//   第一级算到中值并锁存，第二级做加法与饱和。整体延迟从 2 拍变 3 拍，
//   所以对外端口 S_dn_v2/u2/l2 后面再加一级寄存器，保持与 S_dn_out 对齐。
reg S_dn_v2i, S_dn_u2i, S_dn_l2i;   // 内部第二级；端口是第三级的最终结果

wire use_left = S_dn_v2i && !first_d1;
wire use_right = m_aixs_tvalid && !last_d1 && !m_aixs_tuser;

reg [95:0] S_dn_d1, S_dn_d2;
reg        S_dn_v1, S_dn_u1, S_dn_l1;


always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        S_dn_d1 <= 96'd0;  S_dn_d2 <= 96'd0;
        S_dn_v1 <= 1'b0;   S_dn_v2 <= 1'b0;
        S_dn_u1 <= 1'b0;   S_dn_u2 <= 1'b0;
        S_dn_l1 <= 1'b0;   S_dn_l2 <= 1'b0;
        S_dn_v2i <= 1'b0;
        S_dn_u2i <= 1'b0;
        S_dn_l2i <= 1'b0;
    end
    else begin
        S_dn_d1 <= m_aixs_tdata_96;
        S_dn_d2 <= S_dn_d1;
        S_dn_v1 <= m_aixs_tvalid;
        S_dn_v2i <= S_dn_v1;
        S_dn_u1 <= m_aixs_tuser;
        S_dn_u2i <= S_dn_u1;
        S_dn_l1 <= m_aixs_tlast;
        S_dn_l2i <= S_dn_l1;
        // 第三级：与重建流水对齐后送出端口
        S_dn_v2 <= S_dn_v2i;
        S_dn_u2 <= S_dn_u2i;
        S_dn_l2 <= S_dn_l2i;
    end
end

// 6 个色差样本，按顺序排开：
//   [0]     = 上上拍第 3 个像素（拍内第 0 个像素的左邻居）
//   [1..4]  = 上一拍的 4 个像素
//   [5]     = 当前拍第 0 个像素（上一拍第 3 个像素的右邻居）
// 色差用 "通道 + 256 - G" 表示成 10bit 无符号量：单调偏置不改变中位数，
// 但避免了有符号比较的麻烦。
wire [9:0] S_chr [0:5];
wire [9:0] S_chb [0:5];

genvar dc;
generate
    for (dc = 0; dc < 6; dc = dc + 1) begin : g_chroma
        if (dc == 0) begin
            assign S_chr[dc] = use_left ? ({2'b0, S_dn_d2[88+:8]} + 10'd256 - {2'b0, S_dn_d2[80+:8]}) : S_chr[1];
            assign S_chb[dc] = use_left ? ({2'b0, S_dn_d2[72+:8]} + 10'd256 - {2'b0, S_dn_d2[80+:8]}) : S_chb[1];
        end
        else if (dc == 5) begin
            assign S_chr[dc] = use_right ? ({2'b0, m_aixs_tdata_96[16+:8]} + 10'd256 - {2'b0, m_aixs_tdata_96[8+:8]}) : S_chr[4];
            assign S_chb[dc] = use_right ? ({2'b0, m_aixs_tdata_96[0+:8]} + 10'd256 - {2'b0, m_aixs_tdata_96[8+:8]}) : S_chb[4];
        end
        else begin
            assign S_chr[dc] = {2'b0, S_dn_d1[(dc-1)*24+16 +: 8]} + 10'd256 - {2'b0, S_dn_d1[(dc-1)*24+8 +: 8]};
            assign S_chb[dc] = {2'b0, S_dn_d1[(dc-1)*24+ 0 +: 8]} + 10'd256 - {2'b0, S_dn_d1[(dc-1)*24+8 +: 8]};
        end
    end
endgenerate

// 每个像素取自己左右邻居做 3 点中值
//   med3(a,b,c) = max( min(a,b), min( max(a,b), c ) )
// ----------------------------------------------------------------
// 第一级流水：只算到中值，不碰重建
wire [9:0] S_mdr [0:3];
wire [9:0] S_mdb [0:3];
wire [7:0] S_gvv [0:3];

genvar dn;
generate
    for (dn = 0; dn < 4; dn = dn + 1) begin : g_den
        wire [9:0] a_r = S_chr[dn],     b_r = S_chr[dn+1], c_r = S_chr[dn+2];
        wire [9:0] a_b = S_chb[dn],     b_b = S_chb[dn+1], c_b = S_chb[dn+2];

        wire [9:0] mn_r  = (a_r < b_r) ? a_r : b_r;
        wire [9:0] mx_r  = (a_r < b_r) ? b_r : a_r;
        wire [9:0] mn2_r = (mx_r < c_r) ? mx_r : c_r;
        wire [9:0] md_r  = (mn_r < mn2_r) ? mn2_r : mn_r;

        wire [9:0] mn_b  = (a_b < b_b) ? a_b : b_b;
        wire [9:0] mx_b  = (a_b < b_b) ? b_b : a_b;
        wire [9:0] mn2_b = (mx_b < c_b) ? mx_b : c_b;
        wire [9:0] md_b  = (mn_b < mn2_b) ? mn2_b : mn_b;

        // DEN_EN=0 时取中心样本本身：重建回去就是原像素，
        // 这样旁路和正常路径的流水深度一致，不需要额外的对齐。
        assign S_mdr[dn] = DEN_EN ? md_r   : S_chr[dn+1];
        assign S_mdb[dn] = DEN_EN ? md_b   : S_chb[dn+1];
        assign S_gvv[dn] = S_dn_d1[dn*24+8 +: 8];
    end
endgenerate

reg [9:0] S_mdr_q [0:3];
reg [9:0] S_mdb_q [0:3];
reg [7:0] S_gv_q  [0:3];

always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        S_mdr_q[0] <= 10'd0; S_mdr_q[1] <= 10'd0; S_mdr_q[2] <= 10'd0; S_mdr_q[3] <= 10'd0;
        S_mdb_q[0] <= 10'd0; S_mdb_q[1] <= 10'd0; S_mdb_q[2] <= 10'd0; S_mdb_q[3] <= 10'd0;
        S_gv_q[0]  <= 8'd0;  S_gv_q[1]  <= 8'd0;  S_gv_q[2]  <= 8'd0;  S_gv_q[3]  <= 8'd0;
    end
    else begin
        S_mdr_q[0] <= S_mdr[0]; S_mdr_q[1] <= S_mdr[1];
        S_mdr_q[2] <= S_mdr[2]; S_mdr_q[3] <= S_mdr[3];
        S_mdb_q[0] <= S_mdb[0]; S_mdb_q[1] <= S_mdb[1];
        S_mdb_q[2] <= S_mdb[2]; S_mdb_q[3] <= S_mdb[3];
        S_gv_q[0]  <= S_gvv[0]; S_gv_q[1]  <= S_gvv[1];
        S_gv_q[2]  <= S_gvv[2]; S_gv_q[3]  <= S_gvv[3];
    end
end

// ----------------------------------------------------------------
// 第二级流水：重建 C = med(色差) + G - 256，饱和到 0~255
wire [95:0] S_dn_f;

genvar dr;
generate
    for (dr = 0; dr < 4; dr = dr + 1) begin : g_recon
        wire [9:0] rr = S_mdr_q[dr] + {2'b0, S_gv_q[dr]};
        wire [9:0] bb = S_mdb_q[dr] + {2'b0, S_gv_q[dr]};
        wire [7:0] ro = (rr < 10'd256) ? 8'd0 : (rr > 10'd511) ? 8'd255 : rr[7:0];
        wire [7:0] bo = (bb < 10'd256) ? 8'd0 : (bb > 10'd511) ? 8'd255 : bb[7:0];

        assign S_dn_f[dr*24 +: 24] = {ro, S_gv_q[dr], bo};
    end
endgenerate

always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) S_dn_out <= 96'd0;
    else          S_dn_out <= S_dn_f;
end

endmodule

// Explicit 12-bit multiplication is essential: an 8-bit output expression
// otherwise truncates gd*gk BEFORE >>5, producing a sawtooth transfer curve.
module isp_gamma8(input wire [7:0] I_data, output wire [7:0] O_data);
    reg [6:0] slope;
    reg [7:0] base;
    always @(*) begin
        case (I_data[7:5])
            3'd0: begin slope=7'd100; base=8'd0;   end
            3'd1: begin slope=7'd33;  base=8'd100; end
            3'd2: begin slope=7'd25;  base=8'd135; end
            3'd3: begin slope=7'd25;  base=8'd159; end
            3'd4: begin slope=7'd20;  base=8'd184; end
            3'd5: begin slope=7'd17;  base=8'd204; end
            3'd6: begin slope=7'd16;  base=8'd221; end
            default: begin slope=7'd17; base=8'd239; end
        endcase
    end
    wire [11:0] product = I_data[4:0] * slope;
    wire [8:0] mapped = {1'b0,base} + {2'b00,product[11:5]};
    assign O_data = mapped[8] ? 8'hff : mapped[7:0];
endmodule

// ======================================================================
// 逐拍统计：外接框 / 命中数 / 逐行外轮廓跨度 / 有效行数 / 帧尾快照
//   独立成模块，便于用真实 RTL 做单元仿真（含 4 像素末拍与 x mod 4）。
//
//   【帧尾快照一致性】帧尾那一拍本身也是数据，所有指标都必须把它算进去。
//   本模块统一用 next-state（含当前拍）生成快照：
//     O_w/O_h/O_min*/O_max* 用 dmnx_n..dmxy_n，O_cnt 用 dcnt_n，
//     O_span 用 span_n，O_rows 用 rowc_n。
//   否则会出现"span 含尾拍、cnt/bbox 不含尾拍"的指标互相矛盾
//   （右下角目标尤其明显：最后一拍被丢掉就少一整条边）。
// ======================================================================
module isp_span_acc (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_valid,        // **必须是与 I_x/I_y/I_h 同级的延迟后 valid**
    input  wire        I_clr,          // 检测屏蔽期间清空
    input  wire [11:0] I_x,            // 与 I_h 同一拍的拍首像素坐标
    input  wire [11:0] I_y,
    input  wire [3:0]  I_h,            // 本拍 4 像素命中
    output reg  [11:0] O_w,
    output reg  [11:0] O_h,
    output reg  [23:0] O_cnt,
    output reg  [20:0] O_span,
    output reg  [10:0] O_rows,
    output reg  [11:0] O_minx,
    output reg  [11:0] O_maxx,
    output reg  [11:0] O_miny,
    output reg  [11:0] O_maxy,
    output reg         O_evt           // 帧尾单拍，与上面快照同时有效
);
    wire [2:0]  cnt_beat = {2'b00,I_h[0]} + {2'b00,I_h[1]} + {2'b00,I_h[2]} + {2'b00,I_h[3]};
    wire        any_beat = (cnt_beat >= 3'd2);
    wire [1:0]  f_idx = I_h[0] ? 2'd0 : I_h[1] ? 2'd1 : I_h[2] ? 2'd2 : 2'd3;
    wire [1:0]  l_idx = I_h[3] ? 2'd3 : I_h[2] ? 2'd2 : I_h[1] ? 2'd1 : 2'd0;
    wire [11:0] hit_minx = I_x + {10'd0, f_idx};
    wire [11:0] hit_maxx = I_x + {10'd0, l_idx};
    wire        row_end  = (I_x >= 12'd1276);
    wire        frm_end  = row_end && (I_y >= 12'd719);

    reg [11:0] dmnx, dmxx, dmny, dmxy;
    reg [23:0] dcnt;
    reg [11:0] rmnx, rmxx;
    reg        rhas;
    reg [20:0] span_acc;
    reg [10:0] row_cnt;

    // ---- next-state（含当前拍）----
    wire [11:0] dmnx_n = (any_beat && (hit_minx < dmnx)) ? hit_minx : dmnx;
    wire [11:0] dmxx_n = (any_beat && (hit_maxx > dmxx)) ? hit_maxx : dmxx;
    wire [11:0] dmny_n = (any_beat && (I_y      < dmny)) ? I_y      : dmny;
    wire [11:0] dmxy_n = (any_beat && (I_y      > dmxy)) ? I_y      : dmxy;
    // ★ 语义统一：只有"本拍命中数 >= 2"的拍才会扩张 bbox / 累加 span /
    //   计入有效行，所以 count 也只能统计这些被采纳的拍。
    //   否则单像素尾拍会出现"计了数但不进框、也不进 span"的指标互相矛盾。
    //   代价（明确写出）：一条正好 1 像素宽、每拍只有 1 个命中的竖线不会被
    //   任何一拍采纳 —— 这是刻意的单拍腐蚀，用来挡孤立噪点。
    wire [2:0]  adm_beat = any_beat ? cnt_beat : 3'd0;
    wire [23:0] dcnt_n   = dcnt + {21'd0, adm_beat};

    wire        rhas_n   = rhas | any_beat;
    wire [11:0] rmnx_n   = (any_beat && (hit_minx < rmnx)) ? hit_minx : rmnx;
    wire [11:0] rmxx_n   = (any_beat && (hit_maxx > rmxx)) ? hit_maxx : rmxx;
    wire [11:0] span_row = (rmxx_n >= rmnx_n) ? (rmxx_n - rmnx_n + 12'd1) : 12'd0;
    wire [20:0] span_n   = span_acc + {9'd0, span_row};
    wire [10:0] rowc_n   = row_cnt + {10'd0, rhas_n};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            dmnx <= 12'hfff; dmxx <= 12'd0; dmny <= 12'hfff; dmxy <= 12'd0;
            dcnt <= 24'd0; rmnx <= 12'hfff; rmxx <= 12'd0; rhas <= 1'b0;
            span_acc <= 21'd0; row_cnt <= 11'd0;
            O_w <= 12'd0; O_h <= 12'd0; O_cnt <= 24'd0;
            O_span <= 21'd0; O_rows <= 11'd0;
            O_minx <= 12'd0; O_maxx <= 12'd0; O_miny <= 12'd0; O_maxy <= 12'd0;
            O_evt <= 1'b0;
        end
        else begin
            O_evt <= 1'b0;
            if (I_clr) begin
                dmnx <= 12'hfff; dmxx <= 12'd0; dmny <= 12'hfff; dmxy <= 12'd0;
                dcnt <= 24'd0; rmnx <= 12'hfff; rmxx <= 12'd0; rhas <= 1'b0;
                span_acc <= 21'd0; row_cnt <= 11'd0;
            end
            else if (I_valid) begin
                if (frm_end) begin
                    // ★ 全部用 next-state：尾拍既算进 bbox/cnt，也算进 span/rows
                    O_w    <= (dmxx_n >= dmnx_n) ? (dmxx_n - dmnx_n + 12'd1) : 12'd0;
                    O_h    <= (dmxy_n >= dmny_n) ? (dmxy_n - dmny_n + 12'd1) : 12'd0;
                    O_cnt  <= dcnt_n;
                    O_span <= span_n;
                    O_rows <= rowc_n;
                    O_minx <= dmnx_n; O_maxx <= dmxx_n;
                    O_miny <= dmny_n; O_maxy <= dmxy_n;
                    O_evt  <= 1'b1;
                    dmnx <= 12'hfff; dmxx <= 12'd0; dmny <= 12'hfff; dmxy <= 12'd0;
                    dcnt <= 24'd0; rmnx <= 12'hfff; rmxx <= 12'd0; rhas <= 1'b0;
                    span_acc <= 21'd0; row_cnt <= 11'd0;
                end
                else begin
                    dmnx <= dmnx_n; dmxx <= dmxx_n;
                    dmny <= dmny_n; dmxy <= dmxy_n;
                    dcnt <= dcnt_n;
                    if (row_end) begin
                        if (rhas_n) begin
                            span_acc <= span_n;
                            row_cnt  <= row_cnt + 11'd1;
                        end
                        rmnx <= 12'hfff; rmxx <= 12'd0; rhas <= 1'b0;
                    end
                    else begin
                        rmnx <= rmnx_n; rmxx <= rmxx_n; rhas <= rhas_n;
                    end
                end
            end
        end
    end
endmodule


// ======================================================================
// 形状判定：颜色（上游已做）× 形状
//   O_span_ok 为主判据：Σ行跨度 / bbox 面积 ∈ [0.65, 0.90]
//     完整圆 ≈ π/4 = 0.785 ；实心方块/矩形 ≈ 1.0 ；多个分散块会因中间空隙偏低
//   I_area 由上游预先算好（w*h），避免在这里再串一个乘法。
// ======================================================================
module isp_shape_gate #(
    parameter [23:0] MIN_PIX = 24'd800
)(
    input  wire [11:0] I_w,
    input  wire [11:0] I_h,
    input  wire [23:0] I_cnt,
    input  wire [23:0] I_area,
    input  wire [20:0] I_span,
    input  wire [10:0] I_rows,
    output wire        O_cnt_ok,
    output wire        O_fill_ok,
    output wire        O_asp_ok,
    output wire        O_rows_ok,
    output wire        O_span_ok,
    output wire        O_pass
);
    assign O_cnt_ok  = (I_cnt >= MIN_PIX);
    // 填充率只做噪声地板 0.30：黑内芯会显著拉低它，不能当主判据
    assign O_fill_ok = ((I_cnt * 100) >= (I_area * 30));
    // 宽高比 3:4 ~ 4:3
    assign O_asp_ok  = ((I_w * 4) >= (I_h * 3)) && ((I_h * 4) >= (I_w * 3));
    // 有效行连续性 >= 90%
    assign O_rows_ok = ((I_rows * 10) >= (I_h * 9));
    // 外轮廓跨度比 ∈ [0.65, 0.90]
    assign O_span_ok = ((I_span * 100) >= (I_area * 65)) &&
                       ((I_span * 100) <= (I_area * 90));

    assign O_pass    = O_cnt_ok && O_fill_ok && O_asp_ok && O_rows_ok && O_span_ok;
endmodule
