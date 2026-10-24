module isp_top (
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
    output reg [7:0]  O_rgb_r		,   // 全画面滚动平均色（白平衡标定用）
    output reg [7:0]  O_rgb_g		,
    output reg [7:0]  O_rgb_b		,
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
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
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
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (S_awb_tlast ),
    .O_tuser (S_awb_tuser ),
    .O_tdata (S_awb_data ),
    .O_tvalid(S_awb_valid),
    .O_tready(awb_O_tready)
);

    // ==================================================================
    // AWB 旁路开关
    //   原厂 AWB 是灰度世界算法：R' = R x (G/R) = G，B' = B x (G/B) = G，
    //   即把 R、B 强行拉到与 G 相等。现场光照偏蓝时它会把整幅画面拉向
    //   中性，黄色目标被洗淡（实测整帧 G-B 恒为 0，颜色判据完全失效）。
    //   置 1 直接使用去马赛克后的原始 RGB，AWB 逻辑会被综合优化掉。
    // ==================================================================
    localparam BYPASS_AWB = 1'b0;   // 显示走 AWB（现为固定增益白平衡）

    wire [95:0] S_awb_data;
    wire        S_awb_valid, S_awb_tlast, S_awb_tuser;

    wire [95:0] S_rgb_data  = BYPASS_AWB ? m_aixs_tdata_96 : S_awb_data;
    wire        S_rgb_valid = BYPASS_AWB ? m_aixs_tvalid  : S_awb_valid;
    wire        S_rgb_tlast = BYPASS_AWB ? m_aixs_tlast   : S_awb_tlast;
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
    //   R、G 都比 B 高出 YEL_MARGIN，并且 R 与 G 彼此接近（|R-G| <= YEL_RG_TOL）。
    //
    //   为什么是这三条，以及每条各自排除什么：
    //     R >= B+M  白/灰面 R≈G≈B 过不了；青色、蓝色物体过不了
    //     G >= B+M  紫色、品红、红色物体（B 或 G 至少一个偏低）过不了
    //     |R-G|<=T  红色（R>>G）、绿色（G>>R）物体过不了 —— 黄色必须 R≈G
    //     亮度门槛   暗处噪点的 R/G/B 都是个位数，加个门槛直接滤掉
    //
    //   注意：判据取的是**白平衡之后**的数据。之前走自适应 AWB 时
    //   R、B 被强行拉到等于 G，整帧 G-B 恒为 0，任何黄色判据都会失效；
    //   现在换成固定增益，颜色保真，阈值才有物理意义。
    localparam [8:0] YEL_MARGIN  = 9'd16;       // R、G 至少比 B 高这么多
    localparam [8:0] YEL_RG_TOL  = 9'd20;       // R、G 允许的最大偏差
    localparam [7:0] YEL_MIN_LUM = 8'd40;       // 亮度门槛，排除暗处噪声

    // 注意：这个 function 被 4 个 wire 并发调用，里面不能有局部变量
    // （静态局部会被 4 次调用共享，属于不该冒的险），全部写成纯组合表达式，
    // 每个比较都用 {1'b0,...} 显式扩到 9 位，减法不会回卷。
    function yellow_hit;
        input [23:0] px;                                          // {R,G,B}
        begin
            yellow_hit =
                ({1'b0,px[23:16]} >= ({1'b0,px[7:0]} + YEL_MARGIN)) &&      // R >= B+M
                ({1'b0,px[15:8]}  >= ({1'b0,px[7:0]} + YEL_MARGIN)) &&      // G >= B+M
                (px[15:8] >= YEL_MIN_LUM)                           &&      // 亮度门槛
                (({1'b0,px[23:16]} >= {1'b0,px[15:8]}) ?                    // |R-G| <= TOL
                 (({1'b0,px[23:16]} - {1'b0,px[15:8]}) <= YEL_RG_TOL) :
                 (({1'b0,px[15:8]}  - {1'b0,px[23:16]}) <= YEL_RG_TOL)) && // 尺度无关的黄色/肤色判据
                ((({1'b0,px[15:8]} - {1'b0,px[7:0]}) * 4) >=
                 (({1'b0,px[23:16]} - {1'b0,px[7:0]}) * 3));
        end
    endfunction

    localparam [23:0] DET_MIN_PIX = 24'd800;    // 最小命中像素数

    wire h0 = yellow_hit(S_det_data[95:72]);
    wire h1 = yellow_hit(S_det_data[71:48]);
    wire h2 = yellow_hit(S_det_data[47:24]);
    wire h3 = yellow_hit(S_det_data[23:0]);

    // ==================================================================
    // 黄色检测开关
    //   置 0：检测逻辑全部被综合优化掉，时序干净（SWNS 为正），
    //         显示通路只剩 白平衡 + 算法切换，是当前可交付的状态。
    //   置 1：功能完整，但实测 SWNS = -3.407ns 违例。原因是
    //         yellow_hit(4 份组合逻辑) -> det_cnt_beat -> det_any ->
    //         外接框比较 -> 计数器，全挤在同一个 9.66ns 周期里。
    //         要启用必须先打流水：yellow_hit 输出加一级寄存器，
    //         S_det_x / S_det_y 同步延迟一拍再喂给外接框比较。
    // ==================================================================
    localparam DET_EN = 1'b0;

    wire [2:0]  det_cnt_beat = DET_EN ? ({2'b00,h0} + {2'b00,h1} + {2'b00,h2} + {2'b00,h3}) : 3'd0;

    // 外接框只在"本拍至少 2 个像素命中"时才扩张 —— 相当于沿 x 方向做了一次
    // 腐蚀：孤立噪点和色边（通常只有 1 个像素）不会再把框撑到全屏。
    wire        det_any      = DET_EN & (det_cnt_beat >= 3'd2);

    // ---- 中心 64x64 区域平均色：白平衡标定 / 单色目标取样 ----
    //   x 608~671（16 拍），y 328~391（64 行），共 4096 像素，
    //   平均 = 累加值 >> 12，没有除法器。
    //
    //   取样坐标 S_det_x / S_det_y 由下面的拍计数维护（不再依赖 tuser/tlast），
    //   所以这个框是可靠的。把被测物体放到画面正中，读数就是它自己的颜色：
    //   黄卷尺应当读成 R、G 明显高于 B（例如 R190 G180 B090），
    //   若读成 R≈G≈B，说明颜色在到达这里之前就已经被中和掉了。
    wire S_in_center = (S_det_x >= 12'd608) && (S_det_x < 12'd672) &&
                       (S_det_y >= 12'd328) && (S_det_y < 12'd392);

    reg  [20:0] S_sum_r, S_sum_g, S_sum_b;
    reg         S_frm_d;
    reg         S_tuser_evt;                    // 帧尾事件（下面检测状态机产生）

    wire [9:0] S_beat_r = S_det_data[95:88] + S_det_data[71:64]
                        + S_det_data[47:40] + S_det_data[23:16];
    wire [9:0] S_beat_g = S_det_data[87:80] + S_det_data[63:56]
                        + S_det_data[39:32] + S_det_data[15:8];
    wire [9:0] S_beat_b = S_det_data[79:72] + S_det_data[55:48]
                        + S_det_data[31:24] + S_det_data[7:0];

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
            S_frm_d <= S_tuser_evt;

            if (S_frm_d) begin
                O_rgb_r <= S_sum_r[19:12];
                O_rgb_g <= S_sum_g[19:12];
                O_rgb_b <= S_sum_b[19:12];
                S_sum_r <= 21'd0;
                S_sum_g <= 21'd0;
                S_sum_b <= 21'd0;
            end
            else if (S_det_valid && S_in_center) begin
                S_sum_r <= S_sum_r + {11'd0, S_beat_r};
                S_sum_g <= S_sum_g + {11'd0, S_beat_g};
                S_sum_b <= S_sum_b + {11'd0, S_beat_b};
            end
        end
    end

    reg [11:0] S_det_x;                         // 当前拍首像素的 x
    reg [11:0] S_det_y;                         // 行号
    reg [11:0] S_dminx, S_dmaxx, S_dminy, S_dmaxy;
    reg [23:0] S_dcnt;

    // 帧起始把统计结果锁存到求值寄存器，之后用流水算出形状判据。
    // 若把 w*h 乘法、x3/x5 缩放、比较全挤在帧起始那一拍，
    // 实测 10.5ns 超过 9.66ns 预算（-1.045ns 违例）。
    reg [11:0] S_eval_w, S_eval_h;
    reg [23:0] S_eval_cnt;
    reg [11:0] S_eval_minx, S_eval_maxx, S_eval_miny, S_eval_maxy;
    reg [23:0] S_area_r;
    reg        S_pend1, S_pend2;

    wire [11:0] S_dw = (S_dmaxx >= S_dminx) ? (S_dmaxx - S_dminx + 12'd1) : 12'd0;
    wire [11:0] S_dh = (S_dmaxy >= S_dminy) ? (S_dmaxy - S_dminy + 12'd1) : 12'd0;

    // ---- 累加：逐拍统计外接框与命中像素数 ----
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_det_x      <= 12'd0;
            S_det_y      <= 12'd0;
            S_dminx      <= 12'hfff;  S_dmaxx <= 12'd0;
            S_dminy      <= 12'hfff;  S_dmaxy <= 12'd0;
            S_dcnt       <= 24'd0;
            S_eval_w     <= 12'd0;    S_eval_h <= 12'd0;
            S_eval_cnt   <= 24'd0;
            S_eval_minx  <= 12'd0;    S_eval_maxx <= 12'd0;
            S_eval_miny  <= 12'd0;    S_eval_maxy <= 12'd0;
            S_tuser_evt  <= 1'b0;
        end
        else begin
            S_tuser_evt <= 1'b0;

            if (S_det_valid) begin
                // ---- 帧/行边界完全由拍计数自行划分，不依赖 tuser / tlast ----
                // 踩过的坑：AWB 输出的 tlast 是 (valid_d1==0)&&(valid_d2==1)，
                // 也就是 valid 的上升沿；而本工程里 valid 在整帧内近乎连续
                // （signal_delay 的 rd_valid 掉不下来），上升沿一辈子只有一次。
                // 拿它数行，y 永远停在 0 —— 外接框和中心框统计会全部失效，
                // 表现就是"有时能识别、有时完全识别不到"。
                //
                // 现在改成：每行 1280/4 = 320 拍，x 从 0 步进到 1276 后回卷并换行；
                // y 走到 719 且 x 回卷的那一拍就是帧尾，在这里锁存本帧结果。
                // 这条链路只依赖 valid，对 AWB 的 tuser/tlast 时序完全免疫。
                if ((S_det_x >= 12'd1276) && (S_det_y >= 12'd719)) begin
                    // 帧尾：把本帧统计结果交给求值流水，同时清空累加器
                    S_eval_w    <= S_dw;
                    S_eval_h    <= S_dh;
                    S_eval_cnt  <= S_dcnt;
                    S_eval_minx <= S_dminx;
                    S_eval_maxx <= S_dmaxx;
                    S_eval_miny <= S_dminy;
                    S_eval_maxy <= S_dmaxy;
                    S_tuser_evt <= 1'b1;

                    S_dminx <= 12'hfff;  S_dmaxx <= 12'd0;
                    S_dminy <= 12'hfff;  S_dmaxy <= 12'd0;
                    S_dcnt  <= 24'd0;
                    S_det_x <= 12'd0;
                    S_det_y <= 12'd0;
                end
                else begin
                    if (det_any) begin
                        if (S_det_x < S_dminx)          S_dminx <= S_det_x;
                        if (S_det_x + 12'd3 > S_dmaxx)  S_dmaxx <= S_det_x + 12'd3;
                        if (S_det_y < S_dminy)          S_dminy <= S_det_y;
                        if (S_det_y > S_dmaxy)          S_dmaxy <= S_det_y;
                    end
                    S_dcnt <= S_dcnt + {21'd0, det_cnt_beat};

                    if (S_det_x >= 12'd1276) begin
                        S_det_x <= 12'd0;
                        S_det_y <= S_det_y + 12'd1;
                    end
                    else begin
                        S_det_x <= S_det_x + 12'd4;
                    end
                end
            end
        end
    end

    // ---- 求值流水 + 时间滞后 ----
    //   单帧判据在阈值附近会反复越界，表现为外接框一闪一闪。
    //   这里用"置信度积分 + 迟滞"稳定输出：
    //     每帧检出 +8，未检出 -8（0~63 饱和）
    //     置信度 >= 24（约连续 3 帧）才点亮，< 8（约连续 3 帧丢失）才熄灭
    //   熄灭前的保持期里框停在最后一次检出的位置，不会乱跳。
    reg [5:0]  S_det_conf;
    reg        S_det_sticky;
    reg [11:0] S_box_minx, S_box_maxx, S_box_miny, S_box_maxy;

    // 形状判据：
    //   1) 命中像素数够多      —— 排除零散噪点
    //   2) 外接框填充率 >= 40%  —— 排除"散落的黄点凑出来的大框"
    //      原来是 60%，对卷尺这种细长斜放的物体偏严（斜放时外接框空角很多），
    //      会表现为"明明看得见却不出框"。
    //   3) 长宽比在 1:2 ~ 2:1   —— 排除横贯全屏的色带
    wire S_pass_now = (S_eval_cnt >= DET_MIN_PIX) &&
                      ((S_eval_cnt * 5) >= (S_area_r * 2)) &&
                      ((S_eval_w * 2 >= S_eval_h) && (S_eval_h * 2 >= S_eval_w));

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

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),

        .I_96b_frame_start  ( sel_user          ),
        .I_96b_valid        ( sel_valid         ),
        .I_96b_data         ( sel_data          ),
	
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
