module hdmi_mixer #(
    parameter H_OFFSET = 0,
    parameter V_OFFSET = 0,
    parameter IMG_WIDTH = 1600,
    parameter IMG_HEIGHT = 900,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,
    input wire[9:0]   I_fps_value,      // 实时帧率，由顶层每秒锁存一次
    input wire[1:0]   I_mode,           // 当前算法：0=原图 1=边缘 2=灰度 3=波普
    input wire[9:0]   I_lat,            // 端到端延迟（毫秒）
    input wire[11:0]  I_det_minx,       // 黄色目标外接框（像素域）
    input wire[11:0]  I_det_maxx,
    input wire[11:0]  I_det_miny,
    input wire[11:0]  I_det_maxy,
    input wire        I_det_valid,
    input wire        I_det_cand,       // 颜色+面积通过但形状未过（调试用）
    input wire[9:0]   I_det_cnt,        // 命中像素数/16（调试显示）
    input wire[7:0]   I_avg_r,          // 中心区域平均色（调试显示）
    input wire[7:0]   I_avg_g,
    input wire[7:0]   I_avg_b,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg        S_window_active_1d;
    reg        S_window_active_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    wire S_window_active;
    assign S_window_active = I_video_de &&
                             (S_x >= H_OFFSET) &&
                             (S_x < (H_OFFSET + IMG_WIDTH)) &&
                             (S_y >= V_OFFSET) &&
                             (S_y < (V_OFFSET + IMG_HEIGHT));

    // Read exactly IMG_WIDTH pixels on each of IMG_HEIGHT lines.  The HDMI
    // active area may be larger than the stored frame, in which case remaining
    // pixels are emitted as black borders without consuming DDR data.
    assign O_video_rd_en = S_window_active;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            if(I_video_user)
                S_y <= 12'd0;
            else if(I_video_de && I_video_last)
                S_y <= S_y + 12'd1;

            if(!I_video_de || I_video_last)
                S_x <= 12'd0;
            else
                S_x <= S_x + 12'd1;

            // 与输出数据（两级流水）对齐的坐标
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    // ==================================================================
    // FPS OSD —— 画面左上角显示 "FPS:<帧率>"
    //   7 个字符槽：F / P / S / : / 百位 / 十位 / 个位
    //   8x8 点阵 + 四周 1 像素描边，每字符占 10x10 字体像素
    //   字符区 210 x 30 像素，占 1280x720 有效区约 0.68%
    //   背景透明：只覆盖字形和描边像素，其余像素保持原始画面
    // ==================================================================
    localparam integer OSD_SCALE = 3;                       // 字体像素放大倍数
    localparam integer OSD_CELL  = 10;                      // 每字符 10x10 字体像素
    localparam integer OSD_ADV   = OSD_CELL * OSD_SCALE;    // 字距 30
    localparam integer OSD_W     = 8 * OSD_ADV;             // 240（最长一行 LAT:33MS）
    localparam integer OSD_ROWH  = OSD_CELL * OSD_SCALE;    // 30  单行高
    localparam integer OSD_H     = 5 * OSD_ROWH;            // 150 五行
    localparam integer OSD_TOP   = 24;                      // 上边距
    localparam integer OSD_X0    = 24;                      // 左边距

    wire S_osd_box = (S_x_2d >= OSD_X0) && (S_x_2d < (OSD_X0 + OSD_W)) &&
                     (S_y_2d >= OSD_TOP) && (S_y_2d < (OSD_TOP + OSD_H));

    wire [11:0] S_osd_ox = S_x_2d - OSD_X0;
    wire [11:0] S_osd_oy = S_y_2d - OSD_TOP;

    // 第 0 行帧率，第 1 行算法，第 2 行延迟，第 3 行 R/G，第 4 行 B
    wire [2:0]  S_osd_ridx = (S_osd_oy >= (4*OSD_ROWH)) ? 3'd4 :
                             (S_osd_oy >= (3*OSD_ROWH)) ? 3'd3 :
                             (S_osd_oy >= (2*OSD_ROWH)) ? 3'd2 :
                             (S_osd_oy >= OSD_ROWH)     ? 3'd1 : 3'd0;
    wire [11:0] S_osd_oyr  = S_osd_oy -
                             ((S_osd_ridx == 3'd4) ? 4*OSD_ROWH :
                              (S_osd_ridx == 3'd3) ? 3*OSD_ROWH :
                              (S_osd_ridx == 3'd2) ? 2*OSD_ROWH :
                              (S_osd_ridx == 3'd1) ? OSD_ROWH   : 0);

    // ---- 定位到第几个字符槽，以及槽内横坐标 ----
    reg [2:0] S_osd_slot;
    reg [6:0] S_osd_colx;
    always @(*) begin
        if      (S_osd_ox >= 12'd210) begin S_osd_slot = 3'd7; S_osd_colx = S_osd_ox - 12'd210; end
        else if (S_osd_ox >= 12'd180) begin S_osd_slot = 3'd6; S_osd_colx = S_osd_ox - 12'd180; end
        else if (S_osd_ox >= 12'd150) begin S_osd_slot = 3'd5; S_osd_colx = S_osd_ox - 12'd150; end
        else if (S_osd_ox >= 12'd120) begin S_osd_slot = 3'd4; S_osd_colx = S_osd_ox - 12'd120; end
        else if (S_osd_ox >= 12'd90 ) begin S_osd_slot = 3'd3; S_osd_colx = S_osd_ox - 12'd90;  end
        else if (S_osd_ox >= 12'd60 ) begin S_osd_slot = 3'd2; S_osd_colx = S_osd_ox - 12'd60;  end
        else if (S_osd_ox >= 12'd30 ) begin S_osd_slot = 3'd1; S_osd_colx = S_osd_ox - 12'd30;  end
        else                            begin S_osd_slot = 3'd0; S_osd_colx = S_osd_ox[6:0];     end
    end

    // ---- 帧率拆成百/十/个位（比较-减法，避免综合出除法器）----
    reg [3:0] S_dig_h;
    always @(*) begin
        if      (I_fps_value >= 10'd900) S_dig_h = 4'd9;
        else if (I_fps_value >= 10'd800) S_dig_h = 4'd8;
        else if (I_fps_value >= 10'd700) S_dig_h = 4'd7;
        else if (I_fps_value >= 10'd600) S_dig_h = 4'd6;
        else if (I_fps_value >= 10'd500) S_dig_h = 4'd5;
        else if (I_fps_value >= 10'd400) S_dig_h = 4'd4;
        else if (I_fps_value >= 10'd300) S_dig_h = 4'd3;
        else if (I_fps_value >= 10'd200) S_dig_h = 4'd2;
        else if (I_fps_value >= 10'd100) S_dig_h = 4'd1;
        else                             S_dig_h = 4'd0;
    end

    wire [9:0] S_fps_ten_in = I_fps_value - ((S_dig_h << 6) + (S_dig_h << 5) + (S_dig_h << 2));

    reg [3:0] S_dig_t;
    always @(*) begin
        if      (S_fps_ten_in >= 10'd90) S_dig_t = 4'd9;
        else if (S_fps_ten_in >= 10'd80) S_dig_t = 4'd8;
        else if (S_fps_ten_in >= 10'd70) S_dig_t = 4'd7;
        else if (S_fps_ten_in >= 10'd60) S_dig_t = 4'd6;
        else if (S_fps_ten_in >= 10'd50) S_dig_t = 4'd5;
        else if (S_fps_ten_in >= 10'd40) S_dig_t = 4'd4;
        else if (S_fps_ten_in >= 10'd30) S_dig_t = 4'd3;
        else if (S_fps_ten_in >= 10'd20) S_dig_t = 4'd2;
        else if (S_fps_ten_in >= 10'd10) S_dig_t = 4'd1;
        else                             S_dig_t = 4'd0;
    end

    wire [9:0] S_fps_one_in = S_fps_ten_in - ((S_dig_t << 3) + (S_dig_t << 1));
    wire [3:0] S_dig_o      = S_fps_one_in[3:0];

    // ---- 中心区域平均色（0~255，全量程）拆百/十/个位 ----
    //   白平衡标定需要满量程读数：显示值就是该通道的真实平均值，
    //   直接可算增益 gain = 65536 * G / 通道均值。
    function [1:0] div100_255;
        input [7:0] x;
        begin
            if      (x >= 8'd200) div100_255 = 2'd2;
            else if (x >= 8'd100) div100_255 = 2'd1;
            else                  div100_255 = 2'd0;
        end
    endfunction

    function [3:0] div10_99;
        input [6:0] x;                       // 0~99
        begin
            if      (x >= 7'd90) div10_99 = 4'd9;
            else if (x >= 7'd80) div10_99 = 4'd8;
            else if (x >= 7'd70) div10_99 = 4'd7;
            else if (x >= 7'd60) div10_99 = 4'd6;
            else if (x >= 7'd50) div10_99 = 4'd5;
            else if (x >= 7'd40) div10_99 = 4'd4;
            else if (x >= 7'd30) div10_99 = 4'd3;
            else if (x >= 7'd20) div10_99 = 4'd2;
            else if (x >= 7'd10) div10_99 = 4'd1;
            else                 div10_99 = 4'd0;
        end
    endfunction

    // 100 = 64 + 32 + 4
    wire [1:0] S_ar_h = div100_255(I_avg_r);
    wire [1:0] S_ag_h = div100_255(I_avg_g);
    wire [1:0] S_ab_h = div100_255(I_avg_b);

    wire [7:0] S_ar_r = I_avg_r - (({6'd0,S_ar_h} << 6) + ({6'd0,S_ar_h} << 5) + ({6'd0,S_ar_h} << 2));
    wire [7:0] S_ag_r = I_avg_g - (({6'd0,S_ag_h} << 6) + ({6'd0,S_ag_h} << 5) + ({6'd0,S_ag_h} << 2));
    wire [7:0] S_ab_r = I_avg_b - (({6'd0,S_ab_h} << 6) + ({6'd0,S_ab_h} << 5) + ({6'd0,S_ab_h} << 2));

    wire [3:0] S_ar_t = div10_99(S_ar_r[6:0]);
    wire [3:0] S_ag_t = div10_99(S_ag_r[6:0]);
    wire [3:0] S_ab_t = div10_99(S_ab_r[6:0]);

    wire [6:0] S_ar_m = S_ar_r[6:0] - ({3'd0,S_ar_t} << 3) - ({3'd0,S_ar_t} << 1);
    wire [6:0] S_ag_m = S_ag_r[6:0] - ({3'd0,S_ag_t} << 3) - ({3'd0,S_ag_t} << 1);
    wire [6:0] S_ab_m = S_ab_r[6:0] - ({3'd0,S_ab_t} << 3) - ({3'd0,S_ab_t} << 1);

    wire [3:0] S_ar_o = S_ar_m[3:0];
    wire [3:0] S_ag_o = S_ag_m[3:0];
    wire [3:0] S_ab_o = S_ab_m[3:0];

    // ---- 检测像素数（0~999）拆成三位，供调试显示 ----
    reg [3:0] S_dc_h;
    always @(*) begin
        if      (I_det_cnt >= 10'd900) S_dc_h = 4'd9;
        else if (I_det_cnt >= 10'd800) S_dc_h = 4'd8;
        else if (I_det_cnt >= 10'd700) S_dc_h = 4'd7;
        else if (I_det_cnt >= 10'd600) S_dc_h = 4'd6;
        else if (I_det_cnt >= 10'd500) S_dc_h = 4'd5;
        else if (I_det_cnt >= 10'd400) S_dc_h = 4'd4;
        else if (I_det_cnt >= 10'd300) S_dc_h = 4'd3;
        else if (I_det_cnt >= 10'd200) S_dc_h = 4'd2;
        else if (I_det_cnt >= 10'd100) S_dc_h = 4'd1;
        else                           S_dc_h = 4'd0;
    end

    wire [9:0] S_dc_ten_in = I_det_cnt - ((S_dc_h << 7) + (S_dc_h << 6) + (S_dc_h << 5) + (S_dc_h << 2));

    reg [3:0] S_dc_t;
    always @(*) begin
        if      (S_dc_ten_in >= 10'd90) S_dc_t = 4'd9;
        else if (S_dc_ten_in >= 10'd80) S_dc_t = 4'd8;
        else if (S_dc_ten_in >= 10'd70) S_dc_t = 4'd7;
        else if (S_dc_ten_in >= 10'd60) S_dc_t = 4'd6;
        else if (S_dc_ten_in >= 10'd50) S_dc_t = 4'd5;
        else if (S_dc_ten_in >= 10'd40) S_dc_t = 4'd4;
        else if (S_dc_ten_in >= 10'd30) S_dc_t = 4'd3;
        else if (S_dc_ten_in >= 10'd20) S_dc_t = 4'd2;
        else if (S_dc_ten_in >= 10'd10) S_dc_t = 4'd1;
        else                            S_dc_t = 4'd0;
    end

    wire [9:0] S_dc_one_in = S_dc_ten_in - ((S_dc_t << 3) + (S_dc_t << 1));
    wire [3:0] S_dc_o     = S_dc_one_in[3:0];

    // ---- 延迟毫秒值拆成十位/个位（0~99，比较-减法，无除法器）----
    reg [3:0] S_lat_t;
    always @(*) begin
        if      (I_lat >= 10'd90) S_lat_t = 4'd9;
        else if (I_lat >= 10'd80) S_lat_t = 4'd8;
        else if (I_lat >= 10'd70) S_lat_t = 4'd7;
        else if (I_lat >= 10'd60) S_lat_t = 4'd6;
        else if (I_lat >= 10'd50) S_lat_t = 4'd5;
        else if (I_lat >= 10'd40) S_lat_t = 4'd4;
        else if (I_lat >= 10'd30) S_lat_t = 4'd3;
        else if (I_lat >= 10'd20) S_lat_t = 4'd2;
        else if (I_lat >= 10'd10) S_lat_t = 4'd1;
        else                      S_lat_t = 4'd0;
    end

    wire [9:0] S_lat_o_in = I_lat - ((S_lat_t << 3) + (S_lat_t << 1));
    wire [3:0] S_lat_o    = S_lat_o_in[3:0];

    // ---- 每个槽取哪一字模 ----
    //   0=F 1=P 2=S 3=: 4..13=0..9 15=空白
    //   16=R 17=A 18=W 19=E 20=D 21=G 22=Y 23=O 24=L 25=T
    reg [4:0] S_osd_ch;
    always @(*) begin
        case (S_osd_ridx)
            3'd1: begin
                // 第 1 行：算法名  0=RAW  1=EDGE  2=GRAY  3=POP
                case (S_osd_slot)
                    3'd0: S_osd_ch = (I_mode == 2'd0) ? 5'd16 :
                                     (I_mode == 2'd1) ? 5'd19 :
                                     (I_mode == 2'd2) ? 5'd21 : 5'd1;    // R / E / G / P
                    3'd1: S_osd_ch = (I_mode == 2'd0) ? 5'd17 :
                                     (I_mode == 2'd1) ? 5'd20 :
                                     (I_mode == 2'd2) ? 5'd16 : 5'd23;   // A / D / R / O
                    3'd2: S_osd_ch = (I_mode == 2'd0) ? 5'd18 :
                                     (I_mode == 2'd1) ? 5'd21 :
                                     (I_mode == 2'd2) ? 5'd17 : 5'd1;    // W / G / A / P
                    3'd3: S_osd_ch = (I_mode == 2'd1) ? 5'd19 :
                                     (I_mode == 2'd2) ? 5'd22 : 5'd15;   // E / Y / 空白
                    default: S_osd_ch = 5'd15;
                endcase
            end
            3'd3: begin
                // 第 3 行：中心区域 R 均值 + G 均值（满量程 0~255，供白平衡标定）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd16;               // R
                    3'd1:    S_osd_ch = 5'd4 + S_ar_h;       // 百位
                    3'd2:    S_osd_ch = 5'd4 + S_ar_t;       // 十位
                    3'd3:    S_osd_ch = 5'd4 + S_ar_o;       // 个位
                    3'd4:    S_osd_ch = 5'd21;               // G
                    3'd5:    S_osd_ch = 5'd4 + S_ag_h;
                    3'd6:    S_osd_ch = 5'd4 + S_ag_t;
                    default: S_osd_ch = 5'd4 + S_ag_o;
                endcase
            end
            3'd4: begin
                // 第 4 行：中心区域 B 均值（满量程 0~255）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd27;               // B
                    3'd1:    S_osd_ch = 5'd4 + S_ab_h;
                    3'd2:    S_osd_ch = 5'd4 + S_ab_t;
                    3'd3:    S_osd_ch = 5'd4 + S_ab_o;
                    default: S_osd_ch = 5'd15;
                endcase
            end
            3'd2: begin
                // 第 2 行：延迟  LAT:xxMS（毫秒，单位必须写全，避免被当成分钟）
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd24;               // L
                    3'd1:    S_osd_ch = 5'd17;               // A
                    3'd2:    S_osd_ch = 5'd25;               // T
                    3'd3:    S_osd_ch = 5'd3;                // :
                    3'd4:    S_osd_ch = 5'd4 + S_lat_t;      // 十位
                    3'd5:    S_osd_ch = 5'd4 + S_lat_o;      // 个位
                    3'd6:    S_osd_ch = 5'd26;               // M
                    default: S_osd_ch = 5'd2;                // S
                endcase
            end
            default: begin
                // 第 0 行：FPS:xxx
                case (S_osd_slot)
                    3'd0:    S_osd_ch = 5'd0;
                    3'd1:    S_osd_ch = 5'd1;
                    3'd2:    S_osd_ch = 5'd2;
                    3'd3:    S_osd_ch = 5'd3;
                    3'd4:    S_osd_ch = (I_fps_value >= 10'd100) ? (5'd4 + S_dig_h) : 5'd15;
                    3'd5:    S_osd_ch = (I_fps_value >= 10'd10 ) ? (5'd4 + S_dig_t) : 5'd15;
                    3'd6:    S_osd_ch = 5'd4 + S_dig_o;
                    default: S_osd_ch = 5'd15;   // 第 8 格保持空白
                endcase
            end
        endcase
    end

    // 字符槽内坐标：0..9，字形本体位于 1..8，第 0 列/行和第 9 列/行留给描边
    wire [3:0] S_osd_cellx = S_osd_colx / OSD_SCALE;      // 0..9
    wire [3:0] S_osd_celly = S_osd_oyr[6:0] / OSD_SCALE;  // 0..9

    // 字形像素坐标（15 = 越界，取 0）
    wire [3:0] S_gx  = ((S_osd_cellx >= 4'd1) && (S_osd_cellx <= 4'd8)) ? (S_osd_cellx - 4'd1) : 4'd15;
    wire [3:0] S_gxm = ((S_osd_cellx >= 4'd2) && (S_osd_cellx <= 4'd9)) ? (S_osd_cellx - 4'd2) : 4'd15;
    wire [3:0] S_gxp = (S_osd_cellx <= 4'd7) ? S_osd_cellx : 4'd15;

    wire [3:0] S_gy  = ((S_osd_celly >= 4'd1) && (S_osd_celly <= 4'd8)) ? (S_osd_celly - 4'd1) : 4'd15;
    wire [3:0] S_gym = ((S_osd_celly >= 4'd2) && (S_osd_celly <= 4'd9)) ? (S_osd_celly - 4'd2) : 4'd15;
    wire [3:0] S_gyp = (S_osd_celly <= 4'd7) ? S_osd_celly : 4'd15;

    // ---- 8x8 点阵字模，bit0 为最左像素 ----
    function [7:0] osd_font;
        input [4:0] ch;
        input [2:0] row;
        begin
            osd_font = 8'h00;
            case (ch)
                4'd0: case (row)   // F
                    3'd0: osd_font = 8'h3F; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h03; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h03; default: osd_font = 8'h00;
                endcase
                4'd1: case (row)   // P
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h63;
                    3'd4: osd_font = 8'h1F; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h03; default: osd_font = 8'h00;
                endcase
                4'd2: case (row)   // S
                    3'd0: osd_font = 8'h1E; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h1E;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h60;
                    3'd6: osd_font = 8'h1E; default: osd_font = 8'h00;
                endcase
                4'd3: case (row)   // :
                    3'd1: osd_font = 8'h0C; 3'd2: osd_font = 8'h0C;
                    3'd4: osd_font = 8'h0C; 3'd5: osd_font = 8'h0C;
                    default: osd_font = 8'h00;
                endcase
                4'd4: case (row)   // 0
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h6E; 3'd3: osd_font = 8'h76;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd5: case (row)   // 1
                    3'd0: osd_font = 8'h18; 3'd1: osd_font = 8'h1C;
                    3'd2: osd_font = 8'h18; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h7E; default: osd_font = 8'h00;
                endcase
                4'd6: case (row)   // 2
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h0C; 3'd5: osd_font = 8'h06;
                    3'd6: osd_font = 8'h7E; default: osd_font = 8'h00;
                endcase
                4'd7: case (row)   // 3
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h38;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd8: case (row)   // 4
                    3'd0: osd_font = 8'h18; 3'd1: osd_font = 8'h38;
                    3'd2: osd_font = 8'h6C; 3'd3: osd_font = 8'h33;
                    3'd4: osd_font = 8'h7E; 3'd5: osd_font = 8'h60;
                    3'd6: osd_font = 8'h60; default: osd_font = 8'h00;
                endcase
                4'd9: case (row)   // 5
                    3'd0: osd_font = 8'h7E; 3'd1: osd_font = 8'h06;
                    3'd2: osd_font = 8'h3E; 3'd3: osd_font = 8'h60;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd10: case (row)  // 6
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h06;
                    3'd2: osd_font = 8'h06; 3'd3: osd_font = 8'h3E;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd11: case (row)  // 7
                    3'd0: osd_font = 8'h7E; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h60; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h18; default: osd_font = 8'h00;
                endcase
                4'd12: case (row)  // 8
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h66; 3'd3: osd_font = 8'h3C;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                4'd13: case (row)  // 9
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h66; 3'd3: osd_font = 8'h7C;
                    3'd4: osd_font = 8'h60; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd16: case (row)  // R
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h1B; 3'd5: osd_font = 8'h33;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd17: case (row)  // A
                    3'd0: osd_font = 8'h1C; 3'd1: osd_font = 8'h36;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h63;
                    3'd4: osd_font = 8'h7F; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd18: case (row)  // W
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h6B;
                    3'd4: osd_font = 8'h6B; 3'd5: osd_font = 8'h36;
                    3'd6: osd_font = 8'h22; default: osd_font = 8'h00;
                endcase
                5'd19: case (row)  // E
                    3'd0: osd_font = 8'h7F; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h03; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h7F; default: osd_font = 8'h00;
                endcase
                5'd20: case (row)  // D
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h33;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h63;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h33;
                    3'd6: osd_font = 8'h1F; default: osd_font = 8'h00;
                endcase
                5'd21: case (row)  // G
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h73;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd22: case (row)  // Y
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h36; 3'd3: osd_font = 8'h1C;
                    3'd4: osd_font = 8'h08; 3'd5: osd_font = 8'h08;
                    3'd6: osd_font = 8'h08; default: osd_font = 8'h00;
                endcase
                5'd23: case (row)  // O
                    3'd0: osd_font = 8'h3C; 3'd1: osd_font = 8'h66;
                    3'd2: osd_font = 8'h66; 3'd3: osd_font = 8'h66;
                    3'd4: osd_font = 8'h66; 3'd5: osd_font = 8'h66;
                    3'd6: osd_font = 8'h3C; default: osd_font = 8'h00;
                endcase
                5'd24: case (row)  // L
                    3'd0: osd_font = 8'h03; 3'd1: osd_font = 8'h03;
                    3'd2: osd_font = 8'h03; 3'd3: osd_font = 8'h03;
                    3'd4: osd_font = 8'h03; 3'd5: osd_font = 8'h03;
                    3'd6: osd_font = 8'h7E; default: osd_font = 8'h00;
                endcase
                5'd25: case (row)  // T
                    3'd0: osd_font = 8'h7E; 3'd1: osd_font = 8'h18;
                    3'd2: osd_font = 8'h18; 3'd3: osd_font = 8'h18;
                    3'd4: osd_font = 8'h18; 3'd5: osd_font = 8'h18;
                    3'd6: osd_font = 8'h18; default: osd_font = 8'h00;
                endcase
                5'd26: case (row)  // M
                    3'd0: osd_font = 8'h63; 3'd1: osd_font = 8'h77;
                    3'd2: osd_font = 8'h7F; 3'd3: osd_font = 8'h6B;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h63; default: osd_font = 8'h00;
                endcase
                5'd27: case (row)  // B
                    3'd0: osd_font = 8'h1F; 3'd1: osd_font = 8'h63;
                    3'd2: osd_font = 8'h63; 3'd3: osd_font = 8'h1F;
                    3'd4: osd_font = 8'h63; 3'd5: osd_font = 8'h63;
                    3'd6: osd_font = 8'h1F; default: osd_font = 8'h00;
                endcase
                default: osd_font = 8'h00;
            endcase
        end
    endfunction

    // ---- 取某一行字模（越界返回 0）----
    function [7:0] glyph_row;
        input [4:0] ch;
        input [3:0] y;
        begin
            if (y > 4'd7) glyph_row = 8'h00;
            else          glyph_row = osd_font(ch, y[2:0]);
        end
    endfunction

    wire [15:0] S_row_c = {8'b0, glyph_row(S_osd_ch, S_gy )};
    wire [15:0] S_row_m = {8'b0, glyph_row(S_osd_ch, S_gym)};
    wire [15:0] S_row_p = {8'b0, glyph_row(S_osd_ch, S_gyp)};

    wire S_pix_c  = S_row_c[S_gx ];
    wire S_pix_l  = S_row_c[S_gxm];
    wire S_pix_r  = S_row_c[S_gxp];
    wire S_pix_u  = S_row_m[S_gx ];
    wire S_pix_d  = S_row_p[S_gx ];
    wire S_pix_ul = S_row_m[S_gxm];
    wire S_pix_ur = S_row_m[S_gxp];
    wire S_pix_dl = S_row_p[S_gxm];
    wire S_pix_dr = S_row_p[S_gxp];

    wire S_osd_green = S_pix_c;                       // 字形：亮绿
    wire S_osd_black = ~S_pix_c &&                    // 描边：8 邻域有字形则涂黑
                       (S_pix_l | S_pix_r | S_pix_u | S_pix_d |
                        S_pix_ul | S_pix_ur | S_pix_dl | S_pix_dr);

    // ==================================================================
    // 安路 Logo（基础要求 2.3(4)）：右上角 160x160 位图
    //   ROM 像素 25bit = {不透明标志[24], RGB[23:0]}
    //   透明像素(bit24=0)不覆盖画面，因此只有图案本身出现
    // ==================================================================
    localparam integer LOGO_W = 160;
    localparam integer LOGO_H = 160;
    localparam integer LOGO_X = IMG_WIDTH - LOGO_W;   // 1280 - 160 = 1120
    localparam integer LOGO_Y = 0;

    wire S_logo_box = (S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
                      (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H));

    wire [14:0] S_logo_addr = ((S_y_2d - LOGO_Y) * LOGO_W) + (S_x_2d - LOGO_X);
    wire [24:0] S_logo_pixel;

    anlogic_logo_rom u_anlogic_logo_rom (
        .I_addr  ( S_logo_addr  ),
        .O_pixel ( S_logo_pixel )
    );

    // ==================================================================
    // 黄色目标外接框：绿色 3 像素边框
    // ==================================================================
    localparam integer BOX_T = 3;

    // 绿色 = 颜色/面积/形状全部通过；橙色 = 颜色和面积过了但形状没通过，
    // 用于现场判断是"颜色判据漏检"还是"形状判据太严"
    wire S_in_box = (I_det_valid || I_det_cand) &&
                    (S_x_2d >= I_det_minx) && (S_x_2d <= I_det_maxx) &&
                    (S_y_2d >= I_det_miny) && (S_y_2d <= I_det_maxy);

    wire S_box_edge = S_in_box &&
                      ((S_x_2d <  I_det_minx + BOX_T) ||
                       (S_x_2d + BOX_T > I_det_maxx) ||
                       (S_y_2d <  I_det_miny + BOX_T) ||
                       (S_y_2d + BOX_T > I_det_maxy));

    // ==================================================================
    // 中心取样框（标定辅助）：1 像素红色细框
    //   位置与 ISP 里 O_rgb_r/g/b 的取样区域完全一致：
    //     isp_top.v  S_in_center: x 608~671, y 328~391（64x64）
    //   把被测物体（白纸 / 黄卷尺）对进这个框，OSD 第 3、4 行的读数
    //   就是它本身的 R/G/B。演示时把 SHOW_CB 置 0 即可隐藏。
    // ==================================================================
    localparam integer CB_X0 = 608;
    localparam integer CB_Y0 = 328;
    localparam integer CB_W  = 64;
    localparam integer CB_H  = 64;
    localparam integer CB_T  = 1;                 // 线宽：1 像素
    localparam         SHOW_CB = 1'b1;

    wire S_cb_edge = (S_x_2d >= CB_X0) && (S_x_2d < CB_X0 + CB_W) &&
                     (S_y_2d >= CB_Y0) && (S_y_2d < CB_Y0 + CB_H) &&
                     ((S_x_2d <  CB_X0 + CB_T) ||
                      (S_x_2d >= CB_X0 + CB_W - CB_T) ||
                      (S_y_2d <  CB_Y0 + CB_T) ||
                      (S_y_2d >= CB_Y0 + CB_H - CB_T));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_window_active_1d <= 1'b0;
            S_window_active_2d <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_window_active_1d <= S_window_active;
            S_window_active_2d <= S_window_active_1d;
            O_hdmi_de        <= S_video_de_2d;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 4) begin
                // Full-screen MIPI receive diagnostic.  The status is
                // generated and synchronized in design_top_wrapper.
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'd0: O_hdmi_data <= 24'h000080; // blue: sensor config incomplete
                        4'd1: O_hdmi_data <= 24'h000000; // black: no MIPI HS traffic
                        4'd2: O_hdmi_data <= 24'h00ffff; // cyan: HS present, no complete frame
                        4'd3: O_hdmi_data <= 24'hff0000; // red: D-PHY lane error
                        4'd4: O_hdmi_data <= 24'hff00ff; // magenta: RAW10 line length error
                        4'd5: O_hdmi_data <= 24'hffff00; // yellow: not 720 lines per frame
                        4'd6: O_hdmi_data <= 24'h00ff00; // green: MIPI frame structure passed
                        4'd7: O_hdmi_data <= 24'hff8000; // orange: sensor I2C transaction error
                        default: O_hdmi_data <= 24'hffffff;
                    endcase
                    // 诊断页也叠加 FPS，便于同时确认帧率
                    if(S_osd_box) begin
                        if(S_osd_green)
                            O_hdmi_data <= 24'h00ff00;
                        else if(S_osd_black)
                            O_hdmi_data <= 24'h000000;
                    end
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_video_de_2d) begin
                    O_hdmi_data <= S_window_active_2d ? S_video_data_2d : 24'd0;
                    // 黄色目标外接框：绿=完全通过，橙=形状未通过
                    if(S_box_edge)
                        O_hdmi_data <= I_det_valid ? 24'h00ff00 : 24'hff8000;
                    // 中心取样框：红色细框（标定用，和被检测框重叠时让检测框优先）
                    else if(SHOW_CB && S_cb_edge)
                        O_hdmi_data <= 24'hff0000;
                    // 安路 Logo：右上角，仅非透明像素覆盖
                    else if(S_logo_box && S_logo_pixel[24])
                        O_hdmi_data <= S_logo_pixel[23:0];
                    // FPS OSD：背景透明，仅字形（亮绿）和描边（黑）覆盖画面
                    else if(S_osd_box) begin
                        if(S_osd_green)
                            O_hdmi_data <= 24'h00ff00;
                        else if(S_osd_black)
                            O_hdmi_data <= 24'h000000;
                    end
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
        end
    end

endmodule
