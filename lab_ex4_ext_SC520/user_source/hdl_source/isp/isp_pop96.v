// ==========================================================================
// POP 波普风格（96bit/拍，4 像素）：亮度 4 档 + 假色彩 + 套印错位
// --------------------------------------------------------------------------
// 从 isp_top 内联逻辑抽出，设计与单元测试共用同一份 RTL。
//   · 分档：灰度 Y = R/4 + G/2 + B/4，线性光域阈值 12/56/135（满量程 253）
//   · 配色：4 组调色板 × 4 档，平涂色块，保持清晰阶梯
//   · 套印：中间调（档 1/2）按 x-2 的档位着色，形成向右 2 像素的套色毛边
//
// HUE_AWARE=1（本轮 isp_top 已启用）：
//   同一亮度档内，按源像素主色相（暖/绿/冷）把平涂色混向对应的色相锚点：
//       out = base/4 + 3*anchor/4，anchor 再按亮度档缩放。
//   使"手（暖色）与绿球（绿色）在 POP 里不再是同一块颜色"。
//   档位仍由亮度决定，4 档明暗顺序不变；中性像素（低色度）完全保持原调色板。
//   这是风格层的色相区分，不是物体识别。只做移位/加减，无乘法器。
// ==========================================================================
module isp_pop96 #(
    parameter        HUE_AWARE = 1'b0,
    // 1 = 亮度档使用顶层共享的 4 像素 Y（I_y4），0 = 模块内部自行计算。
    // 两种路径的公式完全相同：Y = R/4 + G/2 + B/4。
    parameter        USE_SHARED_Y = 1'b0,
    parameter [7:0]  T0 = 8'd12,
    parameter [7:0]  T1 = 8'd56,
    parameter [7:0]  T2 = 8'd135
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tvalid,
    input  wire        I_tuser,       // 帧首
    input  wire        I_tlast,       // 行尾
    input  wire [95:0] I_tdata,       // 白平衡后 4 像素 RGB888（色相仍用 RGB）
    input  wire [31:0] I_y4,          // 共享亮度 {Y3,Y2,Y1,Y0}，USE_SHARED_Y=1 时有效
    input  wire [1:0]  I_pal,         // 配色方案
    output wire [95:0] O_tdata        // POP 输出（整体相对 I_tdata 延迟 1 拍）
);

    // ------------------------------------------------------------------
    // 流水拆分（时序修复）：
    //   级 1：I_tdata -> 亮度档 / 色相分类 -> 寄存器 b_q/h_q/pal_q
    //   级 2：寄存器 -> 调色板 + tint -> O_tdata（组合，由调用方 pop_d1 寄存）
    // 原实现把"Y 计算 + 档位比较 + 色相分类 + 调色板 + tint"全部压在一拍，
    // 实测 12 级、10.85 ns（WNS -1.255 @9.66 ns）。拆开后两级均有余量。
    // 套印：级 2 用 b_q（当前输出拍的档位）和 b_qq（上一个有效拍档位）
    //       计算 x-2；只在 I_tvalid 拍推进，行首/帧首强制不使用上一行历史。
    // ------------------------------------------------------------------

    function [7:0] rgb_to_y8;
        input [23:0] px;
        begin
            rgb_to_y8 = {2'b0,px[23:18]} + {1'b0,px[15:9]} + {2'b0,px[7:2]};
        end
    endfunction

    function [1:0] pop_band;
        input [7:0] g;
        begin
            if      (g < T0) pop_band = 2'd0;
            else if (g < T1) pop_band = 2'd1;
            else if (g < T2) pop_band = 2'd2;
            else             pop_band = 2'd3;
        end
    endfunction

    // 源像素主色相（0=中性，1=暖/红，2=绿，3=冷/蓝）
    function [1:0] hue_of;
        input [23:0] p;
        reg [7:0] r, g, b, mx, mn;
        begin
            r = p[23:16]; g = p[15:8]; b = p[7:0];
            mx = (r > g) ? r : g;  mx = (mx > b) ? mx : b;
            mn = (r < g) ? r : g;  mn = (mn < b) ? mn : b;
            // 低色度保持中性：灰色噪声 chroma 通常 < 10；门限 16 可覆盖
            // 较暗但仍明显有色的像素（旧值 24 会把暗绿/暗暖判成中性）。
            if ((mx - mn) < 8'd16)      hue_of = 2'd0;
            else if (r >= g && r >= b)  hue_of = 2'd1;
            else if (g >= r && g >= b)  hue_of = 2'd2;
            else                        hue_of = 2'd3;
        end
    endfunction

    // 色相锚点：与调色板无关，保证同亮度档内暖/绿/冷三类的差异足够大。
    // 旧实现只在调色板色上做 ±24 偏移；当基色已到 (255,212,0) 时，
    // 暖色 +24/冷色 -24 会饱和，手与球只剩 ΔR=16/ΔG=24，肉眼区分不足。
    function [23:0] hue_anchor;
        input [1:0] h;
        begin
            case (h)
                2'd1: hue_anchor = 24'hff4000;   // 暖：橙红（压低 G，保证 R 主导）
                2'd2: hue_anchor = 24'h00ff80;   // 绿
                2'd3: hue_anchor = 24'h0040ff;   // 冷：蓝（G 压低，保证 B 主导）
                default: hue_anchor = 24'h000000;
            endcase
        end
    endfunction

    // 按亮度档缩放锚点，保持 4 档明暗顺序（档 0 最暗 .. 档 3 最亮）
    function [7:0] band_scale;
        input [7:0] a;
        input [1:0] b;
        reg [8:0] t;
        begin
            case (b)
                2'd0: t = {1'b0, a} >> 2;
                2'd1: t = {1'b0, a} >> 1;
                2'd2: t = {1'b0, a};
                default: t = {1'b0, a} + ({1'b0, a} >> 1);
            endcase
            band_scale = (t > 9'd255) ? 8'd255 : t[7:0];
        end
    endfunction

    // 平涂 + 色相：out = base/4 + 3*anchor/4。
    // 只做移位/加减（3x/4 = x - x/4），无乘法器；中性(h=0) 原样返回，
    // 因此灰/白/黑场景的观感与旧版完全一致。
    function [23:0] hue_tint;
        input [23:0] c;
        input [1:0]  b;
        input [1:0]  h;
        reg [23:0] a;
        reg [7:0]  ar, ag, ab;
        reg [9:0]  r, g, bb;
        begin
            if (!HUE_AWARE || (h == 2'd0)) begin
                hue_tint = c;
            end
            else begin
                a  = hue_anchor(h);
                ar = band_scale(a[23:16], b);
                ag = band_scale(a[15:8],  b);
                ab = band_scale(a[7:0],   b);
                r  = ({2'b0, c[23:16]} >> 2) + {2'b0, ar} - ({2'b0, ar} >> 2);
                g  = ({2'b0, c[15:8]}  >> 2) + {2'b0, ag} - ({2'b0, ag} >> 2);
                bb = ({2'b0, c[7:0]}   >> 2) + {2'b0, ab} - ({2'b0, ab} >> 2);
                hue_tint = {r[7:0], g[7:0], bb[7:0]};
            end
        end
    endfunction

    function [23:0] pop_pal;
        input [1:0] idx;
        input [1:0] pal;
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
                default: case (idx)
                    2'd0: pop_pal = 24'h7b2d8e;
                    2'd1: pop_pal = 24'hff9ec4;
                    2'd2: pop_pal = 24'hffd400;
                    default: pop_pal = 24'h00c8c8;
                endcase
            endcase
        end
    endfunction

    // ---------------- 级 1：当前输入拍 4 像素亮度档 + 色相分类 ----------------
    // 共享路径与本地路径的位序一致：bit[31:24] 是最左像素。
    wire [31:0] y_local = { rgb_to_y8(I_tdata[95:72]),
                            rgb_to_y8(I_tdata[71:48]),
                            rgb_to_y8(I_tdata[47:24]),
                            rgb_to_y8(I_tdata[23:0]) };
    wire [31:0] y_use = USE_SHARED_Y ? I_y4 : y_local;

    wire [1:0] cb0 = pop_band(y_use[31:24]);
    wire [1:0] cb1 = pop_band(y_use[23:16]);
    wire [1:0] cb2 = pop_band(y_use[15:8]);
    wire [1:0] cb3 = pop_band(y_use[7:0]);
    wire [1:0] ch0 = hue_of(I_tdata[95:72]);
    wire [1:0] ch1 = hue_of(I_tdata[71:48]);
    wire [1:0] ch2 = hue_of(I_tdata[47:24]);
    wire [1:0] ch3 = hue_of(I_tdata[23:0]);

    wire [7:0] bands_cur = { cb3, cb2, cb1, cb0 };
    wire [7:0] hues_cur  = { ch3, ch2, ch1, ch0 };

    reg [7:0] b_q, h_q, b_qq;
    reg [1:0] pal_q;
    reg       prev_last, first_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            b_q       <= 8'd0;
            h_q       <= 8'd0;
            b_qq      <= 8'd0;
            pal_q     <= 2'd0;
            prev_last <= 1'b0;
            first_q   <= 1'b0;
        end
        else if (I_tvalid) begin
            // 只在有效拍推进：消隐/无效拍不会把套印历史错位
            b_q       <= bands_cur;
            h_q       <= hues_cur;
            b_qq      <= b_q;          // 上一个"有效拍"的档位（用于 x-2 套印）
            pal_q     <= I_pal;
            first_q   <= I_tuser | prev_last;  // 本拍是否为行首/帧首
            prev_last <= I_tlast;
        end
    end
    wire line_start = first_q;

    // ---------------- 级 2：套印 + 调色板 + 色相 tint ----------------
    wire [1:0] b0 = b_q[1:0];
    wire [1:0] b1 = b_q[3:2];
    wire [1:0] b2 = b_q[5:4];
    wire [1:0] b3 = b_q[7:6];
    wire [1:0] h0 = h_q[1:0];
    wire [1:0] h1 = h_q[3:2];
    wire [1:0] h2 = h_q[5:4];
    wire [1:0] h3 = h_q[7:6];

    // 行首/帧首：像素 0/1 不能取上一行最后一拍的 x-2 档位
    wire [1:0] s0 = line_start ? 2'd0 : b_qq[5:4];
    wire [1:0] s1 = line_start ? 2'd0 : b_qq[7:6];
    wire [1:0] s2 = b0;
    wire [1:0] s3 = b1;

    wire [1:0] e0 = ((s0 == 2'd1) || (s0 == 2'd2)) ? s0 : b0;
    wire [1:0] e1 = ((s1 == 2'd1) || (s1 == 2'd2)) ? s1 : b1;
    wire [1:0] e2 = ((s2 == 2'd1) || (s2 == 2'd2)) ? s2 : b2;
    wire [1:0] e3 = ((s3 == 2'd1) || (s3 == 2'd2)) ? s3 : b3;

    assign O_tdata = { hue_tint(pop_pal(e0, pal_q), e0, h0),
                       hue_tint(pop_pal(e1, pal_q), e1, h1),
                       hue_tint(pop_pal(e2, pal_q), e2, h2),
                       hue_tint(pop_pal(e3, pal_q), e3, h3) };

endmodule
