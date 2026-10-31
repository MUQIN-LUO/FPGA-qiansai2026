// ==========================================================================
// 连通域 + 圆形判据（时序优化版 v2）
// --------------------------------------------------------------------------
// 相对 dcfcacb 版本，功能语义不变，改动如下：
//   [T1] 上一行重叠 ov1/ov2 预寄存：用 cx+1 提前一拍算好，关闭时取寄存器；
//        cx==0 的跨行拍回退组合值（跨行首拍仍可能关闭，见语义说明）。
//   [T2] 空闲标签号 free_lab 预寄存：关闭事件间隔 >=2 拍，滞后一拍取号等价。
//   [T3] 帧尾评估三级流水：取指标 -> 面积 -> 判据，并整体延后 2 拍启动，
//        给关闭写回留出时间。输出值不变，O_evt 约晚 20 拍（帧消隐期内）。
//   [T4] 溢出哨兵修复：par[0] 与 O_ovf 复位；pick==0 时不读 par[0]。
//        （原版 par[0] 无复位，仿真为 X，标签耗尽时 O_ovf 不确定。）
//   [T5] 关闭处理两段式：
//        A 段（关闭拍）：解析 keep/drop，立即更新并查集指针、cr_*/pr_* 行表，
//                        把 keep/drop/run_s/close_e/y/last_c 寄存到 P_*；
//        B 段（下一拍）：从标签数组 gather、合并、写回统计。
//        关闭事件物理间隔 >=2 拍（一次关闭后下一拍 in_run 必为 0），
//        因此单级 pending 不会丢事件；行尾结算对 P_keep/P_drop 让路，
//        由 B 段补做，保证与单拍版本的统计结果一致。
// --------------------------------------------------------------------------
// 吞吐/同步语义：
//   · 每拍 1 格（4 像素），与像素流 1:1；关闭事件 >=2 拍一次，A/B 两段
//     交替执行，无背压、无丢帧。
//   · 跨行：pr_* 在行末由 cr_* 载入；行末关闭的游程直接补写 pr_*。
//   · 跨帧：帧尾评估结束（S_ev_fin）清空标签与游程表；评估期间 P_valid
//     必须已排空，帧尾事件由 I_valid/last_c/last_r 单拍触发。
//   · 合并：keep 取较小根号；drop 的 bbox/格数/弦长/行数并入 keep；
//     被合并标签当前行的行跨度按原版不并入（其槽位随后失效）。
//   · 溢出：标签耗尽时丢弃该游程并置 O_ovf=1（帧内累积），不静默。
// --------------------------------------------------------------------------
// [颜色一致性]（本轮新增，COLOR_AWARE=1）
//   目标：把"相互接触但颜色不同"的两个物体分到不同连通域。
//   · 输入 I_rgb 是该格 4 像素平均色（5:5:5）。
//   · 颜色跳变格处理：当整格命中且其颜色与上一整格颜色差 >= COLOR_TH 时，
//     把这一格当作"空拍"（cell_v=0）。这样游程在该格自然关闭，下一格以新
//     颜色另起游程。**不新增关闭事件类型**，因此原来"关闭事件间隔 >=2 拍"
//     的 A/B 两级流水不变量继续成立。
//   · 跨行合并：只允许颜色相近的标签合并（l_col 为该标签的代表色）。
//   · 代表色用 (3*old+new)>>2 慢速跟踪，兼顾光照渐变与颜色区分。
//   · 整格命中（I_hit==4'hF）才参与颜色判决，边缘混合格不投票。
//   · 所有颜色逻辑都在 25 MHz CCL 域，不增加 103.52 MHz 像素通路的负担。
// ==========================================================================
module isp_ccl_circle #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [11:0]  MIN_W_PX   = 12'd32,
    parameter [11:0]  MAX_W_PX   = 12'd720,
    parameter [23:0]  MIN_CELLS  = 24'd8,
    parameter [11:0]  ROW_NUM    = 12'd9,
    parameter [11:0]  ROW_DEN    = 12'd10,
    // ---- 颜色一致性 ----
    parameter        COLOR_AWARE = 1'b1,
    // 5:5:5 三通道绝对差之和门限。肤色 vs 绿色约 31（5bit 域），
    // 同一物体内光照渐变一般 < 10；取 12 兼顾区分度与抗渐变。
    parameter [6:0]  COLOR_TH    = 7'd12
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_clr,
    input  wire        I_valid,
    input  wire [3:0]  I_hit,
    input  wire [14:0] I_rgb,        // 该格 4 像素平均色 5:5:5
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    output reg         O_evt,
    output reg  [11:0] O_w,
    output reg  [11:0] O_h,
    output reg  [23:0] O_cnt,
    output reg  [20:0] O_chord,
    output reg  [10:0] O_rows,
    output reg  [11:0] O_minx,
    output reg  [11:0] O_maxx,
    output reg  [11:0] O_miny,
    output reg  [11:0] O_maxy,
    output reg         O_pass,
    output reg         O_cand,
    output reg         O_live,      // 本帧存在至少一个有效连通域（诊断用）
    output reg         O_ok_size,
    output reg         O_ok_asp,
    output reg         O_ok_rows,
    output reg         O_ok_chord,
    output reg  [3:0]  O_lab,
    output reg         O_ovf,
    output wire        O_busy        // 帧尾评估中：外部 FIFO 暂停送数
);

    localparam integer NB = 15;
    localparam integer LAST_CX = IMG_WIDTH/4 - 1;

    // ---------------- 标签状态 ----------------
    reg [3:0]  par    [0:NB];
    reg [8:0]  l_minx [1:NB], l_maxx [1:NB];
    reg [9:0]  l_miny [1:NB], l_maxy [1:NB];
    reg [19:0] l_cnt  [1:NB];
    reg [21:0] l_chord[1:NB];
    reg [10:0] l_rows [1:NB];
    reg [8:0]  l_rmnx [1:NB], l_rmxx [1:NB];
    reg        l_has  [1:NB];
    // 标签代表色（5:5:5）与有效位；仅 COLOR_AWARE=1 时参与合并判决
    reg [14:0] l_col  [1:NB];
    reg        l_col_v[1:NB];

    // ---------------- 游程表 ----------------
    reg [8:0]  pr_s [0:7], pr_e [0:7];
    reg [3:0]  pr_l [0:7];
    reg [3:0]  pr_n;
    reg [8:0]  cr_s [0:7], cr_e [0:7];
    reg [3:0]  cr_l [0:7];
    reg [3:0]  cr_n;

    reg        in_run;
    reg [8:0]  run_s;
    reg [3:0]  run_l, run_l2;
    reg [14:0] run_col;          // 当前游程的代表色（首个整格）
    reg        run_col_v;

    reg        S_ev_fin;
    reg        S_live_any;   // 帧内至少出现过一个 live 标签
    reg        S_rowpend;   // 行尾结算推迟一拍（行尾同时有关闭时）

    integer i, k;

    function [3:0] find_lab;
        input [3:0] l;
        begin
            find_lab = (l == 4'd0) ? 4'd0 : ((par[l] == 4'd0) ? l : par[l]);
        end
    endfunction

    // ---- 颜色一致性 ----
    // 5:5:5 三通道绝对差之和（最大 93，7 位）
    function [6:0] cdist5;
        input [14:0] a;
        input [14:0] b;
        reg [4:0] dr, dg, db;
        begin
            dr = (a[14:10] >= b[14:10]) ? (a[14:10] - b[14:10]) : (b[14:10] - a[14:10]);
            dg = (a[9:5]   >= b[9:5])   ? (a[9:5]   - b[9:5])   : (b[9:5]   - a[9:5]);
            db = (a[4:0]   >= b[4:0])   ? (a[4:0]   - b[4:0])   : (b[4:0]   - a[4:0]);
            cdist5 = {2'd0, dr} + {2'd0, dg} + {2'd0, db};
        end
    endfunction

    // 候选标签 l 是否与参考色 c 足够接近（无法判定时一律接受，避免过度分割）
    function col_ok;
        input [3:0]  l;
        input [14:0] c;
        input        cv;
        begin
            if (!COLOR_AWARE || !cv || (l == 4'd0) || !l_col_v[l])
                col_ok = 1'b1;
            else
                col_ok = (cdist5(c, l_col[l]) < COLOR_TH);
        end
    endfunction

    // 每通道 (3*old+new)>>2，慢速跟踪光照/渐变。
    // 注意：必须用 7 位中间和。若直接写成 (old+old+old+new)>>2，
    // Verilog 会按操作数位宽（5 位）回绕：3*22+22=88 -> 24 -> 6，
    // 代表色被打偏后所有跨行合并都会被误拒（实测标签溢出）。
    function [4:0] ema5;
        input [4:0] old;
        input [4:0] new;
        reg   [6:0] sum;
        begin
            sum  = {2'b0, old} + {2'b0, old} + {2'b0, old} + {2'b0, new};
            ema5 = sum[6:2];
        end
    endfunction

    // ---- 颜色跳变格：整格命中且与上一整格颜色差超限 ----
    reg  [14:0] S_prev_col;      // 上一个"整格命中"的颜色
    reg         S_prev_v;
    wire        cell_full = (&I_hit);
    // 镜面高光：整格、三通道都亮、通道间色差很小（接近白）。
    // 高光是同一物体表面的反光，不是另一个物体；若按颜色跳变切断游程，
    // 球会被切成上/下/左/右多块（实测 204x96、176x53 等块宽高比失败，
    // 而 72x65 的高光本身反而满足圆形判据）。因此高光格：
    //   · 不触发 col_split（继续留在当前游程里）；
    //   · 不更新 run_col（避免把代表色拉向白色）；
    //   · 不更新 S_prev_col（下一格仍以物体色为局部参考）。
    localparam [4:0] SPEC_MIN    = 5'd24;   // 120/255：明显亮于普通物体
    localparam [4:0] SPEC_CHROMA = 5'd6;    // max-min <= 6（5bit）
    wire [4:0] cell_r = I_rgb[14:10];
    wire [4:0] cell_g = I_rgb[9:5];
    wire [4:0] cell_b = I_rgb[4:0];
    wire [4:0] cell_hi = (cell_r >= cell_g && cell_r >= cell_b) ? cell_r :
                         (cell_g >= cell_b) ? cell_g : cell_b;
    wire [4:0] cell_lo = (cell_r <= cell_g && cell_r <= cell_b) ? cell_r :
                         (cell_g <= cell_b) ? cell_g : cell_b;
    wire       specular = COLOR_AWARE && cell_full &&
                          (cell_lo >= SPEC_MIN) && ((cell_hi - cell_lo) <= SPEC_CHROMA);

    // 增亮低饱和的宽判据（曾用于覆盖高光过渡环）会把手部肤色也当成高光：
    // 实测红球+手指被并成 440x190。肤色与"增亮红"在 5:5:5 上不可靠区分，
    // 因此不采用该规则。这里只保留严格镜面白核：
    //   整格、三通道都 >= SPEC_MIN、通道间色差 <= SPEC_CHROMA。
    // 高光过渡环改为在测试模型里平滑衰减（每格色差 < COLOR_TH），
    // 真实场景若出现"非白的高对比反光边"，仍可能切碎目标——已记录为边界。
    wire       no_split = specular;

    // 颜色跳变判据（双条件，跨过单个混合格）：
    //   · 与"当前游程代表色"差 >= COLOR_TH：累计色差已经明显；
    //   · 与"上一整格"差 >= COLOR_TH/2：变化是突变的，不是光照渐变。
    // 只用一个相邻格判据时，若边界恰好是 50/50 混合整格，单步差会小于阈值，
    // 红球+肤手会被桥接成一个域（实测）。只用累计判据又会把球面渐变切成多片。
    localparam [6:0] COLOR_LOCAL = (COLOR_TH >> 1);
    wire        col_split = COLOR_AWARE && in_run && cell_full && run_col_v &&
                            (I_x != 12'd0) && !no_split &&
                            (cdist5(I_rgb, run_col) >= COLOR_TH) &&
                            (!S_prev_v || (cdist5(I_rgb, S_prev_col) >= COLOR_LOCAL));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_prev_col <= 15'd0;
            S_prev_v   <= 1'b0;
        end
        else if (I_clr) begin
            S_prev_col <= 15'd0;
            S_prev_v   <= 1'b0;
        end
        else if (I_valid && cell_full && !no_split) begin
            // 跳变格本身也更新参考色：下一格与它同色时不再判跳变
            S_prev_col <= I_rgb;
            S_prev_v   <= 1'b1;
        end
    end

    reg [3:0] free_lab;
    always @(*) begin
        free_lab = 4'd0;
        for (i = NB; i >= 1; i = i - 1)
            if (par[i] == 4'd0) free_lab = i[3:0];
    end

    // [T2] 预寄存空闲标签号
    reg [3:0] free_lab_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) free_lab_q <= NB[3:0];
        else          free_lab_q <= free_lab;
    end

    // ---------------- 当前拍 ----------------
    wire [8:0] cx     = I_x[10:2];
    // 颜色跳变格按"空拍"处理：游程在此自然关闭，下一格以新颜色另起游程。
    // 这样不新增关闭事件类型，保持 A/B 两级流水的 >=2 拍间隔不变量。
    wire       cell_v   = |I_hit && ~col_split;
    wire       last_c = (I_x >= (IMG_WIDTH  - 4));
    wire       last_r = (I_y >= (IMG_HEIGHT - 1));

    // [T1] 重叠判据，可对当前格与下一格并行求值
    function [7:0] ov_pair;
        input [8:0] c;
        integer kk;
        reg [3:0] a, bb, f;
        begin
            a  = 4'd0;
            bb = 4'd0;
            for (kk = 0; kk < 8; kk = kk + 1) begin
                if ((kk < pr_n) && ((c + 9'd1) >= pr_s[kk]) && (c <= (pr_e[kk] + 9'd1))) begin
                    f = find_lab(pr_l[kk]);
                    if (f != 4'd0) begin
                        if (a == 4'd0)
                            a = f;
                        else if (f != a)
                            bb = f;
                    end
                end
            end
            ov_pair = {a, bb};
        end
    endfunction

    wire [7:0] ov_cur = ov_pair(cx);
    wire [3:0] ov1_cur = ov_cur[7:4];
    wire [3:0] ov2_cur = ov_cur[3:0];

    wire [8:0] cx_nxt = (cx == LAST_CX[8:0]) ? 9'd0 : (cx + 9'd1);
    reg  [3:0] ov1_q, ov2_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ov1_q <= 4'd0;
            ov2_q <= 4'd0;
        end
        else if (I_valid) begin
            {ov1_q, ov2_q} <= ov_pair(cx_nxt);
        end
    end

    // [T6] 行尾强制关闭后，cx==0 不可能发生关闭（in_run 已在行尾清零），
    //      因此关闭路径完全使用预寄存值，不再依赖 I_x 组合回退。
    wire [3:0] ov1_sel = ov1_q;
    wire [3:0] ov2_sel = ov2_q;

    // ---------------- 关闭判定与 A 段解析 ----------------
    // [T6] 跨行修正：最后一格"新起"的游程也必须在行尾关闭，
    //      否则 run_s 留在上一行坐标，close_e 落在新行，减法下溢、bbox 错位。
    wire        closing = (in_run && !cell_v) || (last_c && (in_run || cell_v));
    wire [8:0]  close_e = cell_v ? cx : (cx - 9'd1);
    wire [8:0]  run_s_eff  = in_run ? run_s  : cx;
    wire [3:0]  run_l_eff  = in_run ? run_l  : ov1_cur;
    wire [3:0]  run_l2_eff = in_run ? run_l2 : ov2_cur;

    // 颜色一致性：只允许与本游程代表色相近的标签参与合并。
    //   ref_col/ref_v 取游程代表色；游程没有整格时（ref_v=0）不做颜色限制。
    wire [14:0] ref_col  = in_run ? run_col : I_rgb;
    wire        ref_v    = in_run ? run_col_v : cell_full;
    wire [3:0]  run_l_f  = col_ok(run_l_eff,  ref_col, ref_v) ? run_l_eff  : 4'd0;
    wire [3:0]  run_l2_f = col_ok(run_l2_eff, ref_col, ref_v) ? run_l2_eff : 4'd0;
    wire [3:0]  ov1_f    = col_ok(ov1_sel,    ref_col, ref_v) ? ov1_sel    : 4'd0;
    wire [3:0]  ov2_f    = col_ok(ov2_sel,    ref_col, ref_v) ? ov2_sel    : 4'd0;

    wire [3:0]  pick_raw = (run_l_f  != 4'd0) ? run_l_f  :
                           (run_l2_f != 4'd0) ? run_l2_f :
                           (ov1_f != 4'd0) ? ov1_f :
                           (ov2_f != 4'd0) ? ov2_f : free_lab_q;
    wire [3:0]  L_sel    = (pick_raw == 4'd0) ? 4'd0 :
                           ((par[pick_raw] == 4'd0) ? pick_raw : par[pick_raw]);

    wire [3:0]  other_raw = (run_l2_f != 4'd0 && find_lab(run_l2_f) != L_sel) ? run_l2_f :
                            (ov2_f    != 4'd0 && find_lab(ov2_f)    != L_sel) ? ov2_f    : 4'd0;
    // 保守合并：一次边界混合游程可能同时"接近"两个不同颜色的标签。
    // 若直接 keep/drop，会把两个异色标签桥接成一个（实测红球吞掉肤色小手）。
    // 因此额外要求两个被合并标签的代表色彼此相近；否则只保留主标签，另一个独立成域。
    wire [3:0]  M_root    = find_lab(other_raw);
    wire        pair_ok   = (other_raw == 4'd0) || (M_root == 4'd0) ||
                            !l_col_v[L_sel] || !l_col_v[M_root] ||
                            (cdist5(l_col[L_sel], l_col[M_root]) < COLOR_TH);
    wire [3:0]  M_sel     = (other_raw == 4'd0 || !pair_ok) ? 4'd0 : M_root;

    wire [3:0]  keep = (M_sel != 4'd0 && M_sel < L_sel) ? M_sel : L_sel;
    wire [3:0]  drop = (M_sel != 4'd0 && M_sel < L_sel) ? L_sel :
                       (M_sel != 4'd0)                  ? M_sel : 4'd0;

    // ---------------- B 段 pending 寄存器 ----------------
    reg        P_valid;
    reg [3:0]  P_keep, P_drop;
    reg [8:0]  P_run_s, P_close_e;
    reg [9:0]  P_y;
    reg        P_last_c;
    reg [14:0] P_col;
    reg        P_col_v;

    wire [3:0]  P_drop_s = (P_drop == 4'd0) ? P_keep : P_drop;
    wire [8:0]  b_minx = (P_drop != 4'd0 && l_minx[P_drop_s] < l_minx[P_keep]) ? l_minx[P_drop_s] : l_minx[P_keep];
    wire [8:0]  b_maxx = (P_drop != 4'd0 && l_maxx[P_drop_s] > l_maxx[P_keep]) ? l_maxx[P_drop_s] : l_maxx[P_keep];
    wire [9:0]  b_miny = (P_drop != 4'd0 && l_miny[P_drop_s] < l_miny[P_keep]) ? l_miny[P_drop_s] : l_miny[P_keep];
    wire [9:0]  b_maxy = (P_drop != 4'd0 && l_maxy[P_drop_s] > l_maxy[P_keep]) ? l_maxy[P_drop_s] : l_maxy[P_keep];
    wire [19:0] b_cnt  = l_cnt[P_keep]   + ((P_drop != 4'd0) ? l_cnt[P_drop_s]   : 20'd0);
    wire [21:0] b_chrd = l_chord[P_keep] + ((P_drop != 4'd0) ? l_chord[P_drop_s] : 22'd0);
    wire [10:0] b_rows = l_rows[P_keep]  + ((P_drop != 4'd0) ? l_rows[P_drop_s]  : 11'd0);
    wire        b_has  = l_has[P_keep] | ((P_drop != 4'd0) ? l_has[P_drop_s] : 1'b0);

    wire [19:0] b_run_len = {11'd0, (P_close_e - P_run_s + 9'd1)};

    // B 段合并结果（与单拍版本逐条对应）
    wire [8:0]  b_rmnx0 = (P_run_s  < l_rmnx[P_keep]) ? P_run_s  : l_rmnx[P_keep];
    wire [8:0]  b_rmxx0 = (P_close_e > l_rmxx[P_keep]) ? P_close_e : l_rmxx[P_keep];

    wire [8:0]  b_rmnx  = b_has ? b_rmnx0 : P_run_s;
    wire [8:0]  b_rmxx  = b_has ? b_rmxx0 : P_close_e;
    wire        b_has1  = ~P_last_c;

    wire [21:0] b_chrd1 = b_has
                          ? (b_chrd + (P_last_c ? {13'd0, (b_rmxx0 - b_rmnx0 + 9'd1)} : 22'd0))
                          : (b_chrd + (P_last_c ? {13'd0, b_run_len[8:0]} : 22'd0));
    wire [10:0] b_rows1 = b_rows + (P_last_c ? 11'd1 : 11'd0);

    // 行末就在 B 段这一拍：由 B 段补做行结算（行循环已让路）
    wire        b_settle = last_c && !P_last_c && b_has1;
    wire [21:0] b_chrd2 = b_settle ? (b_chrd1 + {13'd0, (b_rmxx - b_rmnx + 9'd1)}) : b_chrd1;
    wire [10:0] b_rows2 = b_settle ? (b_rows1 + 11'd1) : b_rows1;
    wire        b_has2  = b_settle ? 1'b0 : b_has1;

    // ==================================================================
    // 主状态机
    // ==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (i = 1; i <= NB; i = i + 1) begin
                par[i]     <= 4'd0;
                l_minx[i]  <= 9'h1ff; l_maxx[i] <= 9'd0;
                l_miny[i]  <= 10'h3ff; l_maxy[i] <= 10'd0;
                l_cnt[i]   <= 20'd0;
                l_chord[i] <= 22'd0;
                l_rows[i]  <= 11'd0;
                l_rmnx[i]  <= 9'h1ff; l_rmxx[i] <= 9'd0;
                l_has[i]   <= 1'b0;
                l_col[i]   <= 15'd0;
                l_col_v[i] <= 1'b0;
            end
            par[0] <= 4'd0;
            pr_n <= 4'd0; cr_n <= 4'd0;
            in_run <= 1'b0; run_s <= 9'd0; run_l <= 4'd0; run_l2 <= 4'd0;
            run_col <= 15'd0; run_col_v <= 1'b0;
            P_valid <= 1'b0; P_keep <= 4'd1; P_drop <= 4'd0;
            S_rowpend <= 1'b0;
            P_run_s <= 9'd0; P_close_e <= 9'd0; P_y <= 10'd0; P_last_c <= 1'b0;
            P_col <= 15'd0; P_col_v <= 1'b0;
            S_rowpend <= 1'b0;
            O_ovf  <= 1'b0;
        end
        else if (I_clr) begin
            for (i = 1; i <= NB; i = i + 1) begin
                par[i]     <= 4'd0;
                l_minx[i]  <= 9'h1ff; l_maxx[i] <= 9'd0;
                l_miny[i]  <= 10'h3ff; l_maxy[i] <= 10'd0;
                l_cnt[i]   <= 20'd0;
                l_chord[i] <= 22'd0;
                l_rows[i]  <= 11'd0;
                l_rmnx[i]  <= 9'h1ff; l_rmxx[i] <= 9'd0;
                l_has[i]   <= 1'b0;
                l_col[i]   <= 15'd0;
                l_col_v[i] <= 1'b0;
            end
            par[0] <= 4'd0;
            pr_n <= 4'd0; cr_n <= 4'd0;
            in_run <= 1'b0; run_s <= 9'd0; run_l <= 4'd0; run_l2 <= 4'd0;
            run_col <= 15'd0; run_col_v <= 1'b0;
            P_valid <= 1'b0; P_col_v <= 1'b0;
            S_rowpend <= 1'b0;
            O_ovf  <= 1'b0;
        end
        else if (S_ev_fin) begin
            for (i = 1; i <= NB; i = i + 1) begin
                par[i]     <= 4'd0;
                l_minx[i]  <= 9'h1ff; l_maxx[i] <= 9'd0;
                l_miny[i]  <= 10'h3ff; l_maxy[i] <= 10'd0;
                l_cnt[i]   <= 20'd0;
                l_chord[i] <= 22'd0;
                l_rows[i]  <= 11'd0;
                l_rmnx[i]  <= 9'h1ff; l_rmxx[i] <= 9'd0;
                l_has[i]   <= 1'b0;
                l_col[i]   <= 15'd0;
                l_col_v[i] <= 1'b0;
            end
            par[0] <= 4'd0;
            pr_n <= 4'd0; cr_n <= 4'd0;
            in_run <= 1'b0; run_s <= 9'd0; run_l <= 4'd0; run_l2 <= 4'd0;
            run_col <= 15'd0; run_col_v <= 1'b0;
            P_valid <= 1'b0; P_col_v <= 1'b0;
            S_rowpend <= 1'b0;
            O_ovf <= 1'b0;
            for (i = 0; i < 8; i = i + 1) begin
                pr_s[i] <= 9'd0; pr_e[i] <= 9'd0; pr_l[i] <= 4'd0;
                cr_s[i] <= 9'd0; cr_e[i] <= 9'd0; cr_l[i] <= 4'd0;
            end
        end
        else begin
            // ---------------- B 段：写回上一拍的关闭 ----------------
            P_valid <= 1'b0;
            if (P_valid) begin
                l_minx[P_keep] <= (P_run_s  < b_minx) ? P_run_s  : b_minx;
                l_maxx[P_keep] <= (P_close_e > b_maxx) ? P_close_e : b_maxx;
                l_miny[P_keep] <= (P_y < b_miny) ? P_y : b_miny;
                l_maxy[P_keep] <= (P_y > b_maxy) ? P_y : b_maxy;
                l_cnt[P_keep]  <= b_cnt + b_run_len;
                l_chord[P_keep]<= b_chrd2;
                l_rows[P_keep] <= b_rows2;
                l_rmnx[P_keep] <= b_rmnx;
                l_rmxx[P_keep] <= b_rmxx;
                l_has[P_keep]  <= b_has2;
                // 代表色：新标签直接采用；已有标签只在"本次合并色与标签色足够近"
                // 时慢速跟踪（抗光照渐变）。若一次边界混合色误合并也去更新，
                // 标签色会被逐步拉向相邻物体（实测红球会因此吞掉小手），
                // 所以用 COLOR_TH/2 作为更新门，异常合并不污染代表色。
                if (P_col_v) begin
                    if (!l_col_v[P_keep]) begin
                        l_col[P_keep]   <= P_col;
                        l_col_v[P_keep] <= 1'b1;
                    end
                    else if (cdist5(P_col, l_col[P_keep]) < COLOR_LOCAL) begin
                        l_col[P_keep] <= { ema5(l_col[P_keep][14:10], P_col[14:10]),
                                           ema5(l_col[P_keep][9:5],   P_col[9:5]),
                                           ema5(l_col[P_keep][4:0],   P_col[4:0]) };
                    end
                end
            end

            // 行尾结算推迟：行尾这一拍同时有关闭时，下一拍再结算其余标签
            if (S_rowpend) begin
                for (i = 1; i <= NB; i = i + 1)
                    if (l_has[i] && !(i == P_keep) &&
                        !(P_drop != 4'd0 && i == P_drop)) begin
                        l_chord[i] <= l_chord[i] + {13'd0, (l_rmxx[i] - l_rmnx[i] + 9'd1)};
                        l_rows[i]  <= l_rows[i] + 11'd1;
                        l_has[i]   <= 1'b0;
                    end
                S_rowpend <= 1'b0;
            end

            if (I_valid) begin
                // ---- 1. 游程跟踪 ----
                if (cell_v) begin
                    if (!in_run) begin
                        in_run <= 1'b1;
                        run_s  <= cx;
                        run_l  <= ov1_cur;
                        run_l2 <= ov2_cur;
                        run_col   <= I_rgb;
                        // 高光起头的游程没有"物体色"可记：等第一个非高光整格再定色
                        run_col_v <= cell_full && !specular;
                    end
                    else begin
                        // 代表色：首个整格先取源色，之后按 1/4 EMA 跟随本游程。
                        // 只取"首个整格"会把靠近球/手边界的一个红肤混合色当成
                        // 手的代表色（与红球距离可能 < 阈值），使小手被并进红球；
                        // 长游程 EMA 会收敛到纯肤色。颜色跳变判据仍用更新前的
                        // run_col，因此不会削弱边界检测。
                        // 高光/增亮低饱和格不更新代表色（no_split），避免把球的
                        // 代表色拉向白色后拒绝另一侧同色游程。
                        if (cell_full && !no_split) begin
                            if (!run_col_v) begin
                                run_col   <= I_rgb;
                                run_col_v <= 1'b1;
                            end
                            else begin
                                run_col <= { ema5(run_col[14:10], I_rgb[14:10]),
                                             ema5(run_col[9:5],   I_rgb[9:5]),
                                             ema5(run_col[4:0],   I_rgb[4:0]) };
                            end
                        end
                        if ((ov1_cur != 4'd0) && (run_l == 4'd0))
                            run_l <= ov1_cur;
                        else if ((ov1_cur != 4'd0) && (find_lab(ov1_cur) != find_lab(run_l)) && (run_l2 == 4'd0))
                            run_l2 <= ov1_cur;
                        if ((ov2_cur != 4'd0) && (run_l2 == 4'd0) &&
                            (run_l == 4'd0 || find_lab(ov2_cur) != find_lab(run_l)))
                            run_l2 <= ov2_cur;
                    end
                end

                // ---- 2. 关闭：A 段（指针/行表立即更新，统计交 B 段）----
                if (closing && (keep == 4'd0)) O_ovf <= 1'b1;
                if (closing && (keep != 4'd0)) begin
                    par[keep] <= keep;
                    if (drop != 4'd0) begin
                        par[drop] <= keep;
                        for (i = 1; i <= NB; i = i + 1)
                            if ((par[i] == drop) && (i != drop) && (i != keep))
                                par[i] <= keep;
                    end

                    P_valid   <= 1'b1;
                    P_keep    <= keep;
                    P_drop    <= drop;
                    P_run_s   <= run_s_eff;
                    P_close_e <= close_e;
                    P_y       <= I_y[9:0];
                    P_last_c  <= last_c;
                    P_col     <= run_col;
                    P_col_v   <= run_col_v;

                    if (cr_n < 4'd8) begin
                        cr_s[cr_n] <= run_s_eff;
                        cr_e[cr_n] <= close_e;
                        cr_l[cr_n] <= keep;
                        cr_n       <= cr_n + 4'd1;
                    end
                end

                if (closing) begin
                    in_run    <= 1'b0;
                    run_l     <= 4'd0;
                    run_l2    <= 4'd0;
                    run_col_v <= 1'b0;
                end

                // ---- 3. 行尾：结算本行跨度，游程表换行 ----
                if (last_c) begin
                    if (closing) begin
                        // 本拍有游程关闭：其余标签的结算推迟到下一拍，
                        // 避免把 keep/drop 的组合解析塞进行尾使能路径。
                        S_rowpend <= 1'b1;
                    end
                    else begin
                        for (i = 1; i <= NB; i = i + 1) begin
                            if (l_has[i] &&
                                !(P_valid && (i == P_keep)) &&
                                !(P_valid && (P_drop != 4'd0) && (i == P_drop))) begin
                                l_chord[i] <= l_chord[i] + {13'd0, (l_rmxx[i] - l_rmnx[i] + 9'd1)};
                                l_rows[i]  <= l_rows[i] + 11'd1;
                                l_has[i]   <= 1'b0;
                            end
                        end
                    end
                    for (i = 0; i < 8; i = i + 1) begin
                        pr_s[i] <= cr_s[i];
                        pr_e[i] <= cr_e[i];
                        pr_l[i] <= cr_l[i];
                    end
                    if (closing) begin
                        if (cr_n < 4'd7) begin
                            pr_s[cr_n[2:0]] <= run_s_eff;
                            pr_e[cr_n[2:0]] <= close_e;
                            pr_l[cr_n[2:0]] <= keep;
                            pr_n <= cr_n + 4'd1;
                        end
                        else begin
                            pr_s[3'd7] <= run_s_eff;
                            pr_e[3'd7] <= close_e;
                            pr_l[3'd7] <= keep;
                            pr_n <= 4'd8;
                        end
                    end
                    else begin
                        pr_n <= cr_n;
                    end
                    cr_n <= 4'd0;
                end
            end
        end
    end

    // ==================================================================
    // [T3] 帧尾评估三级流水
    // ==================================================================
    reg [4:0] S_ev;
    wire S_frm_end = I_valid && last_c && last_r;

    wire [4:0] ev_idx = S_ev - 5'd2;          // 3..17 -> 1..15
    wire [3:0] e_i    = ev_idx[3:0];
    wire       e_sel  = (S_ev >= 5'd3) && (S_ev <= 5'd17);
    wire       e_live = e_sel && (e_i != 4'd0) && (par[e_i] == e_i) && (l_rows[e_i] != 11'd0)
                        && (l_maxx[e_i] >= l_minx[e_i]) && (l_maxy[e_i] >= l_miny[e_i]);
    wire [8:0] e_wc  = (l_maxx[e_i] >= l_minx[e_i]) ? (l_maxx[e_i] - l_minx[e_i] + 9'd1) : 9'd0;
    wire [9:0] e_h   = (l_maxy[e_i] >= l_miny[e_i]) ? (l_maxy[e_i] - l_miny[e_i] + 10'd1) : 10'd0;

    reg         M1_live;
    reg  [3:0]  M1_lab;
    reg  [8:0]  M1_wc;
    reg  [9:0]  M1_h;
    reg  [19:0] M1_cnt;
    reg  [21:0] M1_chord;
    reg  [10:0] M1_rows;
    reg  [8:0]  M1_minx, M1_maxx;
    reg  [9:0]  M1_miny, M1_maxy;

    reg         M2_live;
    reg  [3:0]  M2_lab;
    reg  [8:0]  M2_wc;
    reg  [9:0]  M2_h;
    reg  [19:0] M2_cnt;
    reg  [21:0] M2_chord;
    reg  [10:0] M2_rows;
    reg  [8:0]  M2_minx, M2_maxx;
    reg  [9:0]  M2_miny, M2_maxy;
    reg  [18:0] M2_area;

    wire [15:0] M2_wpx = {2'b0, M2_wc, 2'b00};
    wire [15:0] M2_hpx = {6'd0, M2_h};
    wire [24:0] e_c8  = {3'd0, M2_chord} << 3;
    wire [24:0] e_a5  = {6'd0, M2_area} * 12'd5;
    wire [24:0] e_a7  = {6'd0, M2_area} * 12'd7;
    wire        e_chord_ok = (e_c8 >= e_a5) && (e_c8 <= e_a7);
    wire        e_asp_ok   = (M2_wpx * 12'd5 >= M2_hpx * 12'd4) &&
                             (M2_hpx * 12'd5 >= M2_wpx * 12'd4);
    wire        e_rows_ok  = ({5'd0, M2_rows} * ROW_DEN) >= (M2_hpx * ROW_NUM);
    wire        e_size_ok  = (M2_wpx >= MIN_W_PX) && (M2_wpx <= MAX_W_PX) &&
                             (M2_hpx >= MIN_W_PX) && (M2_hpx <= MAX_W_PX) &&
                             (M2_cnt >= MIN_CELLS);
    wire        e_pass     = e_size_ok && e_asp_ok && e_rows_ok && e_chord_ok;
    wire        e_cand     = e_size_ok && e_asp_ok && e_rows_ok && !e_chord_ok;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_ev     <= 5'd0;
            S_ev_fin <= 1'b0;
            O_evt    <= 1'b0;
            O_w <= 12'd0; O_h <= 12'd0; O_cnt <= 24'd0; O_chord <= 21'd0; O_rows <= 11'd0;
            O_minx <= 12'd0; O_maxx <= 12'd0; O_miny <= 12'd0; O_maxy <= 12'd0;
            O_pass <= 1'b0; O_cand <= 1'b0;
            O_live <= 1'b0; S_live_any <= 1'b0;
            O_ok_size <= 1'b0; O_ok_asp <= 1'b0; O_ok_rows <= 1'b0; O_ok_chord <= 1'b0;
            O_lab <= 4'd0;
            M1_live <= 1'b0; M1_lab <= 4'd0; M1_wc <= 9'd0; M1_h <= 10'd0;
            M1_cnt <= 20'd0; M1_chord <= 22'd0; M1_rows <= 11'd0;
            M1_minx <= 9'd0; M1_maxx <= 9'd0; M1_miny <= 10'd0; M1_maxy <= 10'd0;
            M2_live <= 1'b0; M2_lab <= 4'd0; M2_wc <= 9'd0; M2_h <= 10'd0;
            M2_cnt <= 20'd0; M2_chord <= 22'd0; M2_rows <= 11'd0;
            M2_minx <= 9'd0; M2_maxx <= 9'd0; M2_miny <= 10'd0; M2_maxy <= 10'd0;
            M2_area <= 19'd0;
        end
        else begin
            O_evt    <= 1'b0;
            S_ev_fin <= 1'b0;

            if (S_ev == 5'd0) begin
                if (S_frm_end) begin
                    S_ev      <= 5'd2;      // 留 1 拍给 B 段写回
                    O_pass    <= 1'b0;
                    O_cand    <= 1'b0;
                    O_live    <= 1'b0;
                    S_live_any <= 1'b0;
                    O_ok_size <= 1'b0; O_ok_asp <= 1'b0;
                    O_ok_rows <= 1'b0; O_ok_chord <= 1'b0;
                    O_w <= 12'd0; O_h <= 12'd0; O_cnt <= 24'd0;
                    O_chord <= 21'd0; O_rows <= 11'd0;
                    O_minx <= 12'd0; O_maxx <= 12'd0; O_miny <= 12'd0; O_maxy <= 12'd0;
                    O_lab <= 4'd0;
                    M1_live <= 1'b0; M2_live <= 1'b0;
                end
            end
            else if (S_ev <= 5'd19) begin
                // 第 3 级：判据（S_ev=5..19）
                if (S_ev >= 5'd5) begin
                    if (M2_live && e_pass && !O_pass) begin
                        O_pass  <= 1'b1;
                        O_w     <= M2_wpx[11:0];
                        O_h     <= M2_hpx[11:0];
                        O_cnt   <= (M2_cnt << 2);
                        O_chord <= M2_chord[20:0];
                        O_rows  <= M2_rows;
                        O_minx  <= {1'b0, M2_minx, 2'b00};
                        O_maxx  <= {1'b0, M2_maxx, 2'b00} + 12'd3;
                        O_miny  <= {2'b0, M2_miny};
                        O_maxy  <= {2'b0, M2_maxy};
                        O_lab   <= M2_lab;
                        O_ok_size <= 1'b1; O_ok_asp <= 1'b1;
                        O_ok_rows <= 1'b1; O_ok_chord <= 1'b1;
                    end
                    else if (M2_live && e_cand && !O_pass && !O_cand) begin
                        O_cand  <= 1'b1;
                        O_w     <= M2_wpx[11:0];
                        O_h     <= M2_hpx[11:0];
                        O_cnt   <= (M2_cnt << 2);
                        O_chord <= M2_chord[20:0];
                        O_rows  <= M2_rows;
                        O_minx  <= {1'b0, M2_minx, 2'b00};
                        O_maxx  <= {1'b0, M2_maxx, 2'b00} + 12'd3;
                        O_miny  <= {2'b0, M2_miny};
                        O_maxy  <= {2'b0, M2_maxy};
                        O_lab   <= M2_lab;
                        O_ok_size <= 1'b1; O_ok_asp <= 1'b1;
                        O_ok_rows <= 1'b1; O_ok_chord <= 1'b0;
                    end
                end

                // 第 2 级：面积（S_ev=4..18）
                if (S_ev >= 5'd4 && S_ev <= 5'd18) begin
                    M2_live  <= M1_live;
                    M2_lab   <= M1_lab;
                    M2_wc    <= M1_wc;
                    M2_h     <= M1_h;
                    M2_cnt   <= M1_cnt;
                    M2_chord <= M1_chord;
                    M2_rows  <= M1_rows;
                    M2_minx  <= M1_minx;
                    M2_maxx  <= M1_maxx;
                    M2_miny  <= M1_miny;
                    M2_maxy  <= M1_maxy;
                    M2_area  <= M1_wc * M1_h;
                end

                // 第 1 级：锁存指标（S_ev=3..17）
                if (e_sel) begin
                    if (e_live) S_live_any <= 1'b1;
                    M1_live  <= e_live;
                    M1_lab   <= e_i;
                    M1_wc    <= e_wc;
                    M1_h     <= e_h;
                    M1_cnt   <= l_cnt[e_i];
                    M1_chord <= l_chord[e_i];
                    M1_rows  <= l_rows[e_i];
                    M1_minx  <= l_minx[e_i];
                    M1_maxx  <= l_maxx[e_i];
                    M1_miny  <= l_miny[e_i];
                    M1_maxy  <= l_maxy[e_i];
                end

                S_ev <= S_ev + 5'd1;
            end
            else begin
                O_evt    <= 1'b1;
                O_live   <= S_live_any;
                S_ev_fin <= 1'b1;
                S_ev     <= 5'd0;
            end
        end
    end

    // 帧尾评估期间不允许新像素进入（标签数组正被读取）
    assign O_busy = (S_ev != 5'd0);

endmodule
