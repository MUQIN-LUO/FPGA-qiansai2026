// golden reference for tb_ccl_regression:
//   HEAD 2762152 的 isp_ccl_circle，仅加 T4 溢出哨兵修复（par[0]/O_ovf 复位、pick==0 保护）。
// ==========================================================================
// 连通域 + 圆形判据（替代"整帧单一累加器"）
// --------------------------------------------------------------------------
// 【为什么要重写】
//   旧实现 isp_circle_detect 内部是一个 isp_span_acc：整帧统一取 min/max 当
//   外接框，每行"弦长"= 全行最左命中点到最右命中点。只要有背景边缘，
//   外接框就等于整帧、弦长横跨全行，rho = span/(w*h) -> 1.0，圆形判据
//   必然失败；两个相距很远的圆也会被合成一个大框。
//   仿真实测（改造前）：纹理背景 rho=0.995；加入真实圆后 span 一字不变
//   （911848），圆被背景完全吞掉；两个圆合成 644x162 一个框。
//
// 【本模块】
//   1. 降采样成"格"：每拍 4 像素取或 = 1 格，320 格/行，正好 1 格/周期。
//   2. 逐行提取游程（run），与上一行游程按 x±1 重叠合并；15 个标签的
//      并查集，merge 时 eager flatten，父指针深度恒为 1。
//   3. 每个标签独立累计：外接框、格数、**标签内部**逐行跨度（弦长）、
//      有效行数。弦长绝不跨到别的标签或背景。
//   4. 帧尾对每个根标签分别判 size / aspect / rows / chord，取面积最大的
//      通过者输出；同时给出该候选的完整指标供诊断。
//
// 【单位】
//   w_px = 4*w_cells（1 格 = 4 像素），h 用行数，chord/area 都在格单位，
//   比值无量纲。
//
// 【已知边界（写清楚，不藏着）】
//   某个游程恰好在"行尾那一拍"关闭时，它这一行的跨度会记到下一行结算，
//   弦长总和仍然正确，只是归属行偏移 1 行。影响 < 1 个 run 长度。
// ==========================================================================
module isp_ccl_circle_ref #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [11:0]  MIN_W_PX   = 12'd32,
    parameter [11:0]  MAX_W_PX   = 12'd720,
    parameter [23:0]  MIN_CELLS  = 24'd8,
    parameter [11:0]  ROW_NUM    = 12'd9,
    parameter [11:0]  ROW_DEN    = 12'd10
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_clr,
    input  wire        I_valid,
    input  wire [3:0]  I_hit,
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
    output reg         O_ok_size,
    output reg         O_ok_asp,
    output reg         O_ok_rows,
    output reg         O_ok_chord,
    output reg  [3:0]  O_lab,
    output reg         O_ovf                    // 有 run 因标签耗尽被丢弃（明确降级标志）
);

    localparam integer NB = 15;

    // ---------------- 标签状态 ----------------
    reg [3:0]  par    [0:NB];   // 下标 0 保留为"空闲"哨兵，避免越界读    // 0 = 空闲；i = 自己就是根；其它 = 已合并到该根
    reg [8:0]  l_minx [1:NB], l_maxx [1:NB];
    reg [9:0]  l_miny [1:NB], l_maxy [1:NB];
    reg [19:0] l_cnt  [1:NB];
    reg [21:0] l_chord[1:NB];
    reg [10:0] l_rows [1:NB];
    reg [8:0]  l_rmnx [1:NB], l_rmxx [1:NB];   // 本行内的跨度
    reg        l_has  [1:NB];

    // ---------------- 上一行 / 本行 游程表 ----------------
    reg [8:0]  pr_s [0:7], pr_e [0:7];
    reg [3:0]  pr_l [0:7];
    reg [3:0]  pr_n;
    reg [8:0]  cr_s [0:7], cr_e [0:7];
    reg [3:0]  cr_l [0:7];
    reg [3:0]  cr_n;

    reg        in_run;
    reg [8:0]  run_s;
    reg [3:0]  run_l, run_l2;

    // 评估结束脉冲：只由评估 FSM 驱动，主状态机只读它来清空标签
    reg        S_ev_fin;

    integer i, k;

    // find：父指针深度恒为 1
    function [3:0] find_lab;
        input [3:0] l;
        begin
            find_lab = (l == 4'd0) ? 4'd0 : ((par[l] == 4'd0) ? l : par[l]);
        end
    endfunction

    // 空闲标签（最低号优先）
    reg [3:0] free_lab;
    always @(*) begin
        free_lab = 4'd0;
        for (i = NB; i >= 1; i = i - 1)
            if (par[i] == 4'd0) free_lab = i[3:0];
    end

    // ---------------- 当前拍 ----------------
    // ★ 根因修复：cx 必须覆盖 0..319（1280/4 = 320 格/行）。
    //   原来写成 I_x[8:2] 只有 7 位有效值（0..127），越过 128 格（512 像素）
    //   就回绕，导致 minx>maxx、两个目标被并到一起、标签迅速耗尽。
    wire [8:0] cx     = I_x[10:2];
    wire       cell_v   = |I_hit;
    wire       last_c = (I_x >= (IMG_WIDTH  - 4));
    wire       last_r = (I_y >= (IMG_HEIGHT - 1));

    // 与上一行游程的重叠（x±1）
    reg [3:0] ov1, ov2;
    always @(*) begin
        ov1 = 4'd0;
        ov2 = 4'd0;
        for (k = 0; k < 8; k = k + 1) begin
            if ((k < pr_n) && ((cx + 9'd1) >= pr_s[k]) && (cx <= (pr_e[k] + 9'd1))) begin
                if (find_lab(pr_l[k]) != 4'd0) begin
                    if (ov1 == 4'd0)
                        ov1 = find_lab(pr_l[k]);
                    else if (find_lab(pr_l[k]) != ov1)
                        ov2 = find_lab(pr_l[k]);
                end
            end
        end
    end

    // ---------------- 关闭游程：目标标签与合并对象 ----------------
    wire        closing = in_run && (!cell_v || last_c);
    wire [8:0]  close_e = cell_v ? cx : (cx - 9'd1);

    // 选根：run_l 优先（游程期间累计），其次 run_l2，再次本拍重叠，最后新分配
    wire [3:0]  pick_raw = (run_l  != 4'd0) ? run_l  :
                           (run_l2 != 4'd0) ? run_l2 :
                           (ov1    != 4'd0) ? ov1    :
                           (ov2    != 4'd0) ? ov2    : free_lab;
    wire [3:0]  L_sel    = (pick_raw == 4'd0) ? 4'd0 :
                           ((par[pick_raw] == 4'd0) ? pick_raw : par[pick_raw]);

    // 需要合并的另一个根
    wire [3:0]  other_raw = (run_l2 != 4'd0 && find_lab(run_l2) != L_sel) ? run_l2 :
                            (ov2    != 4'd0 && find_lab(ov2)    != L_sel) ? ov2    : 4'd0;
    wire [3:0]  M_sel     = (other_raw == 4'd0) ? 4'd0 :
                            ((par[other_raw] == 4'd0) ? other_raw : par[other_raw]);

    wire [3:0]  keep = (M_sel != 4'd0 && M_sel < L_sel) ? M_sel : L_sel;
    wire [3:0]  drop = (M_sel != 4'd0 && M_sel < L_sel) ? L_sel :
                       (M_sel != 4'd0)                  ? M_sel : 4'd0;

    // 合并后的统计（keep 槽的新值）
    wire [8:0]  n_minx = (drop != 4'd0 && l_minx[drop] < l_minx[keep]) ? l_minx[drop] : l_minx[keep];
    wire [8:0]  n_maxx = (drop != 4'd0 && l_maxx[drop] > l_maxx[keep]) ? l_maxx[drop] : l_maxx[keep];
    wire [9:0]  n_miny = (drop != 4'd0 && l_miny[drop] < l_miny[keep]) ? l_miny[drop] : l_miny[keep];
    wire [9:0]  n_maxy = (drop != 4'd0 && l_maxy[drop] > l_maxy[keep]) ? l_maxy[drop] : l_maxy[keep];
    wire [19:0] n_cnt  = l_cnt[keep] + ((drop != 4'd0) ? l_cnt[drop] : 20'd0);
    wire [21:0] n_chrd = l_chord[keep] + ((drop != 4'd0) ? l_chord[drop] : 22'd0);
    wire [10:0] n_rows = l_rows[keep] + ((drop != 4'd0) ? l_rows[drop] : 11'd0);
    wire        n_has  = l_has[keep] | ((drop != 4'd0) ? l_has[drop] : 1'b0);

    wire [19:0] run_len = {11'd0, (close_e - run_s + 9'd1)};

    // ==================================================================
    // 逐周期：游程跟踪 -> 关闭时写标签 -> 行尾结算
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
            end
            par[0] <= 4'd0;
            pr_n <= 4'd0; cr_n <= 4'd0;
            in_run <= 1'b0; run_s <= 9'd0; run_l <= 4'd0; run_l2 <= 4'd0;
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
            end
            par[0] <= 4'd0;
            pr_n <= 4'd0; cr_n <= 4'd0;
            in_run <= 1'b0; run_s <= 9'd0; run_l <= 4'd0; run_l2 <= 4'd0;
            O_ovf  <= 1'b0;
        end
        else if (S_ev_fin) begin
            // 上一帧评估完成：清空全部标签与游程表，开始新一帧
            for (i = 1; i <= NB; i = i + 1) begin
                par[i]     <= 4'd0;
                l_minx[i]  <= 9'h1ff; l_maxx[i] <= 9'd0;
                l_miny[i]  <= 10'h3ff; l_maxy[i] <= 10'd0;
                l_cnt[i]   <= 20'd0;
                l_chord[i] <= 22'd0;
                l_rows[i]  <= 11'd0;
                l_rmnx[i]  <= 9'h1ff; l_rmxx[i] <= 9'd0;
                l_has[i]   <= 1'b0;
            end
            pr_n <= 4'd0; cr_n <= 4'd0;
            in_run <= 1'b0; run_s <= 9'd0; run_l <= 4'd0; run_l2 <= 4'd0;
            // 游程表必须一起清：否则新帧第 0 行会拿上一帧的旧标签号去匹配，
            // 把无关的 run 合并到一起（实测会造出 minx>maxx 的垃圾槽）
            O_ovf <= 1'b0;   // 每帧起始清零，帧内累积，帧尾报告
            for (i = 0; i < 8; i = i + 1) begin
                pr_s[i] <= 9'd0; pr_e[i] <= 9'd0; pr_l[i] <= 4'd0;
                cr_s[i] <= 9'd0; cr_e[i] <= 9'd0; cr_l[i] <= 4'd0;
            end
        end
        else if (I_valid) begin
            // ---- 1. 游程跟踪 ----
            if (cell_v) begin
                if (!in_run) begin
                    in_run <= 1'b1;
                    run_s  <= cx;
                    run_l  <= ov1;
                    run_l2 <= ov2;
                end
                else begin
                    if ((ov1 != 4'd0) && (run_l == 4'd0))
                        run_l <= ov1;
                    else if ((ov1 != 4'd0) && (find_lab(ov1) != find_lab(run_l)) && (run_l2 == 4'd0))
                        run_l2 <= ov1;
                    if ((ov2 != 4'd0) && (run_l2 == 4'd0) &&
                        (run_l == 4'd0 || find_lab(ov2) != find_lab(run_l)))
                        run_l2 <= ov2;
                end
            end

            // ---- 2. 关闭：写标签统计（keep 槽一次性给出最终值）----
            // 标签耗尽：keep==0 表示没有空闲标签，该 run 只能丢弃 —— 置溢出标志，绝不静默
            if (closing && (keep == 4'd0)) O_ovf <= 1'b1;
            if (closing && (keep != 4'd0)) begin
                // 根指针：keep 指自己；drop 指 keep；其余指向 drop 的一并改指
                par[keep] <= keep;
                if (drop != 4'd0) begin
                    par[drop] <= keep;
                    for (i = 1; i <= NB; i = i + 1)
                        if ((par[i] == drop) && (i != drop) && (i != keep))
                            par[i] <= keep;
                end

                // 合并两槽 + 本次 run 的统计
                l_minx[keep] <= (run_s < n_minx) ? run_s : n_minx;
                l_maxx[keep] <= (close_e > n_maxx) ? close_e : n_maxx;
                l_miny[keep] <= ({2'b0, I_y[9:0]} < n_miny) ? I_y[9:0] : n_miny;
                l_maxy[keep] <= ({2'b0, I_y[9:0]} > n_maxy) ? I_y[9:0] : n_maxy;
                l_cnt[keep]  <= n_cnt + run_len;
                l_chord[keep]<= n_chrd;
                l_rows[keep] <= n_rows;
                // ★ 若本 run 恰好在"行尾那一拍"关闭，必须在这里就把本行跨度结算掉。
                //   否则行尾结算会跳过它（`!(closing && i==keep)`），该行弦长永远
                //   加不上 —— 实测对象贴在右边缘时 lastx=1275、下一拍就是最后一拍，
                //   弦长恒为 0，圆形判据必然失败。
                if (!n_has) begin
                    l_rmnx[keep] <= run_s;
                    l_rmxx[keep] <= close_e;
                    l_has[keep]  <= last_c ? 1'b0 : 1'b1;
                    if (last_c) begin
                        l_chord[keep] <= n_chrd + {13'd0, (close_e - run_s + 9'd1)};
                        l_rows[keep]  <= n_rows + 11'd1;
                    end
                end
                else begin
                    l_rmnx[keep] <= (run_s  < l_rmnx[keep]) ? run_s  : l_rmnx[keep];
                    l_rmxx[keep] <= (close_e > l_rmxx[keep]) ? close_e : l_rmxx[keep];
                    l_has[keep]  <= last_c ? 1'b0 : 1'b1;
                    if (last_c) begin
                        l_chord[keep] <= n_chrd + {13'd0,
                            (((close_e > l_rmxx[keep]) ? close_e : l_rmxx[keep]) -
                             ((run_s  < l_rmnx[keep]) ? run_s  : l_rmnx[keep]) + 9'd1)};
                        l_rows[keep]  <= n_rows + 11'd1;
                    end
                end

                // 记录本行游程（供下一行做重叠匹配）
                if (cr_n < 4'd8) begin
                    cr_s[cr_n] <= run_s;
                    cr_e[cr_n] <= close_e;
                    cr_l[cr_n] <= keep;
                    cr_n       <= cr_n + 4'd1;
                end
            end

            if (closing) begin
                in_run <= 1'b0;
                run_l  <= 4'd0;
                run_l2 <= 4'd0;
            end

            // ---- 3. 行尾：结算各标签本行跨度，游程表换行 ----
            if (last_c) begin
                for (i = 1; i <= NB; i = i + 1) begin
                    if (l_has[i] && !(closing && (i == keep))) begin
                        l_chord[i] <= l_chord[i] + {13'd0, (l_rmxx[i] - l_rmnx[i] + 9'd1)};
                        l_rows[i]  <= l_rows[i] + 11'd1;
                        l_has[i]   <= 1'b0;
                    end
                end
                for (i = 0; i < 8; i = i + 1) begin
                    pr_s[i] <= cr_s[i];
                    pr_e[i] <= cr_e[i];
                    pr_l[i] <= cr_l[i];
                end
                // 本行最后一个 run 若正好在行尾关闭，也要进下一行的匹配表
                // ★ 本 run 若在行尾这一拍关闭，它还没写进 cr_*（非阻塞），
                //   上面的拷贝拿到的是旧 cr_n 内容 —— 必须在这里补写进 pr_*，
                //   否则下一行匹配不到它，每行都新分配标签，15 行后标签耗尽。
                if (closing) begin
                    if (cr_n < 4'd7) begin
                        pr_s[cr_n[2:0]] <= run_s;
                        pr_e[cr_n[2:0]] <= close_e;
                        pr_l[cr_n[2:0]] <= keep;
                        pr_n <= cr_n + 4'd1;
                    end
                    else begin
                        pr_s[3'd7] <= run_s;
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

    // ==================================================================
    // 帧尾评估：15 个标签逐个判定，取面积最大者
    // ==================================================================
    reg [4:0] S_ev;
    wire S_frm_end = I_valid && last_c && last_r;

    wire [3:0]  e_i    = S_ev[3:0];
    wire        e_live = (e_i != 4'd0) && (par[e_i] == e_i) && (l_rows[e_i] != 11'd0)
                        && (l_maxx[e_i] >= l_minx[e_i]) && (l_maxy[e_i] >= l_miny[e_i]);

    wire [8:0]  e_wc  = (l_maxx[e_i] >= l_minx[e_i]) ? (l_maxx[e_i] - l_minx[e_i] + 9'd1) : 9'd0;
    wire [9:0]  e_h   = (l_maxy[e_i] >= l_miny[e_i]) ? (l_maxy[e_i] - l_miny[e_i] + 10'd1) : 10'd0;
    wire [18:0] e_area = e_wc * e_h;                       // 格单位面积
    wire [15:0] e_wpx = {2'b0, e_wc, 2'b00};               // 4 * w_cells
    wire [15:0] e_hpx = {6'd0, e_h};

    wire [24:0] e_c8  = {3'd0, l_chord[e_i]} << 3;         // 8 * chord
    wire [24:0] e_a5  = {6'd0, e_area} * 12'd5;
    wire [24:0] e_a7  = {6'd0, e_area} * 12'd7;

    wire e_chord_ok = (e_c8 >= e_a5) && (e_c8 <= e_a7);
    wire e_asp_ok   = (e_wpx * 12'd5 >= e_hpx * 12'd4) &&
                      (e_hpx * 12'd5 >= e_wpx * 12'd4);
    wire e_rows_ok  = ({5'd0, l_rows[e_i]} * ROW_DEN) >= (e_hpx * ROW_NUM);
    wire e_size_ok  = (e_wpx >= MIN_W_PX) && (e_wpx <= MAX_W_PX) &&
                      (e_hpx >= MIN_W_PX) && (e_hpx <= MAX_W_PX) &&
                      (l_cnt[e_i] >= MIN_CELLS);
    wire e_pass     = e_size_ok && e_asp_ok && e_rows_ok && e_chord_ok;
    wire e_cand     = e_size_ok && e_asp_ok && e_rows_ok && !e_chord_ok;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_ev     <= 5'd0;
            S_ev_fin <= 1'b0;
            O_evt    <= 1'b0;
            O_w <= 12'd0; O_h <= 12'd0; O_cnt <= 24'd0; O_chord <= 21'd0; O_rows <= 11'd0;
            O_minx <= 12'd0; O_maxx <= 12'd0; O_miny <= 12'd0; O_maxy <= 12'd0;
            O_pass <= 1'b0; O_cand <= 1'b0;
            O_ok_size <= 1'b0; O_ok_asp <= 1'b0; O_ok_rows <= 1'b0; O_ok_chord <= 1'b0;
            O_lab <= 4'd0;
            // O_ovf 只由标注级（下面的 always）驱动：这里不再写，避免多驱动。
            // 语义：标注级每帧起始清零、帧内累积、帧尾保持，正好供帧尾读取。
        end
        else begin
            O_evt    <= 1'b0;
            S_ev_fin <= 1'b0;

            if (S_ev == 5'd0) begin
                if (S_frm_end) begin
                    S_ev      <= 5'd1;
                    O_pass    <= 1'b0;
                    O_cand    <= 1'b0;
                    O_ok_size <= 1'b0; O_ok_asp <= 1'b0;
                    O_ok_rows <= 1'b0; O_ok_chord <= 1'b0;
                    O_w <= 12'd0; O_h <= 12'd0; O_cnt <= 24'd0;
                    O_chord <= 21'd0; O_rows <= 11'd0;
                    O_minx <= 12'd0; O_maxx <= 12'd0; O_miny <= 12'd0; O_maxy <= 12'd0;
                    O_lab <= 4'd0;
                    // 不在这里清 O_ovf：帧尾正是要读它的时候
                end
            end
            else if (S_ev <= 5'd15) begin
                if (e_live && e_pass && !O_pass) begin
                    O_pass  <= 1'b1;
                    O_w     <= e_wpx[11:0];
                    O_h     <= e_hpx[11:0];
                    O_cnt   <= (l_cnt[e_i] << 2);
                    O_chord <= l_chord[e_i][20:0];
                    O_rows  <= l_rows[e_i];
                    O_minx  <= {1'b0, l_minx[e_i], 2'b00};
                    O_maxx  <= {1'b0, l_maxx[e_i], 2'b00} + 12'd3;
                    O_miny  <= {2'b0, l_miny[e_i]};
                    O_maxy  <= {2'b0, l_maxy[e_i]};
                    O_lab   <= e_i;
                    O_ok_size <= 1'b1; O_ok_asp <= 1'b1;
                    O_ok_rows <= 1'b1; O_ok_chord <= 1'b1;
                end
                else if (e_live && e_cand && !O_pass && !O_cand) begin
                    O_cand  <= 1'b1;
                    O_w     <= e_wpx[11:0];
                    O_h     <= e_hpx[11:0];
                    O_cnt   <= (l_cnt[e_i] << 2);
                    O_chord <= l_chord[e_i][20:0];
                    O_rows  <= l_rows[e_i];
                    O_minx  <= {1'b0, l_minx[e_i], 2'b00};
                    O_maxx  <= {1'b0, l_maxx[e_i], 2'b00} + 12'd3;
                    O_miny  <= {2'b0, l_miny[e_i]};
                    O_maxy  <= {2'b0, l_maxy[e_i]};
                    O_lab   <= e_i;
                    O_ok_size <= 1'b1; O_ok_asp <= 1'b1;
                    O_ok_rows <= 1'b1; O_ok_chord <= 1'b0;
                end
                S_ev <= S_ev + 5'd1;
            end
            else begin
                O_evt    <= 1'b1;
                S_ev_fin <= 1'b1;
                S_ev     <= 5'd0;
            end
        end
    end

endmodule
