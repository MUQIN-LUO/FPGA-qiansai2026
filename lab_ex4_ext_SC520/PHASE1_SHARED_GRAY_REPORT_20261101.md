# SC520 第一阶段：共享灰度框架 —— 实验性交付报告（2026-11-01）

> ## ⚠️ 实验性 / 仅供明早用户上板验证
>
> - 本报告对应的 RTL 与 bit 是**实验性第一阶段结果**，**只供用户明早自行上板验证**。
> - **未烧录、未上板、未做摄像头实拍**；本文中的“通过”仅指 RTL 仿真或 TD 构建，
>   不等于真实光学/传感器/显示器环境已通过。
> - 未验证项、已知边界和“不要声称”的内容见 §8。**不要**把本报告当作量产或
>   完整圆检测验收依据。
> - 第二阶段（真正接入圆检测）尚未开始；本阶段只预留了灰度流接口。

## 0. 基线与工作区

- 基线提交：`074ae1b`（`origin/main` 与本地 `main` 在开始时一致；
  `git log -1` 为 “Improve circle diagnostics and result stability”）。
- 独立 worktree：`D:\FPGA\wt_phase1_gray`，分支 `feat/phase1-shared-gray`，
  基于 `074ae1b` 创建。原仓库 `D:\FPGA\project` 的未提交 Hough 实验、其他工程
  改动、未跟踪 bit/日志/产物**全部保留**，未 reset/stash/清理。
- 原仓库未跟踪的 `sim/tb/tb_isp_top_display.v`、`ph1p_sim_models.v`、
  `ph1p_phy_gsr_stub.v`、`run_isp_top_display.bat` 已**只读复制**到 worktree
  供诊断；原件保留。
- 构建在独立副本：
  `D:\FPGA\phase1_build_20261101\lab_ex4_ext_SC520`（不写原工程、不覆盖历史 bit）。

## 1. 本阶段做了什么

### 1.1 单一 RGB→Y 源

新增 `user_source/hdl_source/isp/isp_gray_share96.v`：

- `isp_rgb2y96`：纯组合，4 像素/拍，公式与基线**逐位相同**
  `Y = (R>>2) + (G>>1) + (B>>2)`（0..253，8bit 不溢出）。
- `isp_gray_tap96`：只打 **1 拍**寄存器，输出共享 Y、valid/tlast、
  显示 tuser、检测 tuser、以及该拍首像素坐标 `(x,y)`。
  - 显示 tuser 保持 AWB 原来的“早于数据”约定，只供显示通路；
  - 检测 tuser 使用与 valid 对齐的 `O_tuser_alg`，只供预留检测流。
  - 坐标：`x=0,4,...,1276`，`y=0..719`，tuser 标记 `(0,0)`；
    只在 valid 拍推进。

### 1.2 复用点

| 消费者 | 改动 | 是否保持像素级行为 |
|---|---|---|
| GRAY 显示 | 共享 Y 再打 1 拍（共 2 拍），组合复制 `{Y,Y,Y}` | 与基线 `gray_d1/gray_d2` 同拍同值 |
| 显示 EDGE | 新 `edge_detect_y96` 直接吃共享 Y；算法不变 | 水平 2 像素平均、垂直 1 行、2 拍流水全部保留；`tb_gray_share` 与基线逐像素一致 |
| POP | `isp_pop96` 新增 `I_y4/USE_SHARED_Y`，亮度档用共享 Y，色相仍用 RGB | 与内部 Y 模式逐像素一致 |
| 前景 Sobel | `u_fg_edge` 改吃共享 Y | 输入/控制对齐，`tb_det_frontend_awb` 通过（长 `tb_frontend` 见 §3.4） |
| 预留检测流 | `isp_top` 新增 `O_dg_*` 端口 | 无消费者；不影响现有显示/检测 |

`edge_detect_96.v` 现在是**RGB 兼容包装**，内部实例化同一个
`edge_detect_y96.v`；旧测试仍实例化 `edge_detect_96`，因此测试与综合使用同一核心。

### 1.3 预留检测灰度流接口

`isp_top` 新增端口（详见 `sim/README.md`）：

**权威位序（后续圆检测必须按此实现，不能按 `Y0..Y3` 名字猜像素顺序）**：

| `O_dg_y4` 位域 | 像素 | 对应 AWB 输入 `I_rgb` 位域 | 说明 |
|---|---|---|---|
| `[31:24]` | **x+0** | `[95:72]` | 该拍最左/最高像素 |
| `[23:16]` | **x+1** | `[71:48]` | |
| `[15:8]`  | **x+2** | `[47:24]` | |
| `[7:0]`   | **x+3** | `[23:0]`  | 该拍最右/最低像素 |

```
O_dg_y4[31:24]  // Y(x+0) <- I_rgb[95:72]
O_dg_y4[23:16]  // Y(x+1) <- I_rgb[71:48]
O_dg_y4[15:8]   // Y(x+2) <- I_rgb[47:24]
O_dg_y4[7:0]    // Y(x+3) <- I_rgb[23:0]
O_dg_valid
O_dg_tuser      // 与 valid 对齐的检测 tuser，不是显示 tuser
O_dg_tlast
O_dg_x[11:0]    // 该拍 x+0 的列坐标，0,4,...,1276
O_dg_y[11:0]    // 行号 0..719
```

**命名陷阱（源码已锁定，本轮只改文档，不改 RTL/哈希）**：
`isp_gray_share96.v` 内部的 `O_y4 = {Y3,Y2,Y1,Y0}` 是 **MSB-first 打包标签**，
不是像素序号。`Y3` 位于 `[31:24]`，对应 **x+0**；`Y0` 位于 `[7:0]`，
对应 **x+3**。**不要把 `Y0` 当成 x+0**，也不要按 `Y0..Y3` 的顺序遍历像素。
后续圆检测器请直接按上表的位索引 / `x+0..x+3` 消费。

- 4 像素/拍、8bit/像素；
- 相对 AWB 输出 `S_rgb_data` 固定 **1 个 CSI 时钟**；
- **不接 `I_mode`**：显示模式选择不进入检测灰度流；
- 本阶段没有接检测器，综合可能裁剪；接上消费者后即活跃。
- 这是**全分辨率灰度流**，但**没有**做全分辨率梯度/Sobel；
  EDGE 仍是原来的降采样 Sobel（2 样本/拍、垂直 1 行）。

## 2. 明确保留的行为

- RAW / EDGE / GRAY / POP 四种显示模式与按键切换逻辑不变；
- 显示 EDGE 的像素级算法不变（不是“改成全分辨率 Sobel”）；
- POP 调色板/色相/套印不变；
- AWB 动态（灰度世界）与 K1 白纸标定/锁定参数、AE 参数/控制、曝光 HUD 未改；
- 现有颜色无关圆检测（`isp_ccl_bridge` / `isp_ccl_circle` / `isp_shape_circle`）
  接口与算法未改；
- `O_frm_tgl` 仍由**未被检测屏蔽**的真实帧尾 `S_frm_evt_det` 驱动，
  每帧翻转，不依赖检测事件；未接检测器时也持续更新；
- DDR / 显示帧率 / SDC 未放宽。

## 3. 仿真证据

所有 TB 直接实例化工程 RTL；日志在
`D:\FPGA\wt_phase1_gray\lab_ex4_ext_SC520\sim\logs\`。

### 3.1 快速回归（2026-11-01）

| 测试 | 结果 | 说明 |
|---|---|---|
| `tb_gray_share` | PASS，363 checks，0 errors，`EDGE_WHITE_BEATS=12` | RGB→Y 公式、tap 1 拍/u/t/l/坐标、EDGE 新旧逐像素（并确认确实产生非黑边缘）、POP 共享/内部逐像素、GRAY 2 拍路径逐像素、显示 tuser 在 valid=0 拍上单独验证（2 次） |
| `tb_pop` | PASS 0 errors | POP 4 档/调色板/色相/套印回归 |
| `tb_awb_apply` | PASS 0 errors | 灰度世界动态 + K1 标定模式 |
| `tb_cal_mode_cdc` | PASS 0 errors | K1 电平跨域 |
| `tb_ccl_regression` | PASS 0 errors | CCL vs 参考模型 |
| `tb_bridge` | PASS 0 errors | 掩膜 FIFO/CDC/丢拍恢复 |
| `tb_ccl_color` | PASS 0 errors | CCL 颜色一致性 |
| `tb_ae_ctrl` | PASS 0 errors | AE 闭环/限幅 |
| `tb_ae_i2c` | PASS 0 errors | 真实 I2C 165 写 + AE 8 笔写/曝光/增益/完成时序 |
| `tb_det_result` | PASS 0 errors | 圆检测结果锁存/看门狗 |
| `tb_det_frontend_awb` | PASS 0 errors | 前端 AWB/掩膜/CCL/FIFO 恢复，含共享 Y 前景 Sobel |

### 3.2 完整显示通路 A/B（同一 TB，基线 074ae1b vs 新 RTL）

TB：`sim/tb/tb_display_regression.v`；真实 1280×720 行/场时序；
每 720 个 tlast 为一组，打印 `VALID/TLAST/TUSER/HASH`。

| 运行 | 模式 | 结果 |
|---|---|---|
| 2 帧基线 vs 新版 | POP → RAW | 两组 `VALID=230400/TLAST=720/TUSER=1`、HASH 完全一致；`O_frm_tgl` 翻转 2 次；总 valid/tlast 一致；POP 行首诊断 BAD=133（基线相同，见 §8） |
| 2 帧基线 vs 新版 | GRAY → EDGE | GRAY 组 HASH `97174c4c1f72f2b6`、EDGE 组 HASH `0000000000000000`；基线/新版两组 HASH、`VALID=230400/TLAST=720/TUSER=1`、总 valid/tlast、`O_frm_tgl=2` 完全一致；POP_BAD=0 |
| 5 帧基线（部分） | POP/RAW 已完成 | 与 2 帧结果一致；后续模式因控制仿真时长已停止，不冒充完成 |

> **EDGE 组的 HASH 为零**：在完整行为级 TB 的输入图案下，基线 074ae1b
> 和新 RTL 的 EDGE 输出**都**是 0，因此该组只证明“新旧逐位一致”，
> **不能**用它单独证明 EDGE 有可见白线。EDGE 的独立功能性证据是
> `tb_gray_share`：同一小图案下新旧 EDGE 逐像素/握手完全一致，且
> `EDGE_WHITE_BEATS=12`（确实产生非黑边缘）。完整 TB 的 EDGE 全黑属于
> 该行为级测试/输入图案的已知诊断现象，未在本阶段归因或修复。

> 说明：显示通路的 AWB `O_tuser` 按原厂约定**早于数据**，因此显示 tuser
> 落在 valid=0 拍是设计约定，不是错误；本 TB 只统计，不判错。
> 预留检测流 `O_dg_tuser` 另行验证“必须落在 valid 拍”。

### 3.3 预留检测灰度流集成

TB：`sim/tb/tb_dg_stream.v`，完整 `isp_top`；新增 `+LINES=n` 可按行限长，
默认 720 行为完整帧。

检查项：
- `O_dg_valid`/`O_dg_tlast`/`O_dg_tuser` 帧结构（完整帧 230400/720/1）；
- `O_dg_tuser` 必须在 valid 拍；
- `O_dg_x=0,4,...,1276`，`O_dg_y=0..719`，tuser 重启坐标；
- `O_dg_y4` 与 AWB 输出 `S_rgb_data` 的 `R/4+G/2+B/4` 逐拍一致（延迟 1 拍）；
  位序严格按 §1.3：`[31:24]=x+0`、`[23:16]=x+1`、`[15:8]=x+2`、`[7:0]=x+3`；
- 两种显示模式下结构相同（流本身无 `I_mode` 输入）。

**已完整退出并打印 RESULT 的两次运行**：

| 运行 | 命令 | 结果 | 日志 |
|---|---|---|---|
| 完整 1 帧（GRAY） | `+FRAMES=1` | `DG_FRAME 0 VALID=230400 TLAST=720 TUSER=1`；`CHECKS=230400`；**PASS 0 errors** | `logs/tb_dg_stream_f1_full.log` |
| 有界 3 行 × 2 帧（GRAY→EDGE） | `+FRAMES=2 +LINES=3` | 每帧 `VALID=960 TLAST=3 TUSER=1`；`OFFVALID_TUSER=0`；`CHECKS=1920`；**PASS 0 errors**，约 3–6 s | `logs/tb_dg_stream_lines3_2mode.log` |

说明：`logs/tb_dg_stream.log` 与 `logs/tb_dg_stream_f1.log` 是此前为让出 CPU
**主动 TaskStop** 的两次运行，日志停在 `run -all`，没有 RESULT，**不计为通过**；
上面的两次运行才是本阶段的接口证据。完整 1 帧只覆盖 GRAY 模式；
GRAY→EDGE 两模式由 3 行有界运行覆盖，且该流本身没有 `I_mode` 输入。

### 3.4 长回归

- `tb_frontend`（27 场景，真实行突发/消隐，约 15 分钟）**本轮未重跑**，
  以控制仿真时长；其编译依赖已更新为 `edge_detect_y96.v` + `edge_detect_96.v`。
  作为替代，短版检测前端 `tb_det_frontend_awb` 已 PASS（覆盖 AWB/掩膜/CCL/
  FIFO 恢复和共享 Y 前景 Sobel）。因此不把 27 场景长回归计为已通过；
  需要全场景结论时应在空闲机器上另行运行。

## 4. 构建（TD 6.2.1，PH1P35MDG324 / -3）

构建在独立副本，未放宽 SDC、未改时钟约束。

### 4.1 综合（gate）

| 项目 | 新 RTL | 基线 074ae1b |
|---|---|---|
| Setup WNS / TNS | **-0.060 ns / -2.151 ns**（44 违例端点） | +0.680 ns / 0 |
| Hold WNS / TNS | +0.114 ns / 0 | +0.114 ns / 0 |
| LUT（总） / LUT6 | 25,093 / 11,617 | 25,136 / 11,755 |
| 寄存器 | 14,458 | 14,458 |
| slice | 19,651（92.62%） | 19,759（93.13%） |
| ERAM / DSP | 40 / 31 | 40 / 31 |
| STA 覆盖率 | 94.83% | 见基线报告 |

**关键路径核对（回应“AWB 乘法到灰度链”提示）**：
综合 gate 的最差路径**不是**共享灰度，而是既有 CCL 域：
`u_ccl_bridge/u_mask_fifo` RAM 读 → `u_circle_detect/par_reg[0].d`，
`ccl_clk` 40 ns 预算、42 级、数据路径 39.996 ns，Slack -0.060 ns。
该路径的 RTL（`isp_ccl_circle.v` / `isp_ccl_bridge.v`）本阶段未改；
基线同一模块的 gate 余量为 +0.680 ns，本轮是综合映射 QoR 波动。
共享灰度所在的 MIPI 域 gate WNS 为 **+0.748 ns**，不是最差路径。
P&R placement 初始 SWNS +1365 ps，最终完整 P&R 为
**+0.112 ns / +0.020 ns（setup/hold）**，见 §4.2。

### 4.2 布局布线 / bitgen（完整 route + fix_hold + bitgen）

| 项目 | 新 RTL（完整 P&R） | 基线 074ae1b |
|---|---|---|
| Setup WNS / TNS | **+0.112 ns / 0**，0 违例端点 | +0.091 ns / 0 |
| Hold WNS / TNS | **+0.020 ns / 0**，0 违例端点 | +0.020 ns / 0 |
| LUT6 | 18,002 / 21,216（84.85%） | 18,079（85.21%） |
| 寄存器 | 14,358 | 14,314 |
| slice | 18,399（86.72%） | 18,390（86.68%） |
| ERAM / DSP | 40 / 31 | 40 / 31 |
| STA 覆盖率 | **96.24%** | 96.26% |
| 时钟（WNS/HWNS） | MIPI 0.112/0.020；ccl_clk 0.173/0.051；HDMI 0.369/0.026 | MIPI 0.197/0.020；ccl_clk 0.091/0.036；HDMI 0.439/0.022 |

**完整 P&R 最差路径（回应 AWB→灰度链提示）**：

- 最差 setup 路径：`u_isp_top/u_edge_detect_y96/w_t1_reg[5]` →
  `u_isp_top/u_edge_detect_y96/O_tdata_reg[0]`，MIPI_RX_BYTE_CLK
  9.660 ns，Slack **+0.112 ns**。
  这是共享 Y 的 **EDGE 核心内部**（窗口寄存器 → Sobel/比较 → 输出寄存器），
  **不是** AWB 乘法器 → Y 加法器 → tap 的那条路径。
- 第 2 条：`u_edge_detect_y96/O_tdata_reg` → `gam_data_reg`，+0.161 ns
  （EDGE 输出经显示 mux/gamma）。
- 第 3 条：AWB 乘法器 → `u_pop96/h_q_reg`，+0.165 ns（既有 POP 路径）。
- CCL 路径：ccl_clk WNS **+0.173 ns**；综合 gate 的 -0.060 ns CCL
  违例在 P&R 后已恢复，不是最终违例。
- 基线最差时钟是 `ccl_clk`（+0.091 ns）；本轮 `ccl_clk` 改善到
  **+0.173 ns**，最差转为 MIPI 域的 EDGE 核心 **+0.112 ns**。
- 结论：共享 Y 的插入没有让 AWB→灰度链成为最差路径；最差路径是
  EDGE 核心原有的“窗口→Sobel→输出”链，余量 +112 ps，无违例。
  原来的 `ed_tdata_d`（RGB 输入打拍）已被共享 tap 的 Y 寄存器替代；
  “AWB 乘法器 → Y 加法器 → tap”没有进入最差路径前三。
  余量仍然很小，不能据此声称温度/电压/长期稳定。

**bit 与日志**：

- 独立 bit（已复制，未烧录）：
  `D:\FPGA\phase1_build_20261101\deliverable\camera_to_dsi_display_phase1_shared_gray.bit`
  - 大小：1,789,539 bytes
  - SHA256：`f4b69f5a26c8905f4613aac6c5d6066670c889cf62cc6ecd63b7a99692bddbda`
- 原始 phy 位流：`D:\FPGA\phase1_build_20261101\lab_ex4_ext_SC520\td_project\camera_to_dsi_display_Runs\phy_1\camera_to_dsi_display.bit`
  （同一 SHA256）。
- 构建日志：
  - `D:\FPGA\phase1_build_20261101\syn_flow.log`
  - `D:\FPGA\phase1_build_20261101\phy_flow.log`
  - `...\camera_to_dsi_display_Runs\syn_1\run.log`
  - `...\camera_to_dsi_display_Runs\phy_1\td_20261101_012832.log`
  - `...\phy_1\route.qor`、`final_timing.rpt`、`check_timing.rpt`、
    `route_clock_utilization.txt`
- 耗时（TD 流程统计）：syn 约 8m14s；phy 约 9m42s（place 3m24s /
  route 5m57s / fix_hold 10s / bitgen 8s）。

**未约束 / 例外核对**（`check_timing.rpt`，与基线同类）：
1 个无根时钟（`u_uicfgcs520/u_i2c/scl_clk`，I2C SCL 分频）；
37 个输入端口无 input delay、68 个输出端口无 output delay、1 个输入
partial input delay；2 条被忽略的时序例外；0 条 invalid constraint、
0 条 combinational loop、0 条 no-path constraint。SDC 未新增/放宽。

## 5. 板上验证步骤（用户明早自行执行；本报告未执行）

### 5.1 文件

- 新 bit（实验性，未烧录）：
  `D:\FPGA\phase1_build_20261101\deliverable\camera_to_dsi_display_phase1_shared_gray.bit`
  - SHA256 `f4b69f5a26c8905f4613aac6c5d6066670c889cf62cc6ecd63b7a99692bddbda`
  - 1,789,539 bytes
- 构建日志：`D:\FPGA\phase1_build_20261101\syn_flow.log`、
  `D:\FPGA\phase1_build_20261101\phy_flow.log`
- 时序/资源报告：`D:\FPGA\phase1_build_20261101\deliverable\final_timing.rpt`、
  `...\camera_to_dsi_display_phy.area`、`...\route.qor`
- 本报告：`D:\FPGA\wt_phase1_gray\lab_ex4_ext_SC520\PHASE1_SHARED_GRAY_REPORT_20261101.md`

### 5.2 步骤

1. 断电，连接 SC520 板、MIPI 摄像头、HDMI 显示器；确认摄像头/显示器供电。
2. 用 TD 6.2.1 的下载/编程工具选择上面的 bit，器件
   `PH1P35MDG324` / speed `-3` / Flash 模型；**只下载到 SRAM 做本次验证**，
   除非明确要覆盖 Flash。
3. 上电/复位后观察（映射已按实际 RTL/引脚核对：
   `design_top_wrapper.v` 的 `S_btn_press[0..3]` 与
   `constraints_source/pin.adc` 的 `I_button[0..3]`）：
   - 默认 RAW 彩色画面；
   - `I_button[0]` / 板载 K1（D5）短按 → RAW；长按 ≥1.5 s 翻转
     K1 白纸标定/锁定模式（`S_cal_tgl`，50 MHz 域计数 75,000,000 拍）；
   - `I_button[1]` / K2（A9）→ EDGE（Sobel 二值化）；
   - `I_button[2]` / K3（B9）→ GRAY；
   - `I_button[3]` / K4（C7）→ **循环下一个模式**：
     RAW → EDGE → GRAY → POP → RAW（不是“直接跳到 POP”）。
   - 按键内部上拉、按下为低；20 ms 消抖后取下降沿。
4. 核对 OSD：AE 曝光/增益读数、圆形诊断 HUD（既有 CCL 结果）仍更新；
   画面无花屏、无行场错位、无帧率异常。
5. 本阶段没有接新检测器，因此**不应期待新的圆检测行为**；新灰度流只存在于
   RTL 接口，未参与画框。若需验证预留接口，需要第二阶段接检测器。
6. 记录现象（尤其 EDGE 细线、POP 行首、AWB 收敛、K1 切换）；若异常，
   保存串口/OSD/拍屏并回报，不要自行改 SDC 或放宽约束。

## 6. 源 / 工程 / SDC / bit 哈希

已核对（构建副本 `D:\FPGA\phase1_build_20261101\lab_ex4_ext_SC520`）：

- `.al` 实际 `<File Path>` 条目：**70**（基线 074ae1b 为 68，
  本阶段新增 2 条）。全部为相对路径；`../user_source` 68 条
  （59 条 Verilog/SystemVerilog + 9 条 adc/xml/ipc），另 2 条为 SDC 与
  ChipWatcher xml。worktree 与构建副本逐条比较：文件列表相同，0 缺失，
  CRLF 归一化后内容差异 0。
- 构建源清单（70 条，CRLF 归一化，按路径排序）整体 SHA256：
  `37300e7a76842dfa558fab47470f21ce61e7042ffc2114e85473bef1188409ac`；
  明细见 `D:\FPGA\phase1_build_20261101\build_manifest_final.txt`。
- `camera_to_dsi_display.al` SHA256（构建副本打开前）：
  `ff13b9551d5bca1bf74cbb306954b650a262438f8e21e2575dc7287af8a18b92`；
  TD 打开后可能只改写 `Path`/运行元数据，源条目不变。
- `camera_to_dsi_display.sdc` SHA256：
  `e8c6e4d59821d775f6919dab03ae19f9e7431a850f13a2a0fd9cacb0882c09b7`
  （与基线逐字节相同；未放宽、未修改）。
- 关键 RTL SHA256（构建副本原始字节）：
  - `isp_top.v` `d4a142ec3c298d00b4b4357a5a71b0d5588f54f7353af53c4ff0b945dac8e67b`
  - `isp_gray_share96.v` `535775d4647947016962ae16e4764b4b9e13a665ca96aa129aa209029671c2b2`
  - `edge_detect_y96.v` `81d88a374a85ddace80dc81177e8e45995eeea19cbb0d7337e205bee4854930e`
  - `edge_detect_96.v` `993f010e38fefc945e99c0983ee04a68f5564535adda69847d9d244ea92bcc9d`
  - `isp_pop96.v` `8b29f08a10c401b7aa4b4512f92351e00e764b40f9c1a0498cf91cf1804cb5e5`
- bit：`D:\FPGA\phase1_build_20261101\deliverable\camera_to_dsi_display_phase1_shared_gray.bit`
  - 1,789,539 bytes
  - SHA256 `f4b69f5a26c8905f4613aac6c5d6066670c889cf62cc6ecd63b7a99692bddbda`
  - 未烧录；原始 phy 位流同 SHA。
- 本轮仅修正文档中的 `O_dg_y4` 位序说明，**未改 RTL**；上表关键
  RTL 哈希与 bit 仍对应同一构建源码，无需重建。

## 7. 变更文件清单

- 修改：`isp_top.v`、`edge_detect_96.v`、`isp_pop96.v`、`camera_to_dsi_display.al`、
  `sim/run.bat`、`sim/tb/tb_pop.v`、`sim/README.md`、
  `README_新设备构建与迁移.md`；
- 新增：`isp_gray_share96.v`、`edge_detect_y96.v`、`sim/tb/tb_gray_share.v`、
  `sim/tb/tb_display_regression.v`、`sim/tb/tb_dg_stream.v`；
- 从原工作区只读复制：`sim/tb/tb_isp_top_display.v`、`ph1p_sim_models.v`、
  `ph1p_phy_gsr_stub.v`、`sim/run_isp_top_display.bat`。

## 8. 局限 / 不要声称

1. **未烧录、未上板、未实拍**；仿真为合成/行为级场景，不含摄像头光学、
   噪声、运动模糊、Bayer 误差、HDMI 压缩。
2. 显示 EDGE 仍是原算法（水平 2 像素平均、垂直 1 行）；预留灰度流是
   4 像素/拍全分辨率，但**没有**全分辨率梯度/Sobel，不能声称“梯度同步”。
3. 预留 `O_dg_*` 本阶段无消费者，综合可能裁剪；它不是已接入的检测器。
4. 完整显示 TB 的 POP 行首诊断在基线 074ae1b 上同样为 BAD=133；
   本阶段未修复、也不把它归因于共享灰度。该 TB 只用于 A/B 对比。
5. 完整显示 TB 的 EDGE 组 HASH 为零（基线/新版都为零）；它只证明 A/B 一致，
   **不能**单独证明 EDGE 有可见白线。EDGE 功能性证据是
   `tb_gray_share` 的逐像素对比 + `EDGE_WHITE_BEATS=12`。
6. 显示 tuser 早于 valid 是原厂/基线约定；不要用“tuser 必须在 valid 拍”
   去判显示通路错误。检测 tuser 才是 valid 对齐的。
7. 时序余量很小：最终 setup **+0.112 ns**、hold **+0.020 ns**；
   最差路径是 `u_edge_detect_y96` 内部（窗口→Sobel→输出），不是
   AWB→Y→tap。未做温度/电压/长期稳定性验证，不能据此声称稳健。
8. 预留 `O_dg_*` 的完整 1 帧运行只覆盖 GRAY；GRAY→EDGE 由 3 行有界运行
   覆盖。流本身没有 `I_mode` 输入，但仍未接入真实检测器，不能把
   “接口结构通过”说成“圆检测已实现/已通过”。
9. 未修改 AE/AWB/K1 参数、检测算法、DDR/显示时钟或 SDC；若上板观感有变化，
   先核对是否与基线 bit 的差异，而不是先改约束。
10. 原仓库未提交的 Hough 实验与其他工程改动未被纳入本阶段，也未被清理。
11. `tb_frontend` 27 场景长回归未在本轮重跑；只用短版
   `tb_det_frontend_awb` 作为检测前端证据，不能声称全场景已回归。

## 9. 结论

第一阶段完成了“单一 RGB→Y 源 + 共享 tap + 预留 4px/beat 检测灰度接口”的
RTL 整理：

- 快速回归（POP/AWB/K1/CCL/AE+I2C/检测结果/短前端/共享灰度）全部 PASS；
- 完整显示通路 A/B：POP/RAW 与 GRAY/EDGE 两组逐组 HASH、valid/tlast/tuser
  与基线 074ae1b 完全一致；`O_frm_tgl` 每帧翻转；
- 预留检测灰度流：完整 1 帧（GRAY）230400/720/1 PASS；GRAY→EDGE
  两模式 3 行有界运行 PASS；
- 完整 TD syn + P&R + bitgen：setup **+0.112 ns**、hold **+0.020 ns**，
  0 违例；资源与基线基本持平；bit SHA256
  `f4b69f5a26c8905f4613aac6c5d6066670c889cf62cc6ecd63b7a99692bddbda`，
  **未烧录**。

**如实保留的边界**：EDGE 在完整行为级 TB 中整帧为黑（基线同样），
EDGE 功能由模块级逐像素对比证明；`tb_frontend` 27 场景未重跑；
未上板、未实拍。**最终结论以明早上板验证为准；在 Codex 验收前不提交、
不推送。**
