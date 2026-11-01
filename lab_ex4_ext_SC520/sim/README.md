# lab_ex4_ext_SC520 回归测试

所有 TB 都**直接实例化工程实际综合使用的 RTL**，不是测试台内手写镜像。

## 运行

ModelSim（`vlib`/`vlog`/`vsim` 在 PATH 中）：

```bat
cd lab_ex4_ext_SC520\sim
run.bat
```

日志写到 `sim/logs/`。`tb_frontend` 在 1280×720、真实行时序下约 15 分钟，
其余测试均为秒级。

## 测试清单

| 测试 | 实例化的 RTL | 覆盖 |
|---|---|---|
| `tb/tb_pop.v` | `isp_pop96.v`（HUE_AWARE=0/1 两个实例） | 亮度 4 档、4 组调色板、套印 x-2；**色相区分矩阵**：4 调色板 × 4 亮度档 × 暖/绿/冷，检查色相方向与曼哈顿色差门限；中性色必须与 HUE_AWARE=0 完全一致 |
| `tb/tb_awb_apply.v` | `isp_gray_world` + `isp_awb_arbiter` + `awb` + `signal_delay` | 复位 INIT、蓝/红收敛、稳定、过暗/过曝保持、MODE0 忽略 CAL、MODE1 原子生效、切回自动 |
| `tb/tb_cal_mode_cdc.v` | `design_top_wrapper.v` 内的 `cal_mode_cdc` | K1 电平跨域：上升/下降沿同拍 trig、start/abort、单周期脉冲、稳定无伪脉冲、CSI 复位 |
| `tb/tb_ccl_regression.v` | `isp_ccl_circle.v` vs `ref/isp_ccl_circle_ref.v` | 干净圆/遮挡/杂波/方块/卷尺/重叠/空心/溢出/跨行/密集关闭/连续帧/帧中清空，逐字段对比 |
| `tb/tb_bridge.v` | `isp_ccl_bridge.v` + `isp_mask_fifo.v` + `isp_ccl_circle.v` | 25 bit 掩膜+颜色 FIFO、坐标重建、事件 CDC、丢拍恢复、满恰逢 tuser、恢复帧 |
| `tb/tb_ccl_color.v` | `isp_ccl_circle.v` | 颜色一致性：异色接触分离、**同色接触仍合并**（证明不是几何硬切）、红/肤 50/50 混合格边界；颜色输入与 `isp_fg_mask.O_rgb` 一样取 4 像素平均 |
| `tb/tb_frontend.v` | `edge_detect_96` + `isp_bg_est` + `isp_fg_mask` + `isp_ccl_bridge` | 真实前端链路：绿/红/蓝球、手持球、手指压球、蓝布/双色底、无球、方块、卷尺、过小球、**d=40/60/100 px 小球+手遮挡+复杂背景**、**FIFO 溢出注入与恢复** |
| `tb/tb_isp_top_display.v` | 完整 `isp_top`（RAW→demosaic→AWB→显示选择）+ 行为级 PH1P 原语模型 | 显示通路诊断：真实行/帧时序、POP/RAW/GRAY/EDGE 切换、valid/tuser/tlast、POP 行首套印。**当前在行为级模型下未通过，不作为验收证据**；用 `run_isp_top_display.bat` 单独运行，需上板确认 |
| `tb/tb_det_frontend_awb.v` | 真实 `awb` + `edge_detect_96` + `isp_fg_mask` + `isp_bg_est` + `isp_ccl_bridge` | 提前帧首（AWB `O_tuser` off-valid）、检测专用 `O_tuser_alg` 与**首个有效像素**同拍（帧首特征像素 0x808080 断言）、mask/RGB 行首/行尾逐拍对应、圆 bbox 坐标 (540,260)-(743,460)、FIFO 溢出后恢复。**帧首 fg RGB 错位已定位为 `isp_fg_mask` 的 S2 被 `I_edge_valid` 门控导致首拍漏采，改为每拍无条件寄存后 PASS** |
| `tb/tb_ae_ctrl.v` | `ae_ctrl`（行为 cfg_done 握手） | 上电初值、暗场景到 AG_MAX、亮场景先降增益再缩曝光、死区、holdoff；**不证明真实 I2C 写入** |
| `tb/tb_ae_i2c.v` | `uicfgcs520` + `uii2c` + 受控 SDA 从机模型 | 真实 I2C 事务：初始化表 165 写 + 传感器 ID 0x9e4b；AE 请求后核对 8 笔写：0x3812=00（hold 开始）、0x3e00/01/02 曝光 1991、0x3e09/06/07 增益 96、0x3812=30（生效），以及 `O_ae_cfg_done` 完成时序；无 I2C 错误 |
| `tb/tb_det_result.v` | `isp_top.v` 内的真实 `isp_det_result` | 通过→候选→空帧/切换目标：绿色 valid 期间 bbox 必须保持"最后一次通过"的位置，候选不得覆盖；候选只有橙色；清空/复位归零 |

## 前端时序模型（重要）

`tb_frontend.v` 不是“均匀插空”：

- 行：320 拍有效像素**连续突发**（每 MIPI 字节时钟 1 拍），
  随后 1979 拍行消隐；`LINE_CYCLES=2300`（22.22 µs @103.52 MHz，
  对应 1280×720@60 的真实行周期）。
- 帧：720 行 + 30 行场消隐 ≈ 16.67 ms。
- 平均有效拍率 ≈ 14.4 M beat/s（真实约 13.8 M/s），
  但突发瞬间是 103.5 M beat/s；512 深 FIFO 必须吸收一行 320 拍的突发。
- 最后额外注入一帧“无消隐连续流”（行内 320 拍后不插空），必然触发
  FIFO 满/丢拍；随后正常帧必须恢复检测，验证坏帧/重同步路径。
- 每个场景先跑 `warm` 帧让背景估计收敛，再评估 1 帧；
  `warm` 按背景是否变化配置（同底色为 0，换底色为 2）。

## 顶层 CDC 接线检查（TD，不是 ModelSim）

`tb_cal_mode_cdc.v` 验证同步器行为；**顶层实际接线**用 TD elaboration
网表检查（在任意 TD run 目录下执行，该目录需有 `camera_to_dsi_display.prj`）：

```bat
"D:\Program Files\TD\bin\td_commands_prompt.exe" D:/FPGA/project/lab_ex4_ext_SC520/sim/verify_top_cdc.tcl
```

期望输出：

```
CAL_MODE_PIN=u_isp_top.I_cal_mode
CAL_MODE_NET=S_cal_t2
CAL_TRIG_PIN=u_isp_top.I_cal_trig
CAL_TRIG_NET=S_cal_trig
CAL_MODE_CDC=PASS
CAL_TRIG_NET_CHECK=PASS
CAL_TGL_LOADS=S_cal_tgl_i.i0 S_cal_tgl_i1.i0 S_cal_tgl_i5.I S_cal_tgl_reg.q u_cal_mode_cdc.I_cal_tgl
```

即 `I_cal_mode` 只能由 CSI 域同步后的 `S_cal_t2` 驱动，`S_cal_tgl`
的负载只在 50 MHz 域和同步器输入。

## 说明与边界

- 显示约定：AWB 的 `O_tuser` 仍按原厂直通（早于数据 ~52 拍），仅显示通路使用；
  **检测链路改用 `O_tuser_alg=I_tuser_r2`**（与 `O_tvalid/O_tdata` 同拍）。
  `tb_det_frontend_awb` 用帧首特征像素证明 `O_tuser_alg` 与首个有效像素同拍，
  圆 bbox 为 (540,260)-(743,460)；行首/行尾 mask/RGB 逐拍对应通过。
- `isp_fg_mask` 两处对齐修正：
  1) 颜色差分支 S1 比 Sobel/edge 晚一拍 → 增加 S2 对齐；
  2) S2 原先被 `I_edge_valid` 门控，首有效拍（tuser 当拍）的 S1 被漏采，
     导致帧首 O_rgb 仍是背景 → 改为每拍无条件寄存 S1，输出仍由 O_valid 门控。
  修正后 `tb_det_frontend_awb` 的帧首 marker、行首/行尾 RGB、bbox、圆检测、
  FIFO 溢出恢复全部 PASS；`tb_frontend` 27 场景在新 RTL 上重新回归 PASS。
- AE 亮度：参数改为目标 80（原 64）、增益上限 96/6x（原 80/4x）、大步长 8（原 16）；
  曝光优先、死区 6、holdoff 4 帧不变。`tb_ae_ctrl` 验证闭环/限幅；
  `tb_ae_i2c` 用受控 I2C 从机验证真实 uicfgcs520 的 8 笔写（group hold 0x3812=00/30、
  曝光 1991、增益 96）与完成时序。**OSD 第 6 行显示的是“AE 请求并已成功下发”
  的曝光/增益值，不是从传感器回读的实际值**（I2C 未回读）；值经 24MHz→CSI→HDMI
  两级异步域中继，避免 24MHz 与 HDMI 同源 PLL 的 CDC 时序违例，未修改 SDC。
  用户确认曝光已足够；本轮不再修改 AE/曝光参数、控制策略或曝光 HUD。
- **圆形诊断 HUD（OSD 第 7 行）**：`D<hex2>C<cnt>`
  - **EVT_CIRCLE=0（第一阶段 CCL）**：`flags[7:0] = {ovf, live, chord, rows,
    asp, size, pass, valid}`：[7] 标签溢出、[6] 本帧至少有一个有效连通域
    (live)、[5] 弦长通过、[4] 行连续通过、[3] 宽高比通过、[2] 尺寸通过、
    [1] 本帧通过、[0] 迟滞绿色 valid。
    **判据位 [5:2] 只在 pass=1 或候选（size/asp/rows=1 且 chord=0）时有效**。
    当 pass=0 且不是候选时，这些位为 0 只表示"本次没有选中的通过/候选目标"，
    不能据此声称具体是哪一条判据失败。`cnt` = 命中像素数/16（0..999），
    与 bbox/flags 同一次评估、同一拍锁存（`isp_det_result` 内）。
  - **EVT_CIRCLE=1（第二阶段事件检测器）**：位布局由
    `isp_evt_hud_map.v` 显式映射，**跳帧与实际溢出不再合并**：
    - [7] `ovf`：**本处理帧**捕获事件数超过 `EVMAX=1536`（实际溢出；
      溢出帧跳过处理，走 `S_DONE` 短路；按“无检测”老化保持）；
    - [6] `live`：本处理帧事件数 ≠ 0（与 cnt 同一快照）；
    - [5] `drop`：自上一次处理结果以来**至少有一帧被跳过**（该帧在
      `frame_start` 时内核正忙，整帧未捕获；跳过帧本身不产生结果，所以
      该位挂在下一处理结果上）；
    - [4:2] 保留 0（CCL 的 chord/rows/asp/size 位置）；[1] pass、[0] valid
      语义不变。
    因此上板可区分：绿球帧 `cnt` 很低且 [7]=0/[5]=0 → 事件数/对比度限制；
    [7]=1 → 纹理/噪声事件溢出；[5]=1 → 时序掉帧（处理超过一帧）。
  - **EVT 的 `cnt` 契约（10 位）**：`O_det_cnt = min(处理帧事件数, 999)`，
    饱和不截断。HUD 是三位十进制且比较器只覆盖 0..999，所以显示
    `999` 表示“≥999”（满载 1536 显示 999，不会再回绕成 512）。
    检测器内部 `O_count_dbg` 是 12 位精确计数（0..EVMAX），仅在
    `S_DONE` 与 `flags/valid/bbox` 同一拍锁存，保证描述的是同一处理帧
    （旧实现按 `frame_end` 更新，可能比结果早一帧）。
  - 坏帧/丢拍导致整帧没有评估事件时，`isp_det_result` 在帧起始按 miss 衰减
    置信度；连续丢失约 3 帧后旧绿框自动熄灭，不会无限保留。
- **识别框旁 CIRCLE 文字（本版新增）**：框上方（上方空间不足则下方）
  水平居中显示 `CIRCLE X+123 Y-045`。
  - 坐标 = 圆心相对画面中心 `(IMG_WIDTH/2, IMG_HEIGHT/2) = (640,360)`，
    **向右 X 正、向下 Y 正**；始终带符号、三位补零（零显示 `+000`），
    不显示 px。
  - 字高 32 显示像素（8x8 点阵放大 4 倍；10x10 字体单元含 1 像素描边）；
    绿色字形 `0x00FF00` + 黑色 8 邻域描边，背景透明。
  - 位置自动夹在有效区内（x 0..556，y 0..680；右侧留 3 像素流水线余量），
    不裁切；文字优先级高于 Logo/左上 HUD。
  - 文字、框、圆心坐标来自 `S_hdmi_user` 同一快照组（`O_det_cx/cy` 已加入
    CDC payload），保证同一显示帧；`I_det_valid=0` 时文字立即消失。
  - **方向核实**：RTL 显示通路为 1280x720 1:1，无缩放/镜像；中心为
    (640,360)。相机若物理倒装/镜像，需在安装层修正，RTL 不做翻转。
- **检测半径范围（本版更新）**：只有事件半径 `rho >= 33`（即严格 `>32px`）
  且 `rho <= 160` 才进入最终候选；`r=32` 拒绝、`r=33` 接受。注意最终
  3-bin 半径峰上报的是**下沿** `peak_bin+B0`，所以真实 33 在事件级可能
  显示 `r=32`（量化偏小 1px）；像素级 e2e 的 r33 实测显示 `r=33`。
  这是当前算法量化的实际边界，详见 `PHASE2_TESTS.md` rev 2j。
  **范围说明**：本版识别核心只做了这个“半径 26..32 小目标过滤”；
  低对比绿球漏检未修复，半径 >32 的圆形背景亮点仍会被接受，
  不宣称识别稳定性已解决。
- **检测结果状态修复**：旧实现 `O_det_valid`（跨帧迟滞）与 `S_box`（每帧
  `M_size_ok` 更新）共用一组 bbox，候选会把绿色框拉到候选位置。现抽出真实
  `isp_det_result`：绿色 valid 只用最后一次 `pass` 的 bbox；候选 bbox 仅在
  非 valid 时输出（橙色）；flags/cnt/bbox 同拍锁存。`tb_det_result` 覆盖
  pass→candidate→empty/切换目标、flags/cnt 原子性、坏帧丢事件看门狗。
- **红球短前端场景（`tb_frontend` 场景 27..33，可用 `+SCENE_ONLY=n` 单跑）**：
  - 27 原始硬边高光（白核 r=20 + 一步 50% 环）：**已知边界**。实测球被切碎，
    72x65 的高光块反而通过并占据绿框；保留为失败样例，不计入回归错误。
  - 28 平滑高光 + 手指压球并与手相连：球保持 196x190，手球分离；
  - 29 红球 + 背景里另一个真圆（蓝盘 d=120）：**形状无法判断谁是目标**；
    实测选中背景圆。该场景只验证"确实报出一个真圆"，不声称红球必胜；
  - 30 20 个背景块无球：不误报；
  - 31 球前 16 个背景块（标签耗尽）：**已知边界**，ball 被丢弃，
    `O_ovf=1` 在 HUD flags[7] 明确显示；不通过扩大标签容量来掩盖；
  - 32 只有球面渐变：完整 204x190 通过（证明碎片化由高光边界而非渐变导致）；
  - 33 平滑径向高光：完整 204x190 通过。
  - **高光判据的数值口径**：`isp_ccl_circle.v` 的 `SPEC_MIN=5'd24` 比较的是
    8bit 通道的**高 5 位**，即 `24<<3 = 192`，有效门限约 **192/255**。
    源码行内注释写作 “120/255” 是错误注释（120 对应 5'd15），属于注释问题，
    本轮构建源已锁定，未在构建期间改动；功能门限以 5'd24（≈192/255）为准。
  - 以上为合成场景，**拍屏照片不是摄像头原始 RGB，不能声称等价**；硬边高光
    与非白高对比反光边仍可能切碎目标，已如实标注。
- 仿真为合成场景，不含摄像头光学、噪声、运动模糊、Bayer 去马赛克误差与 HDMI 压缩；
  板端仍需人工确认。
- `tb_frontend` 直接给 AWB 之后的颜色；AWB 本身由 `tb_awb_apply` 单独验证，
  两者的联合收敛未做端到端波形。
- 标签表容量 15；彩色/多色背景场景可能出现 `O_ovf=1`（球仍可检出），
  复杂背景的颜色种类超过容量时仍会失败。本轮不扩大标签容量（避免时序/资源风险），
  只保证 `ovf` 在 HUD 上可见。
- POP 色相区分是 RTL 级证据，最终观感需上板确认。

## 第一阶段共享灰度回归（2026-11-01，实验性）

本节只描述**共享 RGB->Y 框架**的回归；完整的上板说明见
`../PHASE1_SHARED_GRAY_REPORT_20261101.md`。所有测试都直接实例化工程 RTL。

| 测试 | 实例化的 RTL | 覆盖 / 判据 |
|---|---|---|
| `tb/tb_gray_share.v` | `isp_gray_share96.v`、`edge_detect_y96.v`、`edge_detect_96.v`、`isp_pop96.v` | ① RGB->Y 公式逐位比对；② tap 1 拍延迟的 Y/valid/tlast/显示 tuser/检测 tuser/(x,y)；③ 新 EDGE（共享 Y）与旧 EDGE（RGB 输入）逐像素、valid/tuser/tlast 完全一致；④ POP 共享 Y 与内部 Y 逐像素一致；⑤ GRAY 显示 2 拍路径与基线 2 寄存器路径逐像素一致；⑥ 16x4 小帧的 tuser/行尾/坐标回卷，以及 1280 宽 tap 的 x=1276->0/y+1。显示 tuser 故意在 valid=0 拍上打入，验证两条 tuser 路径分离。 |
| `tb/tb_display_regression.v` | 完整 `isp_top`（可与 074ae1b 基线 RTL 同 TB 编译） | 真实 1280x720 行/场时序；POP→RAW→GRAY→EDGE 帧边界切换 + 最后一帧 POP→RAW 帧内切换；每 720 个 tlast 为一组，检查 valid=230400、tlast=720、tuser=1，打印 96bit `sel_data` 的逐组 HASH；统计 `O_frm_tgl` 翻转；显示 tuser off-valid 只统计不判错；POP 行首差异只作诊断。 |
| `tb/tb_dg_stream.v` | 完整 `isp_top`（新 RTL） | 检查预留 `O_dg_*`：4 像素/拍、Y 与 AWB 输出逐拍一致、tuser 必须在 valid 拍、x=0..1276 步长 4、y=0..719。完整 1 帧（GRAY）为 230400/720/1；`+LINES=3 +FRAMES=2` 的有界运行覆盖 GRAY→EDGE 两模式（每帧 960/3）。该流不接受 `I_mode`。 |

运行（ModelSim 10.3c，`vlib/vlog/vsim` 在 PATH）：

```bat
cd lab_ex4_ext_SC520\sim
run.bat                         :: 既有快速回归 + tb_gray_share
run_isp_top_display.bat         :: 旧的全 isp_top 诊断 TB（沿用，非本阶段验收依据）
```

`tb_display_regression` 的基线 A/B 需要单独编译 074ae1b 的
`isp_top.v` / `edge_detect_96.v` / `isp_pop96.v` 到另一个 work 库，再运行同一 TB；
逐组 `VALID/TLAST/TUSER/HASH` 必须一致。`tb_dg_stream` 只适用于新 RTL
（基线没有 `O_dg_*` 端口）。

### 预留检测灰度流接口（`isp_top` 端口）

**权威位序（后续圆检测必须按此实现）**：

| `O_dg_y4` 位域 | 像素 | 对应 AWB 输入 `I_rgb` 位域 |
|---|---|---|
| `[31:24]` | **x+0** | `[95:72]`（最左/最高像素） |
| `[23:16]` | **x+1** | `[71:48]` |
| `[15:8]`  | **x+2** | `[47:24]` |
| `[7:0]`   | **x+3** | `[23:0]`（最右/最低像素） |

```
O_dg_y4[31:24]  // Y(x+0) <- I_rgb[95:72]
O_dg_y4[23:16]  // Y(x+1) <- I_rgb[71:48]
O_dg_y4[15:8]   // Y(x+2) <- I_rgb[47:24]
O_dg_y4[7:0]    // Y(x+3) <- I_rgb[23:0]
O_dg_valid
O_dg_tuser      // 只来自与 valid 对齐的 AWB O_tuser_alg，不来自显示 tuser
O_dg_tlast
O_dg_x[11:0]    // 该拍 x+0 的列坐标 0,4,...,1276
O_dg_y[11:0]    // 行号 0..719
```

⚠️ `isp_gray_share96.v` 里的 `O_y4 = {Y3,Y2,Y1,Y0}` 是 **MSB-first 打包标签**，
**不是**像素序号：`Y3` 在 `[31:24]` = **x+0**；`Y0` 在 `[7:0]` = **x+3**。
不要把 `Y0` 当成 x+0。源码已锁定，本轮只改文档，不改 RTL/哈希；
后续检测器按上表位索引 / `x+0..x+3` 消费。

相对 AWB 输出 `S_rgb_data` 的延迟固定为 **1 个 CSI 时钟**；
`O_dg_tuser=1` 标记 (0,0)。该接口只提供 4 像素/拍的全分辨率灰度，
**没有**计算梯度/Sobel；EDGE 仍使用水平 2 像素平均、垂直 1 行缓存的
原有算法，不能把两者混称为“全分辨率梯度”。第一阶段没有接检测器，
综合可能裁剪这些端口；接上消费者后即成为活跃逻辑。
