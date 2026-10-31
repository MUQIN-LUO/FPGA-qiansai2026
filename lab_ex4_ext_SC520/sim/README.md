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
- 仿真为合成场景，不含摄像头光学、噪声、运动模糊、Bayer 去马赛克误差与 HDMI 压缩；
  板端仍需人工确认。
- `tb_frontend` 直接给 AWB 之后的颜色；AWB 本身由 `tb_awb_apply` 单独验证，
  两者的联合收敛未做端到端波形。
- 标签表容量 15；彩色/多色背景场景可能出现 `O_ovf=1`（球仍可检出），
  复杂背景的颜色种类超过容量时仍会失败。
- POP 色相区分是 RTL 级证据，最终观感需上板确认。
