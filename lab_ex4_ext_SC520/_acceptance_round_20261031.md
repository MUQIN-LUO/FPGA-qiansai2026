# SC520 圆形检测本轮验收报告（2026-10-31）

## 0. 约束遵守

- 未修改曝光/AE 参数、AE 控制策略、曝光 HUD（`uiisp_beta/`、`uics520_cfg/` 无改动）。
- 未烧录、未提交、未推送；未改动原工程 `_Runs/`、历史 bit 与 stash。
- POP 通路保留（未改 `isp_pop96.v` / `isp_pop.v`）。
- 构建在独立 scratch：`_build_scratch_final_20261031/`。

## 1. 代码改动

| 文件 | 改动 |
|---|---|
| `design_top_wrapper.v` | `S_det_flags` 纳入既有检测快照握手：`S_stg_flags → S_sn_flags → S_rc_flags`，HDMI 帧边界 `S_det_flags_p <= S_rc_flags`；不再直接采 CSI 域多位信号 |
| `isp/isp_top.v` | `isp_det_result` 增加 flags/cnt 原子锁存、`I_frame` 丢事件看门狗；CCL 增加 `O_live` 并进入 flags[6] |
| `isp/isp_ccl_circle.v` | 严格镜面白核（`SPEC_MIN=5'd24`，≈192/255）不切断游程、不更新代表色；新增 `O_live`；未扩大 15 标签容量 |
| `isp/isp_ccl_bridge.v` | 转发/保持 `O_live`（与 O_evt 同组） |
| `hdmi_mixer.v` | OSD 第 7 行 flags 位序注释更新 |
| `sim/tb/tb_det_result.v` | flags/cnt 原子性、坏帧丢事件看门狗 |
| `sim/tb/tb_frontend.v` | 场景 27–33：硬边高光（已知边界）、平滑高光+手指、真圆背景、复杂背景无球、标签耗尽、纯渐变、平滑高光 |
| `sim/README.md` | HUD 口径、已知边界、`SPEC_MIN` 注释勘误 |

### 1.1 flags 位定义（HUD）

`flags[7:0] = {ovf, live, chord, rows, asp, size, pass, valid}`

- 判据位 [5:2] **只在 pass=1 或候选时有效**；pass=0 且非候选时全 0，
  不能据此声称具体失败原因。
- 坏帧/丢拍无评估事件时，帧起始按 miss 衰减置信度；连续约 3 帧后旧绿框熄灭。

### 1.2 CDC 同组握手证据（结构，非仿真）

- staging：`design_top_wrapper.v:785-800` 同一 `if` 块写 bbox/valid/cand/cnt/flags。
- payload：`:802-820` 同一事务写 `S_sn_* + S_sn_flags` 后翻 `S_sn_tgl`。
- 接收：`:676-691` 同一 `S_snap_evt` 写 `S_rc_* + S_rc_flags` 并回 ack。
- 显示帧：`:694-713` 同一 `S_hdmi_user` 写 `S_det_*_p + S_det_flags_p`。
- `S_det_flags` 直接负载只有 `S_stg_flags`；`S_det_flags_p` 只由 `S_rc_flags` 驱动。

**未仿真验证**：wrapper 级 CSI→HDMI 快照与 HDMI 帧边界锁存没有直接 TB；
`tb_det_result` 只验证 `isp_det_result` 内部 flags/cnt/bbox 同拍原子性与看门狗。
不得把接线检查当作动态行为已验证。elaboration 网表查询结果见 §5。

## 2. 仿真结果（短定向 + 既有回归）

| 测试 | 结果 | 说明 |
|---|---|---|
| `tb_det_result` | PASS 0 errors | pass→candidate→empty、flags/cnt 原子、3 帧丢事件清绿框 |
| `tb_ccl_regression` | PASS 0 errors | DUT==REF 全部场景 |
| `tb_ccl_color` | PASS 0 errors | 异色分离/同色合并/红肤边界 |
| `tb_bridge` | PASS 0 errors | FIFO/CDC/丢拍恢复 |
| `tb_frontend` 场景 27 | KNOWN-LIMIT | 硬边 50% 高光：pass=1 但 72x65 高光块占框；保留失败样例 |
| `tb_frontend` 场景 28 | PASS | 平滑高光+压球手指：球 196x190，手球分离 |
| `tb_frontend` 场景 29 | PASS(记录歧义) | 红球+背景真圆：实际选中背景蓝盘 128x120；形状无法判断目标 |
| `tb_frontend` 场景 30 | PASS | 20 背景块无球：不误报 |
| `tb_frontend` 场景 31 | PASS(记录边界) | 16 背景块：`O_ovf=1`，球被丢弃；不扩标签 |
| `tb_frontend` 场景 32 | PASS | 纯球面渐变：204x190 |
| `tb_frontend` 场景 33 | PASS | 平滑高光：204x190 |

**已知边界（如实保留）**
1. 硬边/非白高对比反光边仍可能把球切碎；场景 27 保留失败结果。
2. 背景本身是真圆时，形状判据无法区分目标；场景 29 不声称球必胜。
3. 15 标签耗尽时新目标被丢弃，仅保证 HUD `ovf=1` 可见；未扩大容量（避免时序/资源风险）。
4. 拍屏照片不是摄像头原始 RGB，不能声称与仿真等价；未做真实摄像头 RGB 回放。

## 3. 构建输入 SHA（独立交付核对）

- **TD 实际使用的 `.al` 条目：68 项**。按内容哈希（不看路径）排序后整体 SHA256：
  `580b594e514ca54ba97871f5cc4aedb105ea90a02a0bf6053cd4f84d745ace66`
  - **当前工作树与 scratch 完全一致**（同一算法得到同一值）。
  - `.al` 文件列表与 syn/phy 实际使用的 `.prj` 列表（绝对路径归一化后）**完全相同**。
  - 68 项逐文件比较：**0 处不一致**。
- `.al` 本身：
  - 当前工作树 `39faa87ec512cbfc1e8f588313a713b52538edb8011cc7348ab6bffcf9597497`
  - scratch `b92666ef6fa444be1d421b0a5dd5fcdd05857b976f6f320eed8b9adff16156f6`
  - 差异仅为 `<Project Path="...">` 指向 scratch；把 `Path` 归一化后 `diff` 为空。
- `camera_to_dsi_display.sdc`：两份同为
  `e8c6e4d59821d775f6919dab03ae19f9e7431a850f13a2a0fd9cacb0882c09b7`
  （未放宽、未修改）。
- 更宽的 111 项清单（全部 `.v/.adc/.sdc/.al`）：主工程 vs scratch **110/111 一致**，
  唯一差异仍是 `.al` 的 `Path`。
- 关键文件 SHA256：
  - `design_top_wrapper.v` `fa88b631647fba728db43c5d3b2f86c6bb3191d742ff3bd91106c845c5f50eeb`
  - `hdmi_mixer.v` `355626a8f331b4e8b8194afbbf3a0af4ea496d4f1ac5bfc2e2288751b13925e8`
  - `isp_top.v` `bd0393bb671ad3d1356355a8c7bf5dea3659c775ff5d03bcf04f4b0100e2f2b0`
  - `isp_ccl_circle.v` `6703fbb70425d3f8707dba93ffd42821bc463e13ae6f07765c75a836b953dd22`
  - `isp_ccl_bridge.v` `6ae9eb53089cfdd2cecae4c08740d9e6eb2b9787495db498c8e43e8ca348ce54`

## 4. 综合

- setup：WNS **+680 ps**，TNS 0，0 违例
- hold：WNS **+114 ps**，TNS 0，0 违例
- 流程：`syn_1` read_design → opt_rtl → opt_gate 全部成功。

## 5. 布局布线 / bitgen

- final setup：WNS **+91 ps**，TNS 0，0 违例（`camera_to_dsi_display_phy.ts`）
- final hold：WNS **+20 ps**，TNS 0，0 违例
- STA coverage：**96.26%**（`camera_to_dsi_display_pr.timing` / `check_timing.rpt`）
- `check_timing` 的未约束/例外项（原始报告已保存）：
  - 1 个无根时钟的网络（HIGH）：`u_uicfgcs520/u_i2c/scl_clk`（I2C SCL 分频时钟，
    未建 `create_clock`/`generated_clock`）。
  - 37 个输入端口无 input delay、68 个输出端口无 output delay、1 个输入
    partial input delay（MEDIUM）：MIPI/DDR/按键/HDMI TMDS 等板级 I/O，
    不在本次内部寄存器路径 STA 范围内；SDC 未新增/放宽约束。
  - 2 条被 `set_clock_groups` 忽略的 DDR 格雷码 `set_max_delay` 例外
    （异步 CDC，预期忽略）。
  - 0 条 combinational loop、0 条 invalid constraint、0 条 no-path constraint。
- 时钟：`MIPI_RX_BYTE_CLK` 9.660 ns、`ccl_clk` 40.000 ns（25 MHz）、
  `HDMI_PIXEL_CLK` 13.333 ns、`sys_clk_50m` 20.000 ns、`pll_clk_24m` 42.000 ns 等；
  `route_clock_utilization.txt` 已记录。
- 位流（scratch 内，未覆盖原工程历史 bit）：
  `D:\FPGA\project\_build_scratch_final_20261031\lab_ex4_ext_SC520\td_project\camera_to_dsi_display_Runs\phy_1\camera_to_dsi_display.bit`
  - 大小：1,789,539 bytes
  - SHA256：`e71a9adc0e6ee4be1c0392fa04387af693f500ff1177bb4fec90d27465af4efd`
  - 未烧录。

### 5.1 flags 连通性（elaboration 网表，非动态仿真）

在 `syn_1` 内 `open_project + elaborate` 后查询（脚本 `verify_flags_cdc*.tcl`）：

- `S_det_flags[0]` 负载：`S_stg_flags_b1_i7.I`（CSI staging 寄存器 D）+
  `u_isp_top.O_det_flags[0]`；**没有 HDMI 域负载**。
- `S_stg_flags[0]` 负载：`S_sn_flags_b1_i7.I`（payload 寄存器）。
- `S_rc_flags[0]` 负载：`S_det_flags_p_b1_i7.I`（HDMI 域寄存器 D）。
- `S_det_flags_p[0]` 负载：`S_det_flags_p_reg.q`、`mux55`、
  `u_hdmi_mixer.I_det_flags[0]`。
- `S_det_flags_p_reg.clk`、`S_det_minx_p_reg.clk`、`S_det_valid_p_reg.clk`
  同为 `S_hdmi_pixel_clk`；三者的 `en` 同为 `_al_n1`（`S_hdmi_user` 使能）。
- `S_stg_flags_reg.clk` 与 `S_stg_minx_reg.clk` 同为 `S_csi_rx_clk`。

这说明 flags 与 bbox/valid 在**同一时钟/同一使能**下成组寄存，且
CSI 域 flags 不再直接进入 HDMI 域。**但仍没有 wrapper 级动态仿真**；
上述是网表连通性证据，不等价于握手时序/帧锁存行为已仿真验证。

## 6. 未验证 / 不要声称

- wrapper 级 CDC 快照握手与 HDMI 帧边界锁存的**动态行为未仿真验证**；
  现有证据是源码结构 + elaboration 网表连通性（同 clk/en）。
- `SPEC_MIN=5'd24` 源码行内注释 “120/255” 有误；实际按高 5 位比较，
  约 **192/255**（24<<3=192）。构建源已锁定，未在构建期间改源码。
- 未做真实摄像头实拍、未上板、未烧录。
- 真实光学高光、运动模糊、噪声未覆盖；合成高光模型不能替代原始 RGB。
- 硬边高光（场景 27）、背景真圆（场景 29）、15 标签耗尽（场景 31）
  仍为已知边界。
