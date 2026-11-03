# FPGA-qiansai2026

全国大学生嵌入式芯片与系统设计竞赛 '2026 · FPGA 创新设计赛道（安路赛道）
**选题二：基于安路 PH1P35 的实时数字图像处理系统**

本仓库以两个**SC520CS 摄像头版本**的官方参考工程为基础，并已在 `lab_ex1_mipi_hdmi_SC520`
中完成**选题二全部四项基础要求**：MIPI 采集 + HDMI 显示、Sobel 边缘检测、
OSD 信息叠加（帧率文字 + 安路 Logo）。

在基础要求之上，`lab_ex4_ext_SC520` 完成了多算法切换、波普风格化、处理延迟可视化
等拓展要求，并在 `feat/phase2-circle` 分支实现了**绿色圆球识别 + 运动跟踪 +
《两只老虎》蜂鸣器**的第二阶段原型（最终构建 rev 2p）。

含完整 TD 工程、全部源码、引脚/时序约束，以及预编译位流。

---

## 1. 硬件平台

| 项 | 型号 |
|---|---|
| 开发板 | HX1P35A（丝印 HX1P35A V1.0） |
| 主芯片 | 安路 PH1P35MDG324（BGA324，-3 速度等级） |
| 摄像头 | **SC520CS**（2-lane MIPI CSI-2，10bit，24P FPC，经 40P→24P 软线接入） |
| 显示 | HDMI 输出，1280×720@60Hz |
| 下载器 | 板载 AL-LINK-FT（FT2232H，A 通道 WinUSB→JTAG，B 通道 FTDI CDM→串口 COM9） |
| 工具链 | Anlogic TangDynasty **TD 6.2.1** |
| 仿真器 | ModelSim（需安路配套版，含 `anl_ph1p`/`anl_common` 库） |

## 2. 与赛题要求的对应关系

| 赛题基础要求 | 对应实现 | 状态 |
|---|---|---|
| **(1)** MIPI 摄像头采集 + HDMI 实时输出 | `lab_ex1` 主通路 | ✅ |
| **(2)** FPGA 端完成至少一种图像处理算法 | `lab_ex1` 的 Sobel 边缘检测（原 `lab_ex2` 移植） | ✅ |
| (3) 处理后视频稳定输出至 HDMI | SWNS 0.106 ns / HWNS 0.021 ns，无违例 | ✅ |
| (4) OSD 信息叠加（含"安路"Logo） | `lab_ex1`：右上角 Logo + 左上角帧率文字 | ✅ |

### 2.1 `lab_ex1` 中的实现细节

#### 新增文件

| 文件 | 作用 |
|---|---|
| `user_source/hdl_source/isp/edge_detect_96.v` | Sobel 边缘检测（移植自 `lab_ex2`，模块内容未改） |
| `user_source/hdl_source/anlogic_logo_rom.v` | 安路 Logo 位图 ROM（160×160，25bit/像素 = `{不透明标志, RGB888}`） |

#### 修改文件

| 文件 | 改动 |
|---|---|
| `isp/isp_top.v` | 在 `awb` 与 96→128bit 打包之间接入边缘检测；输入侧加一级流水寄存器，切断 AWB 乘法器到 Sobel 的 14 级组合逻辑（原违例 -1.221 ns） |
| `hdmi_mixer.v` | 新增帧率 OSD（左上角，8×8 点阵放大 3 倍，亮绿+黑描边，背景透明）与 Logo 叠加（右上角 `LOGO_X = 1280-160`） |
| `design_top_wrapper.v` | LED1 心跳灯；帧率统计（0.5 s 窗口计帧数 ×2，电平翻转跨时钟域到像素域） |
| `constraints_source/pin.adc` | 新增 `O_led1 = D10` |
| `td_project/camera_to_dsi_display.sdc` | 补全 MIPI RX、PLL 各输出时钟约束 |

#### 功能说明

| 功能 | 实现 |
|---|---|
| LED1 心跳 | 50 MHz 分频，半周期 25,000,000 拍 → 1 Hz，引脚 D10 |
| 帧率 OSD | 左上角，每 0.5 s 统计一次 ISP 帧起始数并 ×2；前导零不显示 |
| 边缘检测 | 灰度 `R/4+G/2+B/4` → 2 行 ERAM 缓存 → Sobel（移位代替 ×2）→ `\|Gx\|+\|Gy\| ≥ EDGE_TH` 二值化；阈值在 `isp_top.v` 的 `.EDGE_TH`，范围 0~2040（当前 128） |
| 安路 Logo | 右上角 160×160 区域，按 ROM 的 bit24 做透明叠加，不遮挡画面其余部分 |

#### 注意事项

- 新增 `.v` 文件必须通过 TD 的 `Source → Add Source...` 加入工程，直接改 `.al` 会在 TD 保存工程时被覆盖
- PLL 的 CLK1 必须是 **24 MHz（分频 63）**；改成其他频率会导致传感器寄存器配置与实际时钟不符
- 阈值偏低噪点多、偏高只剩强边缘，按现场光照在 96 ~ 400 之间调

### 2.2 `lab_ex4` 的拓展实现

`lab_ex4_ext_SC520` 由 `lab_ex1` 复制而来，用于拓展要求（2.4）与进阶要求（2.5）。

#### 多算法切换（进阶要求 1）

板载 K1~K4 按键实时切换算法，OSD 第二行显示当前模式：

| 按键 | FPGA 引脚 | 功能 |
|---|---|---|
| K1 | D5 | 原图 `RAW` |
| K2 | A9 | 边缘 `EDGE`（Sobel 二值化） |
| K3 | B9 | 灰度 `GRAY` |
| K4 | C7 | 波普 `POP`，再按循环四档 |

- 按键内部上拉，按下为低；20 ms 消抖后取下降沿（`design_top_wrapper.v`）
- 四条通路在 ISP 内统一延迟 2 拍后三选一，切换时画面不错行
- ⚠️ 按键改作模式切换后，`ae_set` 的按键输入固定为 `4'b1111`，**AE/AG 保持默认值**不再手动可调

#### 波普风格化（扩展要求 1）

还原安迪·沃霍尔《玛丽莲·梦露》丝网印刷系列的三个特征：

| 特征 | 实现 |
|---|---|
| 色调分离 | 灰度 `R/4+G/2+B/4` → 按 64/128/192 量化成 4 档 |
| 假色彩 | 4 档映射到 4 色平涂调色板，共备有 4 组配色 |
| 自动轮换 | 每 3 秒换一套配色，对应系列"同图多配色"的展陈方式 |
| 套印错位 | 中间调色层按 x-2 位置的档位着色，形成 2 像素向右的套色毛边 |

配色方案（`isp_top.v` 的 `pop_pal`）：

| 层 | 方案 0 | 方案 1 | 方案 2 | 方案 3 |
|---|---|---|---|---|
| 0 最暗 | 深紫 `7b2d8e` | 深绿 `00574b` | 深红 `d32f2f` | 黑 `202020` |
| 1 | 皮肤粉 `ff9ec4` | 米黄 `ffe0b2` | 浅粉 `ffc0cb` | 黄 `ffd400` |
| 2 | 头发黄 `ffd400` | 青 `00e5ff` | 深蓝 `1e5aa8` | 红 `ff3b30` |
| 3 最亮 | 背景青 `00c8c8` | 品红 `ff00aa` | 橙 `ff7a00` | 白 `f2f2f2` |

调色板编号由 `design_top_wrapper.v` 的 3 秒计数器产生，跨到 CSI 域后送入 ISP。

#### 处理延迟可视化（进阶要求 3）

OSD 第三行实时显示板级端到端延迟（毫秒）：

```
FPS: 60
POP
LAT:33MS
```

**测量方法**：DDR 乒乓缓冲为 4 槽，帧起始时写指针先自增再用旧值选槽，
因此"正在写入的槽"是 `wp-1`，读槽为 `rp`，缓冲帧数 = `(wp-1) - rp`。
再按实测帧率换算成毫秒：

```
延迟(ms) = 缓冲帧数 x 1000 / 实测帧率
```

60 fps、2 帧缓冲时约 33 ms。换算由 `design_top_wrapper.v` 中一个 12 拍串行
除法器完成，每 0.5 s 更新一次，不占用关键路径。

**指标边界**：包含 DDR 帧缓冲的帧级延迟（主要部分），不含传感器曝光到
MIPI 输出的时间与像素行级流水（< 1 ms），因此可对外表述为
「板级端到端显示延迟约 33 ms（2 帧缓冲）」。

**注意事项**：单位必须写全 `MS`，只写 `M` 会被误认为分钟。
量化阈值与配色均在 `isp_top.v` 的 `pop_band` / `pop_pal` 中，可直接修改。

#### 固定增益白平衡（图像质量基础）

原厂 `awb` 是**灰度世界**算法，而且每拍都跑两个除法器实时求 `G/R`、`G/B`。两个问题：

1. 增益逐拍变化 → 画面颜色抖动/闪烁
2. 把 R、B 强行拉到等于 G → 有色背景被中和，**整帧 G−B 恒为 0，黄色目标被洗淡**

现改为**固定增益**（Q16 定点，`65536 = 1.00×`），G 通道为 1.0 基准：

| 参数 | 值 | 实测倍数 |
|---|---|---|
| `WB_GAIN_R` | `20'd80717` | 1.2316× |
| `WB_GAIN_B` | `20'd121075` | 1.8475× |

**标定方法**（不需要色卡）：把白纸填满画面中心的 64×64 取样框，读 OSD 第三、四行的
R/G/B 平均值，则 `gain = 65536 × G / 该通道均值`。

**实测**：标定时（增益 1.00×）读 `R177 G218 B118`，标定后读 `R063 G064 B063`，
最大偏差 1 LSB，白纸已呈中性。

副产品：删掉 8 个除法器 IP，乘法退化为常数移位加法，面积和时序都更宽松。

拜耳相位经实测确定为 **BGGR**（同一 teal 背包在 RGGB 下呈橄榄绿，即 R/B 互换）。

#### 已知限制

| 项 | 状态 |
|---|---|
| 黄色目标检测 | 代码完整但 `DET_EN = 1'b0`：开启后 SWNS = **-3.407 ns** 违例，需先给 `yellow_hit` 输出打一拍流水 |
| 黑电平校正 | `BLC.v` 存在但**未例化**，原始基座保留了；`raw10→8bit` 是纯 `>>2`，没有减法 |
| Gamma | **完全没有**。线性光直出 HDMI，显示器按 sRGB 解释 → 中间调偏暗、饱和度发灰 |
| 自动曝光 | 未实现。按键改作模式切换后 `ae_set` 固定为 `4'b1111`，曝光保持默认值 |

### 2.3 第二阶段：绿色圆球识别与蜂鸣器（实验性）

> 分支 `feat/phase2-circle`，最终构建 rev 2p。**未上板验证**：位流已生成但未烧录；
> 低对比绿球漏检问题仍未解决，不能据此宣称识别稳定性已解决。

在 `lab_ex4_ext_SC520` 的共享灰度通路上新增一条与主视频**并行**的检测链路，
不改变原有算法切换、波普、延迟显示行为：

```
共享灰度 (R/4+G/2+B/4)
  └─► 4×4 块梯度事件前端 (isp_grad_event96)
        有符号 Sobel + 8-bit 方向量化，1 事件/块，带溢出计数
        └─► 中心投票 + 径向直方图验证 (isp_circle_hough2)
              去掉全分辨率边缘缓存；4px 中心网格投票 → 细中心 →
              r=33..160 径向直方图 + 扇区覆盖/填充率/锐度评分
              └─► 跨帧跟踪 (isp_ball_track)
                    连续 10 次小范围稳定才锁框；
                    运动门限 1.5r+16 / 20%r+2；
                    稳定后 4 个真实相机帧 hold + 200 ms 硬超时
                    └─► bbox + `CIRCLE X±ddd Y±ddd` OSD
                          └─► 球内颜色采样 (isp_green_sample)
                                绿色目标文字扩展为 `CIRCLE GREEN ...`（不参与 valid）
```

| 功能 | 实现 | 状态 |
|---|---|---|
| 绿色圆球识别 | `isp_grad_event96` + `isp_circle_hough2` | 合成场景通过；真实低对比绿球仍漏检 |
| 运动球跟踪 | `isp_ball_track`，连续 10 次稳定才锁框 | 定向仿真 PASS |
| 识别框与坐标 | `CIRCLE X±ddd Y±ddd`，绿色目标扩展为 `CIRCLE GREEN ...` | 逐像素仿真 PASS |
| 运行时长 | 右下角 `DD:HH:MM:SS`，50 MHz 独立计时，request/ack 跨域原子搬运 | 仿真 PASS |
| 蜂鸣器 | `isp_two_tigers`：32 音符 ROM、120 BPM、16 s；每次真实检测触发，连续 1 s 无真实检测则停止/静音/重臂 | 定向仿真 PASS；**未上板** |

**触发语义**：只有检测器当帧真正给出 `pass`（`O_det_fresh_tgl` 翻转）才计时；
跟踪器的绿框 hold、跳帧都不参与。播放中再次检测只清零看门狗，不从头重播；
播完目标仍在时不会循环，消失 1 s 后再出现才从第一个音重播。

**资源与时序（rev 2p 最终构建）**：slices 17,268/21,216 = **81.39%**，
ERAM 86/108，DSP 27/40；setup WNS **+144 ps**、hold WNS **+20 ps**，0 违例。
最终位流 `camera_to_dsi_display_rev2p_two_tigers_timeout.bit`
（SHA256 `56bddade…fb8e`，1,789,539 bytes），**未烧录**。

**已知限制（不得隐去）**：

- 低对比绿球（如 RGB 40,200,40 / 140 灰，r60）仍漏检；
- 半径 >32 px 的填充圆形高亮仍可能被框，形状法无法与球区分，本轮未加颜色门限；
- 蜂鸣器仅通过定向仿真，未在板级喇叭上验证；
- 检测 toggle 跨域要求真实检测间隔 > 2 个 50 MHz 周期（相机帧率下远满足）；
  复位释放后 3 周期消隐内的事件会丢一次，下一帧补上。

完整研究记录、模型/事件级/像素级证据与逐版构建数据见
`lab_ex4_ext_SC520/PHASE2_CIRCLE_RESEARCH_20261101.md`。

## 3. 系统架构

```
SC520CS 摄像头
   │ 2-lane MIPI (10bit)
   ▼
MIPI D-PHY RX ──► CSI 解包 ──► RAW10 解包 ──► 转 AXI4-Stream
                                                    │
                                                    ▼
                              ISP：BLC → Demosaic(BGGR) → AWB
                                     │                   │
                       ┌─────────────┘                   └──────────────┐
                       ▼                                                ▼
             lab_ex2 显示通路                                   lab_ex4 检测通路（旁路）
   灰度 R/4+G/2+B/4 → 2 行缓存 → Sobel → 二值化      共享灰度 → 4×4 块梯度事件
                       │                                → 中心投票 + 径向直方图
                       ▼                                → 跟踪 / CIRCLE OSD / 蜂鸣器
              DDR2 帧缓存（写/读乒乓）                        （不进入主视频）
                       ▼
          VTC 视频时序 → HDMI Mixer → HDMI TX(TMDS)
                       ▼
             HDMI 显示器 1280×720@60
```

### 时钟结构（取自 `camera_to_dsi_display.sdc`）

| 时钟 | 频率 | 用途 |
|---|---|---|
| `I_sys_clk` | 50 MHz | 板载有源晶振，PLL 输入 |
| PLL clk0 | 100 MHz | MIPI D-PHY LP 时钟 |
| PLL clk1 | 24 MHz | 摄像头 XCLK / I2C 配置 |
| PLL clk4 | 74.25 MHz | HDMI 像素时钟 |
| PLL clk5 | 371.25 MHz | HDMI 串行时钟（5×） |
| MIPI RX | 240 MHz（4.167 ns） | D-PHY 参考时钟 |
| DDR2 | 控制器输出 | 与上两组为异步时钟组 |

### 关键规模（Phy 后实测，lab_ex1）

LUT 10,002 ｜ REG 13,396 ｜ SLICE 12,390 ｜ ERAM 39 ｜ DSP 8 ｜ PLL 3 ｜ GCLK 3 ｜ IO 17
时序：SWNS = **1.242 ns**，HWNS = **0.020 ns**（无违例）

## 4. 目录结构

```
FPGA-qiansai2026/
├─ lab_ex1_mipi_hdmi_SC520/          ← 基础要求(1)~(4)：采集 + 边缘检测 + OSD + Logo
│  ├─ td_project/                    ← TD 工程（.al 在这里）
│  │  ├─ camera_to_dsi_display.al    ← ★ 双击打开整个工程
│  │  ├─ camera_to_dsi_display.sdc   ← 时序约束
│  │  ├─ al_ip/                      ← PLL / FIFO / ChipWatcher 等 IP
│  │  └─ camera_to_dsi_display_Runs/ ← 编译输出（重新编译生成）
│  └─ user_source/
│     ├─ constraints_source/pin.adc  ← ★ 引脚约束
│     ├─ hdl_source/                 ← 全部 RTL 源码
│     │  ├─ isp/edge_detect_96.v     ← Sobel 边缘检测
│     │  └─ anlogic_logo_rom.v       ← 安路 Logo 位图 ROM
│     └─ ip_source/                  ← IP 配置（.xml / .ipc）
│
├─ lab_ex2_edge_SC520/               ← 官方参考工程：仅边缘检测
│  └─ （目录结构同上）
│
└─ lab_ex4_ext_SC520/                ← ★ 拓展要求 + 第二阶段开发工程（由 lab_ex1 复制）
   ├─ user_source/hdl_source/isp/
   │  ├─ isp_grad_event96.v          ← 4×4 块梯度事件前端
   │  ├─ isp_circle_hough2.v         ← 中心投票 + 径向直方图圆检测
   │  ├─ isp_ball_track.v            ← 跨帧跟踪 / 稳定 / hold
   │  ├─ isp_green_sample.v          ← 球内颜色采样（GREEN 标签）
   │  ├─ isp_two_tigers.v            ← 《两只老虎》蜂鸣器
   │  ├─ isp_evt_hud_map.v           ← 检测 HUD 位映射
   │  └─ isp_rtc_cdc.v               ← 运行时长跨时钟域搬运
   ├─ sim/tb/                        ← 第二阶段定向仿真（跟踪/文字/蜂鸣器）
   └─ PHASE2_CIRCLE_RESEARCH_20261101.md  ← 研究记录与逐版证据
   （其余目录结构同 lab_ex1，不含编译产物，首次打开需完整综合）
```

> `lab_ex1` 作为**基础要求的定版基线**保留，后续拓展要求（2.4 / 2.5）与第二阶段
> 圆球识别在 `lab_ex4_ext_SC520` 中开发，避免验证过的基线被实验性改动污染。

## 5. 如何打开工程

1. 安装 **TD 6.2.1**（资料包 `1_安路工具和软件\安路开发软件TD_6.2.1版本.zip`）
2. 双击 `lab_exN_.../td_project/camera_to_dsi_display.al`（或 TD 里 `File → Open Project`）
3. 确认左侧 `Hierarchy Navigation` 显示 **`PH1:PH1P35MDG324-3`**，且 `Disabled Source Files` 为 **(0)**
4. **不要移动或重命名任何子目录**——`.al` 内全部使用相对路径（`../user_source/...`）

> 器件设置：`Family = PH1`，`Device = PH1P35MDG324`，`Speed = -3`

## 6. 预编译位流清单

`lab_ex1_mipi_hdmi_SC520/td_project/camera_to_dsi_display_Runs/`

| 文件 | 用途 |
|---|---|
| `best_result/camera_to_dsi_display.bit` | 标准版（摄像头实时画面） |
| `phy_1/camera_to_dsi_display_1280x720_60fps.bit` | 720p@60 |
| `phy_1/camera_to_dsi_display_1600x900_30fps.bit` | 1600×900@30 |
| `phy_1/camera_to_dsi_display_1792x1080_normal.bit` | 1792×1080 |
| `phy_1/camera_to_dsi_display_1080p_colorbar_test.bit` | 1080p 彩条（**不用摄像头**） |
| `phy_1/camera_to_dsi_display_1280x720_75M_HDMI_COLORBAR_TEST.bit` | 720p 彩条（**不用摄像头**） |
| `phy_1/camera_to_dsi_display_hdmi_core_tpg.bit` | TPG 测试图（**不用摄像头**） |
| `phy_1/camera_to_dsi_display_hdmi_core_tpg_vic0_75m.bit` | TPG 测试图（75MHz 版） |
| `phy_1/compiled.bit` | ChipWatcher 编译产物，忽略 |

烧录：TD → `BitWriter` → `Add` 选位流 → `Mode = PROGRAM_SRAM` → `Run`
（SRAM 掉电丢失；最终演示前用 `PROGRAM_FLASH` 烧进板载 GD25WQ128 Flash，约 110 秒）

第二阶段位流**未随仓库提交**，位于本地构建快照
`../phase2_build_rev2p_timeout/deliverable/`：

| 文件 | 用途 |
|---|---|
| `camera_to_dsi_display_rev2p_two_tigers_timeout.bit` | 圆球识别 + 跟踪 + 《两只老虎》蜂鸣器（1 s 真实检测看门狗） |

SHA256 `56bddade58945fee33457038c910d807a30a31d7322f0f8f047992425cc3fb8e`，
1,789,539 bytes，**未烧录**。

## 7. 图像处理算法说明（lab_ex2）

模块：`user_source/hdl_source/isp/edge_detect_96.v`

- 输入 AXI4-Stream，**1 拍 = 96 bit = 4 个 RGB888 像素**
- 灰度：`Y = R/4 + G/2 + B/4`（纯移位加法，**零 DSP**，最大值 253 不溢出）
- 降采样：每拍 4 像素两两平均 → 640 个水平样本，行缓存减半
- 行缓存：`line1_mem` / `line2_mem`，320×16bit，`ram_style="block"` → 映射到 ERAM
- 边缘：Sobel（`<<<1` 代替 ×2），`|Gx| + |Gy|` 代替开方
- 二值化：`mag >= EDGE_TH` → 白，否则黑
- 阈值：`isp/isp_top.v` 第 105 行 `.EDGE_TH(11'd80)`（**编译期 parameter，改后需重新综合**）

## 8. 注意事项

- ⚠️ `*.enc.v` / `*.enc.sv` 为**厂商加密 IP**（MIPI D-PHY、HDMI TX 核心、DDR 控制器），
  只能例化、不可修改，且**本仓库应保持私有**，请勿公开传播。
- ⚠️ `*_Runs/` 是编译输出目录，**重新跑 Phy Opt 会被覆盖**，预编译位流请先另行备份。
- 摄像头型号务必与工程匹配：本仓库为 **SC520CS** 版本；
  SC500CS 需使用 `lab_ex*_SC500_720P` 系列工程（顶层例化 `uicfgcs500`）。
- 差分对（MIPI / HDMI）的引脚约束通常只约束 P 端，具体见 `pin.adc`。
- 第二阶段检测器由 `isp_top.v` 的 `EVT_CIRCLE` 参数选择（默认 1）；
  `EVT_CIRCLE=0` 时完整保留第一阶段 CCL/形状检测，两种配置都写入约束文件。
- 蜂鸣器 `O_buzzer` 约束在 **C12**（LVCMOS33，8 mA），`pin.adc` 原先无 C12；
  使用前确认板上 C12 已接无源蜂鸣器，否则该引脚无输出负载。
- 识别框、`GREEN` 标签与运行时长 OSD 均只改显示，不改变检测 `valid` 语义。

## 9. 参考资料

- 《TangDynasty 用户手册 V6.2》《SWUG107 设计仿真指南》《SWUG110 BitWizard 指南》（TD 安装目录 `doc/`）
- 《UG1316 安路科技 PH1P 系列 FPGA MIPI 用户手册》
- 《SC520CS-03 数据手册 V1.6》《SC520CS 设计应用指南 V1.4》
- 《HX1P35B 开发板引脚分配汇总》《新 HX1P35A 开发板手册 HDL 版》
