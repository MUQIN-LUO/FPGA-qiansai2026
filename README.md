# FPGA-qiansai2026

全国大学生嵌入式芯片与系统设计竞赛 '2026 · FPGA 创新设计赛道（安路赛道）
**选题二：基于安路 PH1P35 的实时数字图像处理系统**

本仓库以两个**SC520CS 摄像头版本**的官方参考工程为基础，并已在 `lab_ex1_mipi_hdmi_SC520`
中完成**选题二全部四项基础要求**：MIPI 采集 + HDMI 显示、Sobel 边缘检测、
OSD 信息叠加（帧率文字 + 安路 Logo）。

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

## 3. 系统架构

```
SC520CS 摄像头
   │ 2-lane MIPI (10bit)
   ▼
MIPI D-PHY RX ──► CSI 解包 ──► RAW10 解包 ──► 转 AXI4-Stream
                                                    │
                                                    ▼
                              ISP：BLC → Demosaic(BGGR) → AWB
                                                    │
                                    ┌───────────────┴──────────────┐
                                    │  lab_ex2 额外插入：            │
                                    │  灰度 R/4+G/2+B/4              │
                                    │  → 2 行行缓存 → Sobel → 二值化  │
                                    └───────────────┬──────────────┘
                                                    ▼
                                   DDR2 帧缓存（写/读乒乓）
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
├─ lab_ex1_mipi_hdmi_SC520/          ← 基础要求(1)：采集 + 显示
│  ├─ td_project/                    ← TD 工程（.al 在这里）
│  │  ├─ camera_to_dsi_display.al    ← ★ 双击打开整个工程
│  │  ├─ camera_to_dsi_display.sdc   ← 时序约束
│  │  ├─ al_ip/                      ← PLL / FIFO / ChipWatcher 等 IP
│  │  └─ camera_to_dsi_display_Runs/ ← 编译输出（含预编译位流）
│  └─ user_source/
│     ├─ constraints_source/pin.adc  ← ★ 引脚约束
│     ├─ hdl_source/                 ← 全部 RTL 源码
│     └─ ip_source/                  ← IP 配置（.xml / .ipc）
│
└─ lab_ex2_edge_SC520/               ← 基础要求(2)：边缘检测
   └─ （目录结构同上）
```

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

## 9. 参考资料

- 《TangDynasty 用户手册 V6.2》《SWUG107 设计仿真指南》《SWUG110 BitWizard 指南》（TD 安装目录 `doc/`）
- 《UG1316 安路科技 PH1P 系列 FPGA MIPI 用户手册》
- 《SC520CS-03 数据手册 V1.6》《SC520CS 设计应用指南 V1.4》
- 《HX1P35B 开发板引脚分配汇总》《新 HX1P35A 开发板手册 HDL 版》
