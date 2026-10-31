# lab_ex4_ext_SC520 —— 新设备构建与迁移说明

适用对象：在一台**新电脑**上 clone 本仓库后，要重新构建 `lab_ex4_ext_SC520` 的人。
工具：Anlogic TangDynasty（TD）6.2.1，器件 PH1P35MDG324 / speed -3（**Flash** 模型）。

> 本文件已随 CCL 时序优化更新。§4 保留 2026-10 之前“时序未通过”的历史，
> 并明确区分**已解决**与**仍未验证**的内容。

---

## 1. 先说结论：什么可迁移，什么没验证

**可迁移（已验证）**
- `.al` 里 **70 个文件条目全部是相对路径**，相对于 `.al` 自己所在目录：
  - 68 条形如 `../user_source/...`（62 条 `hdl_source`，含新增的
    `isp_mask_fifo.v`、`isp_ccl_bridge.v`；6 条 adc/IP xml）
  - 2 条形如 `camera_to_dsi_display.sdc`、`al_ip/ChipWatcher_mipi/ChipWatcher_mipi.xml`
  - **没有任何一条是绝对路径**（`grep -E 'Path="(/|[A-Za-z]:)'` 无结果）。
- 约束与 IP 均已入库：`td_project/camera_to_dsi_display.sdc`、
  `user_source/constraints_source/pin.adc`、`user_source/ip_source/**`（19 个 xml/ipc）、
  `user_source/hdl_source/isp/demosaic_4x_2_0/**`（6 个 xml）。
- 回归测试已入库：`sim/`（见 §6），测试直接实例化工程 RTL。

**未验证（不要当成已解决）**
- `.al` 顶部的 `<Project ... Path="...">` 属性里是**某台机器上的绝对路径**。
  观察到 TD 在 GUI 里跑过一次流程后会把该属性改写成当前机器路径；
  但**没有证据证明**旧绝对路径在新机器上一定被忽略。若提示工程路径无效，
  用 TD 的重新定位（Relocate）指到 `td_project`。
- 本仓库**不包含**任何已构建产物（`*_Runs/` 在 `.gitignore` 中排除）。

---

## 2. 新设备上的操作步骤

```
git clone <repo>
cd <repo>/lab_ex4_ext_SC520/td_project
```

**方式 A（GUI，推荐）**
1. TD → Open Project，选 `td_project/camera_to_dsi_display.al`。
2. 若提示工程路径无效/文件找不到：用 TD 的重新定位（Relocate）把工程目录指到
   `lab_ex4_ext_SC520/td_project`。
3. 打开后逐项核对 §3 的清单。
4. 在 Design 流程里重建 Run（`syn_1` / `phy_1`）。**必须重建 Run**，不要复用
   别人机器上的 `_Runs/` 目录。

**方式 B（命令行）**
1. 先按方式 A 打开并保存一次，让 TD 依据 `.al` 生成
   `td_project/camera_to_dsi_display_Runs/{syn_1,phy_1}`。
2. 分别在 `syn_1`、`phy_1` 目录下执行
   `td_commands_prompt.exe <无空格路径>/DefaultFlow.tcl`。
   ⚠️ `td_commands_prompt.exe` 会把含空格的参数按空格拆开，
   因此先把 `DefaultFlow.tcl` 复制到无空格路径；另有两点：
   - 生成的 `settings.cfg` 可能缺少 `device_name`/`package_name`，需补
     `set device_name ph1_35p.db`、`set package_name PH1P35MDG324`；
   - `phy` 的 `settings.cfg` 中 `parent` 必须指向对应 `syn` 目录。

---

## 3. 打开后必须核对的清单

| 检查项 | 期望值 | 为什么 |
|---|---|---|
| 文件条目数 | **70** | 少一个就会缺 design unit |
| `isp_ccl_circle.v` | 在（CompileOrder 61） | 连通域圆形检测 |
| `isp_gray_world.v`、`isp_shape_circle.v` | 在（58/59） | 灰度世界 AWB、前景掩膜 + 圆形判据 |
| `isp_mask_fifo.v` | 在（64） | 掩膜跨时钟 FIFO |
| `isp_ccl_bridge.v` | 在（65） | FIFO/坐标重建/CCL/丢拍恢复/结果 CDC 的桥接 |
| `isp_pop.v`、`red_ball_detect_96.v` | 在（62/63） | POP/颜色判据（默认关闭） |
| `isp_top.v` 的 `.EDGE_TH` | `11'd48` | 边缘阈值 |
| `isp_top.v` 的 `DET_MODE` | `1'b0` | 颜色无关圆形检测 |
| `isp_top.v` 的 `AWB_MODE` | `1'b0` | 灰度世界 AWB |
| 器件 | PH1P35MDG324 / -3 | **Flash**，不要选 Pro |
| `isp_bg_mask.v` | **不在**文件列表 | 其 `isp_fg_mask` 与 `isp_shape_circle.v` 重名且未例化，属死代码 |

时钟约束（`camera_to_dsi_display.sdc`）：
- CCL 使用 **50 MHz 系统时钟 /2 = 25 MHz**（`isp_top` 内 `ccl_div2` + BUFG），
  与 MIPI 字节时钟 **真正异步**；
- SDC 含 `create_generated_clock -name ccl_clk ... -divide_by 2` 及异步时钟组；
- **没有 false path，也没有放宽任何时钟**。

---

## 4. 当前代码的真实实验状态

### 4.1 历史（2026-10 之前，已解决）

提交 `dcfcacb` 首次接入 `isp_ccl_circle` 后，完整布线得到
SWNS = **−22.244 ns**、3426/13368 个 setup 违例端点，**当时不可上板**。
根因是 15 槽标签表在 103.52 MHz 单周期内完成“解析 + 合并 + 写回”，
关键路径 35 级、`keep` 扇出 461。当时的掩膜域仿真 8/8 通过，
但那只是功能层面，不代表时序或真实场景通过。

### 4.2 现在（本轮改动后的实测）

- CCL 移入**异步 25 MHz 域**（50 MHz/2 + BUFG），掩膜经 512×10 异步 FIFO
  （hit/tuser/tlast + err/resync 双标志）跨域；坐标在 CCL 域重建；
  事件用 3 级同步 + 边沿检测，结果数据保持握手。
- 丢拍恢复：坏帧后的 tuser 拍组合清 CCL 并暂存一拍，下一拍以 clean 状态送入；
  `frame_open` 保证无 tuser 的损坏段不产生事件。
- 被掩盖的 `edge_detect_96`、`isp_chroma_denoise` 路径各加一级流水。
- 本机 TD 6.2.1 完整 syn + phy（PH1P35MDG324/-3，slow/fast 角），
  2026-10-31 独立 scratch 构建（源码锁定后）：

| 项目 | 结果 |
|---|---|
| setup | **SWNS +0.091 ns，STNS 0，0 违例** |
| hold | **HWNS +0.020 ns，HTNS 0，0 违例** |
| STA 覆盖 | 96.26% |
| 资源 | LUT6 18,079 / 21,216（85.21%）；寄存器 14,314；slice 86.68%；ERAM 40/108；DSP 31/40 |
| 位流 | `camera_to_dsi_display.bit` SHA256 `e71a9adc…65af4efd` |

**结论**：时序上具备烧录条件，但 setup 余量只有 91 ps、hold 20 ps；
**本轮未烧录、未做摄像头实拍与上板验证**。位流在 scratch 工程内，
未覆盖原工程历史 bit。

### 4.3 仍未验证 / 不要声称

- 720p60 输入假设（VTS<1302）下的 FIFO 速率/深度证明；更高像素率模式需重算。
- 异步 CDC 由 2FF/3FF 设计与 `isp_ccl_bridge` 回归保证，不在同步 STA 中计时。
- 真实场景（光照、噪声、运动模糊、遮挡）未实测；仿真为掩膜域合成场景。

---

## 5. 已知限制（不因迁移而改变）

1. 前景掩膜 = 色度 ∪ 亮度梯度，整帧一个掩膜；背景本身大面积彩色/强纹理时，
   前景质量仍是更高层的瓶颈。
2. CCL 标签容量 15、每行 8 个游程；密集杂波会耗尽标签，`O_ovf=1` 降级。
   本轮不扩大容量；HUD flags[7] 明确显示溢出，目标被丢弃是已知边界。
3. 粗大相连遮挡（如整条手臂）会把球并成长条，aspect 判据拒绝；仅容忍细遮挡。
4. **硬边/高对比非白反光**（一步跳到 50% 白的过渡环）仍可能把球切碎；
   场景 27 保留该失败样例。平滑径向高光已能保持完整球（场景 33）。
5. 背景中若存在另一个**真圆**，形状判据无法判断哪个是目标；场景 29
   只证明"确实报出一个真圆"，不声称红球必胜。
6. 时序余量小（setup +0.091 ns / hold +0.020 ns），未做温度/电压实测与上板。
7. `isp_bg_mask.v` 的背景差分掩膜未接入（重复模块名），当前仍用
   `isp_shape_circle.v` 的判据。
8. HUD flags 判据位只在 pass 或候选时有效；无 pass/候选时只能可靠读出
   `live`/`ovf`，不能据此区分全部失败原因。

---

## 6. 回归测试（已入库）

目录 `sim/`：

| 测试 | 实例化 | 覆盖 |
|---|---|---|
| `tb/tb_bridge.v` | **工程实际使用的** `isp_ccl_bridge.v` + `isp_mask_fifo.v` + `isp_ccl_circle.v` | 跨域 FIFO、事件 3 级 CDC、err/resync、满恰逢 tuser、恢复帧、连续帧 |
| `tb/tb_ccl_regression.v` | `isp_ccl_circle.v`，与 `ref/isp_ccl_circle_ref.v` 逐拍对比 | 圆/遮挡/背景/无球/方块/卷尺/合并/空心环/溢出/行尾/跨行/密集流 |

运行：`cd sim && run.bat`（或见 `sim/README.md`）。测试台只驱动掩膜流/像素流，
CDC 与恢复逻辑来自实际 RTL，**不是手写镜像**。

---

## 7. 已知限制（与 §5 相同，保留旧编号便于对照）

见 §5。若需回退到“整帧单一 bbox”的旧逻辑，请注意当前结构已改为
`isp_ccl_bridge` 包装 `isp_ccl_circle`（带 `O_busy`），旧的
`isp_circle_detect` 不能直接替换；需同时调整桥接模块。
