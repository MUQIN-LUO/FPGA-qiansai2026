# lab_ex4_ext_SC520 —— 新设备构建与迁移说明

适用对象：在一台**新电脑**上 clone 本仓库后，要重新构建 `lab_ex4_ext_SC520` 的人。
工具：Anlogic TangDynasty（TD）6.2.1，器件 PH1P35MDG324 / speed -3（**Flash 模型**）。

---

## 1. 先说结论：什么可迁移，什么没验证

**可迁移（已验证）**
- `.al` 里 **63 个文件条目全部是相对路径**，相对于 `.al` 自己所在目录：
  - 61 条形如 `../user_source/hdl_source/...`
  - 2 条形如 `camera_to_dsi_display.sdc`、`al_ip/ChipWatcher_mipi/ChipWatcher_mipi.xml`
  - **没有任何一条是绝对路径**（`grep -E 'Path="(/|[A-Za-z]:)'` 无结果）。
  所以只要保持仓库目录结构，源文件、约束、IP 定义都能被正确引用。
- 约束与 IP 均已入库：`td_project/camera_to_dsi_display.sdc`、
  `user_source/constraints_source/pin.adc`、`user_source/ip_source/**`（19 个 xml/ipc）、
  `user_source/hdl_source/isp/demosaic_4x_2_0/**`（6 个 xml）。

**未验证（不要当成已解决）**
- `.al` 顶部的 `<Project ... Path="...">` 属性里是**某台机器上的绝对路径**。
  观察到的事实：TD 在 GUI 里跑过一次流程后，会把这个属性**改写成当前机器**
  的路径 —— 说明它由工具维护、更像一条"记录"而不是查找键。
  但**我没有证据证明**这个残留的旧绝对路径在新机器上一定被忽略。
  **不要把"它看起来不影响"当成已验证。**
- 本仓库**不包含**任何已构建产物（`*_Runs/` 已在 `.gitignore` 中排除）。

## 2. 新设备上的操作步骤

```
git clone <repo>
cd <repo>/lab_ex4_ext_SC520/td_project
```

**方式 A（GUI，推荐）**
1. TD → Open Project，选 `td_project/camera_to_dsi_display.al`。
2. 若提示工程路径无效/文件找不到：用 TD 的重新定位（Relocate / 重新指定工程目录）
   把工程目录指到 `lab_ex4_ext_SC520/td_project`。
3. 打开后逐项核对 §3 的清单。
4. 在 Design 流程里重建 Run（`syn_1` / `phy_1`）。**必须重建 Run**，
   不要复用别人机器上的 `_Runs/` 目录（本仓库也没有）。

**方式 B（命令行）**
1. 先按方式 A 打开并保存一次，让 TD 依据 `.al` 生成 `td_project/camera_to_dsi_display_Runs/{syn_1,phy_1}`。
   **这一步不能跳过**：CLI 读的是 Run 目录下的 `.prj`，而 `.prj` 是 TD 从 `.al`
   生成的；仓库里没有 `.prj`。
2. 分别在 `syn_1`、`phy_1` 目录下执行 `td_commands_prompt.exe <flow.tcl>`，
   其中 tcl 只需 `source <TD安装目录>/doc/scripts/DefaultFlow.tcl`。

## 3. 打开后必须核对的清单

| 检查项 | 期望值 | 为什么 |
|---|---|---|
| 文件数 | 63 个条目 | 少一个就会缺 design unit |
| `user_source/hdl_source/isp/isp_ccl_circle.v` 在列表里 | 在（CompileOrder 61） | 连通域圆形检测，缺它 `isp_top` 例化失败 |
| `isp_gray_world.v`、`isp_shape_circle.v` 在列表里 | 都在 | 灰度世界 AWB、前景掩膜 + 圆形判据 |
| `isp_top.v` 的 `.EDGE_TH` | `11'd48` | 显示边缘阈值（用户要求保持 48） |
| `isp_top.v` 的 `DET_MODE` | `1'b0` | 0 = 颜色无关圆形检测；1 = 旧红/黄颜色判据 |
| `isp_top.v` 的 `AWB_MODE` | `1'b0` | 0 = 灰度世界；1 = 白纸一键标定 |
| 器件 | PH1P35MDG324 / -3 | **Flash**，不要选 Pro |

## 4. 当前代码的真实实验状态（务必先读）

- **`isp_ccl_circle` 已接入，但时序未通过。** 最近一次完整布线结果：
  SWNS = **−22.244 ns**、STNS = −47.6 µs、**3426 / 13368 个 setup 违例端点**
  （hold 正常 +0.020 ns）。**该配置不可上板。**
- 功能上（掩膜域真实 RTL 对比，8/8 通过）：干净圆、球+细手指遮挡、圆+背景零散噪声、
  无目标、方块（给候选不通过）、两圆分离、长条 —— 连通域版本确实解决了
  "整帧单一 bbox 被背景撑爆"的问题。
- 若要退回上一版可用逻辑：把 `isp_top.v` 里 `u_circle_detect` 的
  `isp_ccl_circle` 换回 `isp_circle_detect`（接口等价，`O_chord` ↔ `O_span`），
  该版本曾布线通过（SWNS ≈ +0.004 ns、0 违例）。
- 详细结论与限制见仓库提交 `dcfcacb` 的提交说明，以及交付包里的各轮 README。

## 5. 已知限制（不因迁移而改变）

1. 前景掩膜 = 色度 ∪ 亮度梯度，整帧一个掩膜；连通域版缓解了 bbox 合并，
   但**没有解决**前景质量本身。
2. 粗大的相连遮挡（如一条 140×120 的臂）会把球并成长条，aspect 判据拒绝；
   目前只容忍细手指级别的局部遮挡。
3. 15 个标签 / 每行 8 个游程，杂乱场景可能耗尽（模块给出 `O_ovf` 降级标志）。
4. 仿真用的是存储器原语行为替身（`PH1_LOGIC_ERAM`），只覆盖功能，
   不构成硬件等价性证明。
