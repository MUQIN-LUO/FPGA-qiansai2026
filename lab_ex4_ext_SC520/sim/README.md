# lab_ex4_ext_SC520 回归测试

两个回归都直接实例化**工程实际使用的 RTL**，不是测试台内的手写镜像：

| 测试 | 实例化的 RTL | 覆盖 |
|---|---|---|
| `tb/tb_bridge.v` | `isp_ccl_bridge.v` + `isp_mask_fifo.v` + `isp_ccl_circle.v` | 掩膜 FIFO 跨域、坐标重建、事件 3 级 CDC、丢拍双标志（err/resync）、tuser 暂存无丢拍恢复、`frame_open` 帧门控 |
| `tb/tb_ccl_regression.v` | `isp_ccl_circle.v`（与 `ref/isp_ccl_circle_ref.v` 逐拍对比） | 干净圆、细遮挡、复杂背景、无球、方块、卷尺、相邻/合并圆、空心环、标签溢出、行尾合并、最后一拍关闭、跨行修复、无空拍密集流、连续帧 |

## 运行

ModelSim（`vlib`/`vlog`/`vsim` 在 PATH 中）：

```bat
cd lab_ex4_ext_SC520\sim
run.bat
```

或手动：

```
vlib work
vlog ..\user_source\hdl_source\isp\isp_ccl_circle.v ref\isp_ccl_circle_ref.v tb\tb_ccl_regression.v
vsim -c tb_ccl_regression -do "run -all; quit -f"

vlog ..\user_source\hdl_source\isp\isp_ccl_bridge.v ..\user_source\hdl_source\isp\isp_mask_fifo.v ..\user_source\hdl_source\isp\isp_ccl_circle.v tb\tb_bridge.v
vsim -c tb_bridge -do "run -all; quit -f"
```

## 说明与边界

- `ref/isp_ccl_circle_ref.v` 是 HEAD `2762152` 版本的 golden（仅含溢出哨兵修复），
  用于逐字段对比；桥接模块的测试**不需要**这个副本。
- 桥接测试的写侧为 103.52 MHz、CCL 侧 25 MHz（50 MHz/2），一行 2300 拍；
  场景包含两帧干净圆、连续帧、帧内强制溢出、满恰逢 tuser 的恢复。
- 仿真为掩膜域合成场景，不含摄像头光学、噪声与实拍条件。
