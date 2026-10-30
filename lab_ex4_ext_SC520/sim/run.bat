@echo off
rem Run the CCL / bridge regressions with ModelSim (vlib/vlog/vsim must be on PATH).
cd /d "%~dp0"
vlib work
vlog -quiet ..\user_source\hdl_source\isp\isp_ccl_circle.v ref\isp_ccl_circle_ref.v tb\tb_ccl_regression.v
vsim -c tb_ccl_regression -do "run -all; quit -f"
vlog -quiet ..\user_source\hdl_source\isp\isp_ccl_bridge.v ..\user_source\hdl_source\isp\isp_mask_fifo.v ..\user_source\hdl_source\isp\isp_ccl_circle.v tb\tb_bridge.v
vsim -c tb_bridge -do "run -all; quit -f"
