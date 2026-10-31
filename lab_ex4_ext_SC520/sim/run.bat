@echo off
rem ==========================================================================
rem lab_ex4_ext_SC520 full regression (ModelSim 10.3c; vlib/vlog/vsim on PATH)
rem   All testbenches instantiate the same RTL used by the TD build.
rem   Long test: tb_frontend (~15 min at 1280x720 / real line timing).
rem ==========================================================================
setlocal
cd /d "%~dp0"
if not exist logs mkdir logs
vlib work

set I=..\user_source\hdl_source\isp
set R=..\user_source\hdl_source

echo === tb_pop ===
vlog -quiet %I%\isp_pop96.v tb\tb_pop.v
vsim -c -quiet tb_pop -do "run -all; quit -f" -l logs\tb_pop.log

echo === tb_awb_apply ===
vlog -quiet %R%\awb.v %I%\awb\divider\divider_gate.v %I%\isp_gray_world.v ^
    %I%\isp_awb_arbiter.v %I%\awb\signal_delay.v ^
    tb\blk_mem_gen_awb_delay_signal.v tb\tb_awb_apply.v
vsim -c -quiet tb_awb_apply -do "run -all; quit -f" -l logs\tb_awb_apply.log

echo === tb_cal_mode_cdc ===
vlog -quiet %R%\design_top_wrapper.v tb\tb_cal_mode_cdc.v
vsim -c -quiet tb_cal_mode_cdc -do "run -all; quit -f" -l logs\tb_cal_mode_cdc.log

echo === tb_ccl_regression ===
vlog -quiet %I%\isp_ccl_circle.v ref\isp_ccl_circle_ref.v tb\tb_ccl_regression.v
vsim -c -quiet tb_ccl_regression -do "run -all; quit -f" -l logs\tb_ccl_regression.log

echo === tb_bridge ===
vlog -quiet %I%\isp_ccl_bridge.v %I%\isp_mask_fifo.v %I%\isp_ccl_circle.v tb\tb_bridge.v
vsim -c -quiet tb_bridge -do "run -all; quit -f" -l logs\tb_bridge.log

echo === tb_ccl_color ===
vlog -quiet %I%\isp_ccl_circle.v tb\tb_ccl_color.v
vsim -c -quiet tb_ccl_color -do "run -all; quit -f" -l logs\tb_ccl_color.log

echo === tb_ae_ctrl ===
vlog -quiet %I%\..\uiisp_beta\ae_set.v tb\tb_ae_ctrl.v
vsim -c -quiet tb_ae_ctrl -do "run -all; quit -f" -l logs\tb_ae_ctrl.log

echo === tb_ae_i2c (controlled I2C slave model) ===
set U=%R%\uics520_cfg
vlog -quiet %U%\uicfgcs520.v %R%\uics500_cfg\uii2c.v %U%\uics520reg_720p60.v ^
    %U%\SC520GainTbl.v %U%\uics520regAE.v tb\tb_ae_i2c.v
vsim -c -quiet tb_ae_i2c -do "run -all; quit -f" -l logs\tb_ae_i2c.log

echo === tb_det_result (pass -> candidate -> empty / target switch) ===
vlog -quiet %I%\isp_top.v tb\tb_det_result.v
vsim -c -quiet tb_det_result -do "run -all; quit -f" -l logs\tb_det_result.log

echo === tb_det_frontend_awb (early tuser / circle / FIFO recovery) ===
vlog -quiet %R%\awb.v %I%\awb\divider\divider_gate.v %I%\awb\signal_delay.v ^
    tb\blk_mem_gen_awb_delay_signal.v %I%\edge_detect_y96.v %I%\edge_detect_96.v ^
    %I%\isp_shape_circle.v %I%\isp_mask_fifo.v %I%\isp_ccl_circle.v ^
    %I%\isp_ccl_bridge.v tb\tb_det_frontend_awb.v
vsim -c -quiet tb_det_frontend_awb -do "run -all; quit -f" -l logs\tb_det_frontend_awb.log

echo === tb_frontend (real line burst / blanking) ===
vlog -quiet %I%\isp_ccl_circle.v %I%\isp_mask_fifo.v %I%\isp_ccl_bridge.v ^
    %I%\isp_shape_circle.v %I%\edge_detect_y96.v %I%\edge_detect_96.v tb\tb_frontend.v
vsim -c -quiet tb_frontend -do "run -all; quit -f" -l logs\tb_frontend.log

echo === tb_gray_share (shared Y: gray/EDGE/POP/detection stream) ===
vlog -quiet %I%\isp_gray_share96.v %I%\edge_detect_y96.v %I%\edge_detect_96.v ^
    %I%\isp_pop96.v tb\tb_gray_share.v
vsim -c -quiet tb_gray_share -do "run -all; quit -f" -l logs\tb_gray_share.log

echo.
echo ============================================
echo ALL REGRESSIONS FINISHED. Check logs\*.log for RESULT lines.
echo ============================================
