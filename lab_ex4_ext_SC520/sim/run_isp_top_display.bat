@echo off
rem ==========================================================================
rem Real isp_top display-path diagnostic test (ModelSim).
rem   Uses behavioral PH1P_LOGIC_ERAM/BUFG/GSR models (sim/tb/ph1p_sim_models.v)
rem   because the encrypted vendor sim models cannot bind PH1P_PHY_GSR here.
rem   NOTE: this TB currently does NOT pass cleanly under these models
rem   (tuser off valid + some line-start pixel differences). It is a diagnostic
rem   harness, not acceptance evidence. Board verification is required.
rem ==========================================================================
setlocal
cd /d "%~dp0"
if not exist logs mkdir logs
vlib lib_isp
set I=..\user_source\hdl_source\isp
set R=..\user_source\hdl_source
set D=%I%\demosaic_4x_2_0
vlog -quiet -work lib_isp tb\ph1p_sim_models.v ^
    %R%\awb.v %I%\awb\divider\divider_gate.v %I%\awb\signal_delay.v ^
    tb\blk_mem_gen_awb_delay_signal.v ^
    %I%\isp_gray_world.v %I%\isp_awb_arbiter.v %I%\edge_detect_y96.v ^
    %I%\edge_detect_96.v %I%\isp_gray_share96.v ^
    %I%\isp_shape_circle.v %I%\isp_mask_fifo.v %I%\isp_ccl_circle.v ^
    %I%\isp_ccl_bridge.v %I%\isp_pop96.v %I%\red_ball_detect_96.v ^
    %I%\raw10_unpacket_2lane.v ^
    %D%\demosaic.v %D%\bilinear_interpolation.v %D%\line_buffer_demosaic.v ^
    %D%\mipi_to_raw_converter.v %D%\raw_matrix_3x3_buffer.v %D%\zhenghe.v ^
    %D%\blk_mem_gen_demosaic\blk_mem_gen_demosaic.v ^
    %D%\blk_mem_gen_demosaic\RTL\ram_c36c2ca3b3a7.v ^
    %D%\blk_mem_gen_zhenghe\blk_mem_gen_zhenghe.v ^
    %D%\blk_mem_gen_zhenghe\RTL\ram_d32970204f32.v ^
    %I%\data128_96\data128_96.v %I%\data96_128\data96_128.v %I%\isp_top.v ^
    tb\tb_isp_top_display.v
vsim -c -quiet -L lib_isp lib_isp.tb_isp_top_display +FRAMES=4 ^
    -do "run -all; quit -f" -l logs\tb_isp_top_display.log
