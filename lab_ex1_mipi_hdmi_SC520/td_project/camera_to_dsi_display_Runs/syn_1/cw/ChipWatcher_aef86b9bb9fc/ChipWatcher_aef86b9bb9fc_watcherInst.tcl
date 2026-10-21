source "E:/Anlogic/TD_6.2.1_Engineer_6.2.168.116/cw/atpl/templa.tcl"
set fd [open "E:/Anlogic/TD_6.2.1_Engineer_6.2.168.116/cw/atpl/cwc_ip.atpl" r]
set tmpl [read $fd]
close $fd
set parser [::tmpl_parser::tmpl_parser $tmpl]

set ComponentName        ChipWatcher_aef86b9bb9fc
set bus_num              9
set depth                1024
set ram_len              42
set input_pipe_num       0
set output_pipe_num      0
set capture_ctrl_exist   0
set trig_bus_num         9
set trig_bus_din_num     42
set trig_bus_ctrl_len    144
set trig_ctrl_len        164
set trig_bus_width       { 1,1,1,1,11,9,1,1,16 };
set trig_bus_din_pos     { 0,1,2,3,4,15,24,25,26 };
set trig_bus_ctrl_pos    { 0,4,8,12,16,53,84,88,92 };
set bus_size             {  16 1 1 9 11 1 1 1 1 }
set data_enable          { probe0 probe1 probe2 probe3 probe4 probe5 probe6 probe7 probe8 }
set trig_enable          { probe0 probe1 probe2 probe3 probe4 probe5 probe6 probe7 probe8 }
set fp [open "cw/ChipWatcher_aef86b9bb9fc/ChipWatcher_aef86b9bb9fc_watcherInst.sv" w+]
puts $fp [eval $parser]
close $fp
