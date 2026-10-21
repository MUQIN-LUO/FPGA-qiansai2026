import_device ph1_35p.db -package PH1P35MDG324
set_param flow ooc_flow on
read_verilog -file "ChipWatcher_aef86b9bb9fc_watcherInst.sv"
optimize_rtl
map_macro
map
pack
report_area -file ChipWatcher_aef86b9bb9fc_gate.area
export_db -mode ooc "ChipWatcher_aef86b9bb9fc_ooc.db"
