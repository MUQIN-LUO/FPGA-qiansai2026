import_device ph1_35p.db -package PH1P35MDG324 -speed 3
set rc [catch {open_project camera_to_dsi_display.prj} msg]
puts "OPEN_RC=$rc MSG=$msg"
set rc [catch {elaborate -top design_top_wrapper} msg]
puts "ELAB_RC=$rc MSG=$msg"

set pm [string trim [get_pins *I_cal_mode*]]
set nm [string trim [get_nets -of [get_pins *I_cal_mode*]]]
set pt [string trim [get_pins *I_cal_trig*]]
set nt [string trim [get_nets -of [get_pins *I_cal_trig*]]]
puts "CAL_MODE_PIN=$pm"
puts "CAL_MODE_NET=$nm"
puts "CAL_TRIG_PIN=$pt"
puts "CAL_TRIG_NET=$nt"
if {$nm eq "S_cal_t2"} {
    puts "CAL_MODE_CDC=PASS"
} else {
    puts "CAL_MODE_CDC=FAIL expected S_cal_t2"
}
if {$nt eq "S_cal_trig"} {
    puts "CAL_TRIG_NET_CHECK=PASS"
} else {
    puts "CAL_TRIG_NET_CHECK=FAIL expected S_cal_trig"
}
# S_cal_tgl 的负载应只在其源域/同步器输入，不应直接进 isp_top
catch {set lg [get_pins -of [get_nets S_cal_tgl]]} m
puts "CAL_TGL_LOADS=$m"
exit
