onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/*
add wave -noupdate -group imem /tb_SimpleAdd/imem/*
add wave -noupdate -group dmem /tb_SimpleAdd/dmem/*
add wave -noupdate -expand -group proc /tb_SimpleAdd/proc/*
TreeUpdate [SetDefaultTree]
quietly wave cursor active 2
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
