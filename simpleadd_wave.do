onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/clk
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/read
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/busy
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/en
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/addr
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/din
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/dout
add wave -noupdate -expand -group tb_SimpleAdd /tb_SimpleAdd/size
add wave -noupdate /tb_SimpleAdd/mem/clk
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/addr
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/data_in
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/access_size
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/rd_wr
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/enable
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/data_out
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/busy
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/words_total
add wave -noupdate -expand -group Memory /tb_SimpleAdd/mem/words_read
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {35 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 194
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
WaveRestoreZoom {0 ns} {68 ns}
