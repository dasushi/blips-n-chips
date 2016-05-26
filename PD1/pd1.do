# Create work library
vlib work

# Compile source files
vlog -work work pd1_tb.sv
vlog -work work memory.sv

# Select testbench
vsim tb_SimpleAdd

# Show signals in wave window
do simpleadd_wave.do

# Open wave window
view wave

# Run for 1000 ns
run 1000

# Zoom wave
wave zoom full
