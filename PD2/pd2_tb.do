# Create work library
vlib work

# Compile source files
vlog -work work *.sv

# Select testbench
vsim tb_SimpleAdd

# Show signals in wave window
do pd2_wave.do

# Open wave window
view wave

# Run for 10000 ns
run 100000

# Zoom wave
wave zoom full
