# if using Makefile for iverilog:
# - change $stop to $finish is testbenches
# - uncomment "initial $dumpvars(0, tb_name);" to get waveforms
source=pd4_tb.sv main_memory.sv regfile.sv mips.sv
iFlags=-g2012

.PHONY: run
run: Test

# tb_test
.PHONY: Test
Test: tb_test.vvp
	vvp $<

.PHONY: Test-debug
Test-debug: tb_test.lx2
	gtkwave -A $< &

tb_test.vvp: $(source) test.x
	iverilog $(iFlags) -s tb_test -o $@ $(source)

%.lx2: %.vvp
	vvp $< -lxt2
	mv dump.lx2 $@

clean:
	rm -f *.vvp *.lx2 *.vcd
