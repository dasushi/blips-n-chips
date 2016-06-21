# if using Makefile for iverilog:
# - change $stop to $finish is testbenches
# - uncomment "initial $dumpvars(0, tb_name);" to get waveforms
source=pd3_tb.sv memory.sv regfile.sv mips.sv
iFlags=-g2005-sv

.PHONY: run
run: BubbleSort fact SimpleIf SumArray Swap

# tb_BubbleSort
.PHONY: BubbleSort
BubbleSort: tb_BubbleSort.vvp
	vvp $<

.PHONY: BubbleSort-debug
BubbleSort-debug: tb_BubbleSort.lx2
	gtkwave -A $< &

tb_BubbleSort.vvp: $(source) BubbleSort.x
	iverilog $(iFlags) -s tb_BubbleSort -o $@ $(source)

# tb_fact
.PHONY: fact
fact: tb_fact.vvp
	vvp $<

.PHONY: fact-debug
fact-debug: tb_fact.lx2
	gtkwave -A $< &

tb_fact.vvp: $(source) fact.x
	iverilog $(iFlags) -s tb_fact -o $@ $(source)

# tb_SimpleIf
.PHONY: SimpleIf
SimpleIf: tb_SimpleIf.vvp
	vvp $<

.PHONY: SimpleIf-debug
SimpleIf-debug: tb_SimpleIf.lx2
	gtkwave -A $< &

tb_SimpleIf.vvp: $(source) SimpleIf.x
	iverilog $(iFlags) -s tb_SimpleIf -o $@ $(source)

# tb_SumArray
.PHONY: SumArray
SumArray: tb_SumArray.vvp
	vvp $<

.PHONY: SumArray-debug
SumArray-debug: tb_SumArray.lx2
	gtkwave -A $< &

tb_SumArray.vvp: $(source) SumArray.x
	iverilog $(iFlags) -s tb_SumArray -o $@ $(source)

# tb_Swap
.PHONY: Swap
Swap: tb_Swap.vvp
	vvp $<

.PHONY: Swap-debug
Swap-debug: tb_Swap.lx2
	gtkwave -A $< &

tb_Swap.vvp: $(source) Swap.x
	iverilog $(iFlags) -s tb_Swap -o $@ $(source)

%.lx2: %.vvp
	vvp $< -lxt2
	mv dump.lx2 $@

clean:
	rm -f *.vvp *.lx2 *.vcd
