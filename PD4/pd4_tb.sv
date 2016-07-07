// pipelined but no forwarding or stalling required
// expected output is found in test.out
module tb_test;
	logic clk;
	logic reset;

	parameter benchmark = "test.x";
	parameter mem_depth = 2**20;
	parameter [31:0] mem_start = 32'h80020000;

	// instruction memory
	logic im_rw = 1;
	logic [31:0] im_addr, im_dout;
	logic [1:0] im_sz = 0;
    memory #(.benchmark(benchmark), .depth(mem_depth)) imem(.clk(clk),
		.addr(im_addr), .data_out(im_dout),
		.access_size(im_sz), .rd_wr(im_rw), .enable(~reset));

	// data memory
	logic dm_rw;
	logic [31:0] dm_addr, dm_din, dm_dout;
	logic [1:0] dm_sz = 0;
    memory #(.benchmark(benchmark), .depth(mem_depth)) dmem(.clk(clk),
		.addr(dm_addr), .data_in(dm_din), .data_out(dm_dout),
		.access_size(dm_sz), .rd_wr(dm_rw), .enable(~reset));

	mips #(.pc_init(mem_start), .sp_init(mem_start+mem_depth), .ra_init(0))
		proc(.clk(clk), .reset(reset),
		.instr_addr(im_addr), .instr_in(im_dout),
		.data_addr(dm_addr), .data_in(dm_dout), .data_out(dm_din),
		.data_rd_wr(dm_rw));
 
    initial begin
        clk = 1; forever #5 clk = ~clk;
    end

    initial begin
        reset <= 1;
		#10 reset <= 0;

		wait(im_addr==32'h80020018)
   				$display("time %3d, v0 %0d", $time, tb_test.proc.regs.data[2]);
		wait(im_addr==32'h8002001c)
   				$display("time %3d, s0 %0d", $time, tb_test.proc.regs.data[16]);
		wait(im_addr==32'h80020020)
   				$display("time %3d, s1 %0d", $time, tb_test.proc.regs.data[17]);
		wait(im_addr==32'h80020024)
   				$display("time %3d, s2 %0d", $time, tb_test.proc.regs.data[18]);
		wait(im_addr==32'h8002002c)
   				$display("time %3d, v1 %0d", $time, tb_test.proc.regs.data[3]);
		wait(im_addr==32'h8002003c)
   				$display("time %3d, v0 %0d", $time, tb_test.proc.regs.data[2]);
   		$monitor("time %3d, pc %8h", $time, im_addr);

		wait(im_addr==0)
			#1 $finish; //$finish if using iverilog
    end

    initial $dumpvars(0, tb_test); // for iverilog+gtkwave

endmodule
