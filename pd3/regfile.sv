module regfile(
		input [4:0] wr_num, input [31:0] wr_data, input wr_en,
		input [4:0] rd0_num, output [31:0] rd0_data,
		input [4:0] rd1_num, output [31:0] rd1_data,
		input clk);

    reg[31:0] data [1:31]; // general-purpose registers r1-r31 (r0===0)

	always @(posedge clk)
        if(wr_en && wr_num!=0) data[wr_num] <= wr_data;
   	assign rd0_data = (rd0_num!=0) ? data[rd0_num] : 32'h00000000;
   	assign rd1_data = (rd1_num!=0) ? data[rd1_num] : 32'h00000000;

endmodule
