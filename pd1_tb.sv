/* tb_SimpleAdd
* Loads SimpleAdd.x and tests reads and writes of single words.
*/
module tb_SimpleAdd;
	logic clk, read, busy, en;
	logic [31:0] addr, din, dout;
	logic [1:0] size;

    memory #(.benchmark("SimpleAdd.x"), .depth(2**20)) mem(.clk(clk),
		.addr(addr), .data_in(din), .data_out(dout),
		.access_size(size), .rd_wr(read), .busy(busy), .enable(en));

    initial $monitor("time %3d, addr %8h, data %8h, en %1b", $time, addr, dout,
		en);
 
    initial begin
        clk = 1; forever #5 clk = ~clk;
    end

    initial begin
        addr <= 32'h80020000; size <= 0; read <= 1; en <= 0;
		#10 en <= 1;
		#10 addr <= addr + 4;
		#10 read <= 0; din <= 32'haaaaeeee;
		#10 read <= 1;
		#20 $stop;
    end

endmodule


/* tb_BubbleSort
* Loads BubbleSort.x and tests reads of multiple words.
*/
module tb_BubbleSort;
	logic clk, read, busy, en;
	logic [31:0] addr, din, dout;
	logic [1:0] size;

    memory #(.benchmark("BubbleSort.x"), .depth(2**20)) mem(.clk(clk),
		.addr(addr), .data_in(din), .data_out(dout),
		.access_size(size), .rd_wr(read), .busy(busy), .enable(en));

    initial $monitor("time %3d, addr %8h, data %8h, busy %1b", $time, addr,
		dout, busy);
 
    initial begin
        clk = 1; forever #5 clk = ~clk;
    end

    initial begin
        addr <= 32'h80020000; size <= 0; read <= 1; en <= 0;
		#10 en <= 1;
		#10 size <= 1;
		#40 size <= 2;
		#80 size <= 3;
		#160
		#10 $stop;
    end

endmodule
