module mips(input clk, reset,
					input logic [31:0] instr_in, data_in,
					output logic [31:0] instr_addr, data_addr, data_out, 
					output logic data_rd_wr);
	parameter [31:0] pc_init = 32'h80020000;
    parameter [31:0] sp_init = 32'h80120000; 
	parameter [31:0] ra_init = 32'h00000000;
	
	logic 		 alu_en,
				 sign_extend_en,
				 EX_reg_wr_en,
				 EX_wb_en,
				 EX_mem_en,
				 data_rw_en,
				 ME_wb_en,
				 ME_data_rw_en,
				 ME_reg_wr_en,
				 ME_mem_en,
				 WB_wb_en,
				 WB_reg_wr_en;			 
	
	
			   
	
	logic [4:0]	 EX_wr_sel,
				 ME_wr_sel,
				 WB_wr_sel,
				 rd_sel,
				 rs_sel,
				 alu_op;
	
	logic [31:0] pc; 			// program counter
	logic [31:0] /*IF_IS_IMPLIED*/ ID_instr_reg, EX_instr_reg, ME_instr_reg, WB_instr_reg;
	//Instruction propagation	
	logic [31:0] rs, 			// register output 2
				 rd, 			// destination register
				 alu_out,		// alu output reg
				 EX_wr_reg,
				 ME_wr_reg,
				 WB_wr_reg_data;		// write to register register
	 
	regfile regs(.wr_num(WB_wr_sel), .wr_data(WB_wr_reg_data), .wr_en(WB_reg_wr_en),
		.rd0_num(rd_sel), .rd0_data(rd),
		.rd1_num(rs_sel), .rd1_data(rs),
		.clk(clk));
	
	logic IF_en,ID_en, EX_en, ME_en, WB_en;
	
	initial begin
		IF_en = 1'b0;
		ID_en = 1'b0;
		EX_en = 1'b0;
		ME_en = 1'b0;
		WB_en = 1'b0;
		
		data_addr 	<= sp_init;
		pc 			<= pc_init;
		instr_addr  <= pc_init;
		data_rw_en   <= 1'b1;
		data_rd_wr	<= 1'b1;
		alu_en		<= 1'b0;
		EX_wb_en	<= 1'b0;
		ME_wb_en	<= 1'b0;
		WB_wb_en    <= 1'b0;
		EX_mem_en 	<= 1'b0;
		ME_mem_en 	<= 1'b0;
		alu_out		<= 32'h0;
		alu_op 		<= 5'b0;
		
		WB_wr_sel 		<= 5'h1D; //sp_init = R29
		WB_wr_reg_data		<= sp_init;
		WB_reg_wr_en    <= 1'b1;
		ME_reg_wr_en	<= 1'b0;
		EX_reg_wr_en	<= 1'b0;
		
		//#10 wr_sel 		<= 5'h1F; //ra_init = R31
		//wr_reg		<= ra_init;
		/*#90 pc <= 32'h80020018;
		#10	pc <= 32'h8002001c;
		#10	pc <= 32'h80020020;
		#10	pc <= 32'h80020024;
		#20	pc <= 32'h8002002c;
		#40	pc <= 32'h8002003c;
		#30 pc <= 32'h0;*/
	end
	
	always @ (reset) begin
	    if(reset) begin
	    	IF_en <= 1'b0;
			ID_en <= 1'b0;
			EX_en <= 1'b0;
			ME_en <= 1'b0;
			WB_en <= 1'b0;
			WB_reg_wr_en    <= 1'b1;

		end else begin
			IF_en <= 1'b1;
			WB_wr_sel 		<= 5'h1D; //sp_init = R29
			WB_wr_reg_data		<= sp_init;
			WB_reg_wr_en    <= 1'b1;

		end
	end
	
	always @ (pc)
	begin
		if(~reset) begin
			instr_addr <= pc;
			IF_en = 1'b1;
		end
	end
	
	always @ (posedge clk)
	begin : IF
		if(IF_en) begin
			ID_instr_reg <= instr_in;
			ID_en <= 1'b1;
		end else begin
			WB_wr_sel <= 5'h1F; //ra_init = R31
			WB_wr_reg_data <= ra_init;
			WB_reg_wr_en <= 1'b1;
		end
	end //IF
	
	always @ (posedge clk) 
	begin: ID
		if(ID_en) begin
		//decode current_instr to instruction (+ immediate)
		//instructions we will need: 	
		//addiu addu jr li lw move nop sw
		//pages refer to pdf page # in MIPS ISA
		EX_instr_reg <= ID_instr_reg;
		case(ID_instr_reg[31:26])
			6'b001001: begin
				//ADDIU rt, rs, immediate
				//add immediate unsigned (pg 47)
				//[31:26]: 001001
				// [0010 01][11 101][1 1101]
				EX_wr_sel <= ID_instr_reg[20:16]; //store result here (rt)
				rs_sel <= ID_instr_reg[25:21]; //operand 1, rs 
				sign_extend_en <= 1'b1; //operand 2, immediate, sets alu_2
				alu_en <= 1'b1;
				alu_op <= 5'b00001;
				EX_wb_en <= 1'b1;
				EX_mem_en <= 1'b0;
				alu_op <= 5'b00001;
			end
			6'b100011: begin
				//LW rt, offset(base)
				//load word from memory (pg 171) 
				//[31:26]: 100011
				EX_wr_sel <= ID_instr_reg[20:16]; // store from mem to this reg (rt)
				rs_sel = ID_instr_reg[25:21]; // base addr
				sign_extend_en <= 1'b1; //sets alu_2 with offset
				alu_en <= 1'b1;
				alu_op <= 5'b00001;
				EX_wb_en <= 1'b1;
				EX_mem_en <= 1'b1;
			end
			//NOTE: LW and SW seem similar so far - change to common stage? only diff is 10(0/1)011
			6'b101011: begin
				//SW, rt, offset(base)
				//store word from rt to memory[base+offset] (pg 280)
			    //address error exception if last two bits != 00
				//[31:26]: 101011
				EX_wr_sel <= ID_instr_reg[20:16]; //writeback setup
				rs_sel <= ID_instr_reg[25:21]; //base address
				rd_sel <= ID_instr_reg[20:16]; //source word
				sign_extend_en <= 1'b1; //sets alu_2 with immediate
				alu_en <= 1'b1;
				alu_op <= 5'b00001;
				EX_wb_en <= 1'b0;
				EX_mem_en <= 1'b1;
			end
			6'b000100 : begin
				//BEQ
				//  rs [25:21] rt [20:16] offset [15:0]
				//I: target_offset ← sign_extend(offset || 02)
				//	condition ← (GPR[rs] = GPR[rt])
				//I+1: if condition then
				//		PC ← PC + target_offset
				//	 endif
				rs_sel = ID_instr_reg[25:21];
				rd_sel = ID_instr_reg[20:16];
				if (rs == rd) begin
					alu_en = 1'b1;
					sign_extend_en = 1'b1;
					alu_op <= 5'b10000;

						if(EX_instr_reg[15]) begin //Sign Extend should always be '1' don't explicitally check
							// This is calculating +4 too many
							pc <= pc + {12'hfff, 2'b11, ID_instr_reg[15:0], 2'b00};
						end else begin
							// This is calculating +4 too many
							pc <= pc + {12'h000, 2'b00, ID_instr_reg[15:0], 2'b00};
						end 
						
				end else begin
					alu_en = 1'b0;
					sign_extend_en = 1'b0;
					alu_op <= 5'b00000;
				end
				EX_wb_en <= 1'b0;
				EX_mem_en <= 1'b0;
			end
			6'b000000 : begin  //SPECIAL
				case(instr_in[5:0]) 
					6'b100011 : begin //SUBU
						//SUBU sub unsigned
						// 000000 rs [25:21] rt [20:16] rd [15:11] 00000 100011 (SUBU)
						// rd <- rs - rt
						rs_sel <= ID_instr_reg[25:21]; //op2
						rd_sel <= ID_instr_reg[20:16]; //op1
						EX_wr_sel <= ID_instr_reg[15:11]; //destination
						alu_en <= 1'b1;
					    alu_op <= 5'b00010;
						sign_extend_en <= 1'b0;
						EX_wb_en <= 1'b1;
						EX_mem_en <= 1'b0;
					end
					6'b001000 : begin
						//JR rs
						//jump register, set PC to rs (pg 155) 
						//[31:26]: 000000 & [5:0]: 0010000
						rs_sel = ID_instr_reg[25:21];
						pc = rs;
						alu_en <= 1'b0;
						alu_op <= 5'b00000;
						EX_wb_en <= 1'b0;
						EX_mem_en <= 1'b0;
					end
					6'b000000 : begin
						//NOP
						//not an op, actually SLL r0, r0, 0 (pg 226) do we need to actually implement this way?
						//[31:26]: 000000 & SLL [5:0]=000000
						//r0 <= r0 << 1'b0;
						//SLL
						// 000000 00000 rt [20:16] rd [15:11] sa [10:6] 000000
						//rd <- rt << sa
						EX_wr_sel = ID_instr_reg[15:11];
						rs_sel <= ID_instr_reg[20:16];
						alu_en <= 1'b0;
						alu_op <= 5'b01000;
						EX_mem_en <= 1'b0;
						EX_wb_en <= 1'b0;
					end
				endcase
			end
		endcase
		EX_en <= 1'b1;
		end
	end // ID
	
	always @ (posedge clk)
	begin: EX
		if(EX_en) begin
			ME_wb_en <= EX_wb_en;
			ME_mem_en <= EX_mem_en;
			ME_reg_wr_en <= EX_reg_wr_en;
			ME_wr_sel <= EX_instr_reg[20:16];
			ME_instr_reg <= EX_instr_reg;
			ME_en <= 1'b1;
			//execute arithmetic and instruction, eg. address = base + offset
			//run if alu_en is set
			//adds alu_1 and alu_2
			//output to alu_out
			if(alu_en == 1'b1) begin
				case(alu_op)
					5'b01000: begin //SLL
						alu_out <= rs << EX_instr_reg[10:6];
						ME_wr_reg <= rs << EX_instr_reg[10:6];
					end
					5'b00010: begin //SUBU
						alu_out <= rs - rd;
						ME_wr_reg <= rs - rd;
					end
					5'b00001: begin //ADDIU
						if(sign_extend_en) begin
							if(EX_instr_reg[15]) begin
								alu_out  = rs + { 16'hffff, EX_instr_reg[15:0]};
								data_out = rd;
								if (~EX_mem_en) begin
									ME_wr_reg <= rs + { 16'hffff, EX_instr_reg[15:0]};
								end else begin
									ME_wr_reg <= rd;
								end
							end else begin
								alu_out = rs + { 16'h0000, EX_instr_reg[15:0]};
								data_out = rd;
								if (~EX_mem_en) begin
									ME_wr_reg <= rs + { 16'h0000, EX_instr_reg[15:0]};
								end else begin
									ME_wr_reg <= rd;
								end
							end
							if (EX_mem_en) begin
								if(EX_instr_reg[29]) begin //TODO: Add write flag to avoid holding this reg
									data_addr = alu_out;
									data_rd_wr <= 1'b0;
								end else begin
									data_addr = alu_out;
									data_rd_wr <= 1'b1;
								end
							end else begin
								data_rd_wr <= 1'b1;
							end
						end else begin
							alu_out <= rs + rd;
							ME_wr_reg <= rs +rd;
						end
					end
				endcase
			end
			if(~(ID_instr_reg[31:26] == 6'b000100)) begin
				// branch delay
				pc <= pc + 4;//increment PC after done with instr_in
			end 
		end
	end //EX
		
	always @ (posedge clk)
	begin: ME
		if(ME_en) begin
		 	WB_instr_reg <= ME_instr_reg;
		 	WB_wb_en <= ME_wb_en;
		 	WB_wr_sel <= ME_wr_sel;
		 	if(ME_mem_en) begin
				if(ME_instr_reg[29]) begin //TODO: Add write flag to avoid holding this reg
					WB_wb_en <= 1'b1;
					WB_wr_reg_data <= ME_wr_reg;
				end else begin
					WB_wb_en <= 1'b0;
					WB_wr_reg_data = data_in;
				end
			end else begin
				WB_wr_reg_data <= ME_wr_reg;
				WB_en <= 1'b1;
			end
		end
	end // ME

	always @ (posedge clk)
	begin: WB
		if(WB_en) begin
			if(WB_wb_en) begin
				WB_reg_wr_en <= 1'b1;
			end else begin
				WB_reg_wr_en <= 1'b0;
			end
		end
	end // WB

	
endmodule