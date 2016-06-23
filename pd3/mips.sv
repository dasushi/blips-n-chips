module mips(input clk, reset,
					input logic [31:0] instr_in, data_in,
					output logic [31:0] instr_addr, data_addr, data_out, 
					output logic data_rd_wr); //1=read
	parameter [31:0] pc_init = 32'h80020000;
    parameter [31:0] sp_init = 32'h80120000; 
	parameter [31:0] ra_init = 32'h00000000;
    
	logic 		 reg_wr_en, alu_en, wb_en, mem_en, data_rw_en, sign_extend_en, link_en;
	logic [4:0]  instr_counter,	//Rolling bit counter to detect which stage the instruction is in
				 wr_sel, rd_sel, rs_sel;
	logic [4:0]	 alu_op;
	logic [31:0] pc; 			// program counter
	logic [31:0] instr_reg, 	// current instruction register
				 //mem_reg, 		// memory data register (mdr/mbr)
				 //rt, 			// register output 1
				 rs, 			// register output 2
				 rd, 			// destination register
				 //alu_1, 		// alu input 1
				 //alu_2, 		// alu input 2
				 alu_out,		// alu output reg
				 wr_reg;		// write to register register
	 
	regfile regs(.wr_num(wr_sel), .wr_data(wr_reg), .wr_en(reg_wr_en),
		.rd0_num(rd_sel), .rd0_data(rd),
		.rd1_num(rs_sel), .rd1_data(rs),
		.clk(clk));
	
	initial begin
		//initialize
		data_addr 	<= sp_init;
		pc 			<= pc_init;
		instr_addr  <= pc_init;
		data_rw_en   = 1'b1;
		data_rd_wr	<= data_rw_en;
		alu_en		<= 1'b0;
		wb_en		<= 1'b0;
		mem_en 		<= 1'b0;
		alu_op 		<= 5'b0;
		instr_counter <= 5'b00001;
		instr_reg	<= 32'h0;
		//mem_reg	<= 32'h0;
		//alu_1		<= 32'h0;
		//alu_2 		<= 32'h0;
		alu_out		<= 32'h0;
		
		wr_sel 		<= 5'h1D; //sp_init = R29
		wr_reg		<= sp_init;
		reg_wr_en	<= 1'b1;
		#10 wr_sel 		<= 5'h1F; //ra_init = R31
		wr_reg		<= ra_init;
		
		sign_extend_en = 1'b0;
	end
	
	always @ (pc)
	begin
		instr_addr <= pc;	
	end
	
	always @ (data_rw_en) begin
		data_rd_wr	= data_rw_en;
	end
	
	//main task
	always @ (posedge clk)
	begin : STAGES
	
		// Determine which stage
		case(instr_counter)
			5'b00001 : begin //IF
				instr_counter <= instr_counter << 1;
				reg_wr_en	<= 1'b0;
				data_rw_en  <= 1'b1;
				alu_en		<= 1'b0;
				wb_en		<= 1'b0;
				alu_op 		<= 5'b0;
				link_en 	<= 1'b0;
				mem_en 		<= 1'b0;
				wr_sel 		<= 4'h0;
				rd_sel 		<= 4'h0;
				rs_sel 		<= 4'h0;
				wr_reg		<= 32'h0;
				sign_extend_en = 1'b0;
			end
			5'b00010 : begin //ID
				//decode current_instr to instruction (+ immediate)
				//instructions we will need: 	
				//addiu addu jr li lw move nop sw
				//pages refer to pdf page # in MIPS ISA
				instr_reg <= instr_in;
				case(instr_in[31:26])
					6'b001001: begin
						//ADDIU rt, rs, immediate
						//add immediate unsigned (pg 47)
						//[31:26]: 001001
						// [0010 01][11 101][1 1101]
						wr_sel <= instr_in[20:16]; //store result here (rt)
						rs_sel <= instr_in[25:21]; //operand 1, rs 
						//alu_1 <= rs;
						sign_extend_en <= 1'b1; //operand 2, immediate, sets alu_2
						alu_en <= 1'b1;
						alu_op <= 5'b00001;
						wb_en <= 1'b1;
					end
					6'b100011: begin
						//LW rt, offset(base)
						//load word from memory (pg 171) 
						//[31:26]: 100011
						wr_sel <= instr_in[20:16]; // store from mem to this reg (rt)
						rs_sel <= instr_in[25:21]; // base addr
						//alu_1 <= rd;
						sign_extend_en <= 1'b1; //sets alu_2 with offset
						alu_en <= 1'b1;
						alu_op <= 5'b00001;
						wb_en <= 1'b1;
						mem_en <= 1'b1;
					end
					//NOTE: LW and SW seem similar so far - change to common stage? only diff is 10(0/1)011
					6'b101011: begin
						//SW, rt, offset(base)
						//store word from rt to memory[base+offset] (pg 280)
						//address error exception if last two bits != 00
						//[31:26]: 101011
						wr_sel <= instr_in[20:16]; //writeback setup
						rs_sel <= instr_in[25:21]; //base address
						rd_sel <= instr_in[20:16]; //source word
						//alu_1 <= rd;
						sign_extend_en <= 1'b1; //sets alu_2 with immediate
						alu_en <= 1'b1;
						alu_op <= 5'b00001;
						wb_en <= 1'b0;
						mem_en <= 1'b1;
					end
					6'b000010: begin
					//J
					// 000010 instr_index [25:0]
					// I:
					// I+1:PC ← PCGPRLEN-1..28 || instr_index || 02
						wr_reg <= { pc[31:28], instr_index[25:0], 2'b00 };
						rs_sel = instr_in[20:16];
						wr_sel = instr_in[25:21];
						rd_sel <= instr_in[15:11];
						alu_op <= 5'b00001;
						alu_en <= 1'b1;
						wb_en <= 1'b0;
						mem_en <= 1'b1;
					end
					6'b000011: begin
					//JAL
					//I: GPR[31]← PC + 8
					//I+1:PC ← PCGPRLEN-1..28 || instr_index || 02
					// 000011 instr_index [25:0]
						wr_sel <= 5'h1f;
						wr_reg <= pc + 8; //{pc[31:28], instr_index[25:0], 2'b00};
						rs_sel = instr_in[20:16];
						alu_op <= 5'b10000;
						link_en <= 1'b1;
						alu_en <= 1'b1;
						wb_en <= 1'b0;
						mem_en <= 1'b1;
					end
					6'b000100 : begin
						//BEQ
						//  rs [25:21] rt [20:16] offset [15:0]
						//I: target_offset ← sign_extend(offset || 02)
						//	condition ← (GPR[rs] = GPR[rt])
						//I+1: if condition then
						//		PC ← PC + target_offset
						//	 endif
						rs_sel = instr_in[25:21];
						rd_sel = instr_in[20:16];
						if rs == rd begin
							alu_en = 1'b1;
							sign_extend = 1'b1;
							alu_op <= 5'b10000;
						end else begin
							alu_en = 1'b0;
							sign_extend = 1'b0;
							alu_op <= 5'b00000;
						end
						wb_en <= 1'b0;
						mem_en <= 1'b0;
					end
					6'b000101: begin
						//BNE
						// 000101 rs [25:21] rt [20:16] offset [15:0]
						rs_sel = instr_in[25:21];
						rd_sel = instr_in[20:16];
						if rs == rd begin
							alu_en = 1'b0;
							sign_extend = 1'b0;
							alu_op <= 5'b00000;
						end else begin
							alu_en = 1'b1;
							sign_extend = 1'b1;
							alu_op <= 5'b10000;
						end
						wb_en <= 1'b0;
						mem_en <= 1'b0;
					end
					6'b001010: begin
					//SLTI set on less than immediate
					// 001010 rs [26:21] rt [20:16] immediate [15:0]
					// rt <- (rs < immediate)
					//if GPR[rs] < sign_extend(immediate)
					//then GPR[rt] ← 0GPRLEN-1|| 1
					//else
					//GPR[rt] ← 0GPRLEN
					//endif
						rs_sel = instr_in[20:16];
						wr_reg <= {16'b0, instr_in[15:0]};
						wr_sel = instr_in[25:21];
						rd_sel <= instr_in[15:11];
						sign_extend_en <= 1'b1;
						alu_op <= 5'b00001;
						alu_en <= 1'b1;
						wb_en <= 1'b0;
						mem_en <= 1'b1;
					end
					6'b001111: begin 
					//LUI load upper immediate
					// 001111 00000 rt [20:16] immediate [15:0]
					// rt <= immediate || 0000 0000 0000 0000
						wr_sel = instr_in[20:16];
						rd_sel <= instr_in[15:11];
						wr_reg <= {instr_in[15:0], 16'b0};
						wb_en <= 1'b1;
						alu_en <= 1'b0;
						alu_op <= 5'b00;
						mem_en <= 1'b0;
					end
					
					//not needed
					//SB store byte
					// 101000 base [25:21] rt [20:16] offset [15:0]
					// memory[base+offset] <- rt
					//LBU load byte unsigned
					// 100100 base [25:21] rt [20:16] offset [15:0]
					// rt <- memory[base+offset]
					6'b000000 : begin  //SPECIAL
						case(instr_in[5:0]) 
							6'b100001 : begin //ADDU
								//ADDU rt, rd, rs
								//add unsigned word (pg 48)
								//[31:26]: 000000 & [5:0]: 100001
								rs_sel = instr_in[25:21]; //op1
								rd_sel = instr_in[20:16]; //op2
								wr_sel <= instr_in[15:11]; //destination
								alu_en <= 1'b1;
								wb_en  <= 1'b1;
								alu_op <= 5'b00001;
								mem_en <= 1'b0;
							end
							6'b100011 : begin //SUBU
								//SUBU sub unsigned
								// 000000 rs [25:21] rt [20:16] rd [15:11] 00000 100011 (SUBU)
								// rd <- rs - rt
								rs_sel = instr_in[25:21]; //op2
								wr_sel = instr_in[15:11]; //destination
								rd_sel <= instr_in[20:16]; //op1
								alu_en <= 1'b1;
								alu_op <= 5'b00010;
								wb_en <= 1'b1;
								mem_en <= 1'b0;
							end
							6'b001000 : begin
								//JR rs
								//jump register, set PC to rs (pg 155) 
								//[31:26]: 000000 & [5:0]: 0010000
								rs_sel = instr_in[25:21];
								pc = rs;
								wb_en <= 1'b0;
								alu_en <= 1'b0;
								alu_op <= 5'b00;
								mem_en <= 1'b0;
							end
							6'b001010 : begin
								//LI rt, immediate
								//load immediate to reg (not in ISA - is this upper or lower? or full 32 bit?)
								//immediate in lower 16
								//upper 16: 2402 = 0010 1000 0000 0010
								wr_sel <= instr_in[25:21];
								wr_reg <= {16'b0, instr_in[15:0]};
								wb_en <= 1'b1;
								alu_en <= 1'b0;
								alu_op <= 5'b00;
								mem_en <= 1'b0;
							end
							6'b000101 : begin
								//MOVE rd, rs
								//move register to register
								//03a0f021 03c0e821 : 000000 rs rt rd 000000 opcode
								//0000 0011 0000 0000 0000 0000 1000 0101
								wr_sel = instr_in[15:11];
								rs_sel = instr_in[26:21];
								wr_reg = rs;
								wb_en <= 1'b1;
								alu_en <= 1'b0;
								alu_op <= 5'b00000;
								mem_en <= 1'b0;
							end
							6'b000000 : begin
								//NOP
								//not an op, actually SLL r0, r0, 0 (pg 226) do we need to actually implement this way?
								//[31:26]: 000000 & SLL [5:0]=000000
								//r0 <= r0 << 1'b0;
								//SLL
								// 000000 00000 rt [20:16] rd [15:11] sa [10:6] 000000
								//rd <- rt << sa
								rd_sel = instr_in[10:6];
								wr_sel = instr_in[15:11];
								rs_sel <= instr_in[20:16];
								wb_en <= 1'b1;
								alu_en <= 1'b1;
								alu_op <= 5'b01000;
								mem_en <= 1'b0;
							end
						endcase
					b'011100 : begin	//SPECIAL2
						case(instr_in[5:0])
							b'000010 : begin
								//MUL
								// 011100 (SPECIAL2) rs [25:21] rt [20:16] rd[15:11] 00000 000010 (MUL)
								//rd <- rs * rt;
								rs_sel = instr_in[20:16];
								wr_sel = instr_in[15:11];
								rd_sel <= instr_in[25:21];
								alu_en <= 1'b1;
								alu_ops <= 5'b01000;
								wb_en <= 1'b0;
								mem_en <= 1'b1;
							end
						endcase
					end
				endcase
				instr_counter <= instr_counter << 1;
			end
			5'b00100 : begin //EX
				//execute arithmetic and instruction, eg. address = base + offset
				//run if alu_en is set
				//adds alu_1 and alu_2
				//output to alu_out
				if(alu_en == 1'b1) begin
					case(alu_op)
						5'b01000: begin //SLL
							alu_out <= rd << rs;
							wr_reg <= rd << rs;
						end
						5'b00100: begin //MUL
							alu_out <= rs * rd;
							wr_reg <= rs * rd;
						end
						5'b00010: begin //SUBU
							alu_out <= rs - rd;
							wr_reg <= rs - rd;
						end
						5'b00001: begin //ADD
							if(sign_extend_en) begin
								if(instr_reg[15]) begin
									alu_out <= rs + { 16'hffff, instr_reg[15:0]};
									wr_reg <= rs + { 16'hffff, instr_reg[15:0]};
								end else begin
									alu_out <= rs + { 16'h0000, instr_reg[15:0]};
									wr_reg <= rs + { 16'h0000, instr_reg[15:0]};
								end 
							end
						end
						5'b10000 : begin //BRANCH OFFSET
							//Sign Extend should always be '1' don't explicitally check
							if(instr_reg[15]) begin
								pc <= pc + {12'hfff, 2'b11, instr_reg[15:0], 2'b00};
							end else begin
								pc <= pc + {12'h000, 2'b00, binstr_reg[15:0], 2'b00};
							end 
						end
					endcase
				end
				alu_en <= 1'b0;
				mul_en <= 1'b0;
				pc <= pc + 4;//increment PC after done with instr_in
				instr_counter <= instr_counter << 1;
			end
			5'b01000 : begin //ME
				//read from calculated address
				// calculated addr is either alu_out / reg (rt)
				if(mem_en) begin
					if(instr_reg[29]) begin //TODO: Add write flag to avoid holding this reg
						data_addr = alu_out;
						data_rw_en = 1'b0;
						data_out <= rd;
					end else begin
						data_addr = alu_out;
						data_rw_en = 1'b1;
						wr_reg <= data_in;
					end
				end
				mem_en <= 1'b0;
				instr_counter <= instr_counter << 1;
			end
			5'b10000 : begin //WB
				//write back to register if required
				if(wb_en) begin
					reg_wr_en <= 1'b1;
				end
				wb_en <= 1'b0;
				instr_counter <= 5'b00001;
			end
		endcase

		//
	end // clock
	
	task sign_extend(input logic [15:0] val);
	begin
		//TODO: determine what reg to write to (ALU input 2)
		sign_extend_en = 1'b1;
	end
	endtask

endmodule