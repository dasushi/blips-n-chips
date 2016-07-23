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
				 ME_reg_wr_en,
				 ME_mem_en,
				 WB_wb_en,
				 WB_reg_wr_en,
				 link_en,
				 stall,
				 stall_deasserted;			 
	
	logic hit_slti;
	
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
				 alu_rs, 			// register output 2
				 alu_rd, 			// destination register
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
		link_en 	<= 1'b0;
		stall 		<= 1'b0;
		stall_deasserted <= 1'b0;
		
		hit_slti <= 1'b0;
		
		//WB_wr_sel 		<= 5'h1D; //sp_init = R29
		//WB_wr_reg_data	<= sp_init;
		WB_reg_wr_en    <= 1'b1;
		ME_reg_wr_en	<= 1'b0;
		EX_reg_wr_en	<= 1'b0;
		
		//WB_wr_sel 		<= 5'h1E; //ra_init = R31
		//WB_wr_reg		<= ra_init;
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
			IF_en 			<= 1'b1;
			WB_wr_sel		<= 5'h1D; //sp_init = R29
			WB_wr_reg_data	<= sp_init;
			//WB_wr_sel		<= 5'h1E; //fp_init = R30
			//WB_wr_reg_data  <= ra_init;
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
			if(!stall && (ID_instr_reg[31:26] == 6'b100011) && (instr_in[20:16] == ID_instr_reg[20:16])) begin
				stall <= 1'b1;
				stall_deasserted = 1'b1;
			end else if(!stall && (ID_instr_reg[31:26] == 6'b101011) && (instr_in[31:26] == 6'b101011)) begin
				stall <= 1'b1; //There's a delay in SW for some reason, can't for the life of me figure out where in main_memory.sv
			end else if(!stall && (ID_instr_reg[31:26] == 6'b000000 && ID_instr_reg[5:0] == 6'b100001)) begin 
				if(instr_in[15:11] == instr_in[20:16]) begin
					stall <= 1'b1;
				end else if(instr_in[15:11] == instr_in[25:21]) begin
					stall <= 1'b1;
				end
			end else begin
				stall <= 1'b0;
				//ID_instr_reg <= instr_in;
				//ID_en <= 1'b1;
			end
			if(!stall) begin
				ID_instr_reg <= instr_in;
				ID_en <= 1'b1;
			end
		end else begin
			WB_wr_sel <= 5'h1F; //ra_init = R31
			WB_wr_reg_data <= ra_init;
			WB_reg_wr_en <= 1'b1;
		end
	end //IF
	
	always @ (posedge clk) 
	begin: ID
		if(stall) begin
			EX_instr_reg <= EX_instr_reg;
		end else if(ID_en) begin
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
			6'b001111: begin 
				//LUI load upper immediate
				// 001111 00000 rt [20:16] immediate [15:0]
				// rt <= immediate || 0000 0000 0000 0000
				EX_wr_sel = instr_in[20:16];
				rd_sel <= instr_in[15:11];
				EX_wr_reg <= {instr_in[15:0], 16'b0};
				EX_wb_en <= 1'b1;
				alu_en <= 1'b0;
				alu_op <= 6'b0;
				EX_mem_en <= 1'b0;
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
				//May need to stall for sequential stores, doesn;t seem like the correct values is getting clocked in by the memory
				EX_wr_sel <= 5'bx; //writeback setup
				rs_sel <= ID_instr_reg[25:21]; //base address
				rd_sel <= ID_instr_reg[20:16]; //source word
				sign_extend_en <= 1'b1; //sets alu_2 with immediate
				alu_en <= 1'b1;
				alu_op <= 5'b00001;
				EX_wb_en <= 1'b0;
				EX_mem_en <= 1'b1;
			end
			6'b000010: begin
				//J
				// 000010 instr_index [25:0]
				// I:
				// I+1:PC ← PCGPRLEN-1..28 || instr_index || 02
				alu_op <= 6'b010000;
				link_en <= 1'b1;
				alu_en <= 1'b1;
				EX_wb_en <= 1'b0;
				EX_mem_en <= 1'b0;
				if(ID_instr_reg[15]) begin //Sign Extend should always be '1' don't explicitally check
					pc <= pc + {12'hfff, 2'b11, ID_instr_reg[15:0], 2'b00};
				end else begin
					pc <= pc + {12'h000, 2'b00, ID_instr_reg[15:0], 2'b00};
				end 
			end
			6'b000011: begin
				//JAL
				//I: GPR[31]← PC + 8
				//I+1:PC ← PCGPRLEN-1..28 || instr_index || 02
				// 000011 instr_index [25:0]
				//set GPR31
				EX_wr_sel = 5'h1f; //select reg 31
				EX_reg_wr_en = 1'b1; //write to reg
				EX_wr_reg = pc + 8; //increment by 8
				alu_op <= 6'b010000; //same op as branch
				link_en <= 1'b1; //indicate to calculate link address using instr_index[25:0]
				alu_en <= 1'b1;
				EX_wb_en <= 1'b0;
				EX_mem_en <= 1'b0;
				pc <= {pc[31:28], ID_instr_reg[25:0], 2'b00};
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
					if(EX_instr_reg[15]) begin
						pc <= pc + {12'hfff, 2'b11, ID_instr_reg[15:0], 2'b00};
					end else begin
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
			6'b000101: begin
				//BNE
				// 000101 rs [25:21] rt [20:16] offset [15:0]
				rs_sel = ID_instr_reg[25:21];
				rd_sel = ID_instr_reg[20:16];
				if (rs != rd) begin
					alu_en = 1'b1;
					sign_extend_en = 1'b1;
					alu_op <= 5'b10000;
					if(EX_instr_reg[15]) begin //Sign Extend should always be '1' don't explicitally check
						pc <= pc + {12'hfff, 2'b11, ID_instr_reg[15:0], 2'b00};
					end else begin
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
			6'b001010: begin
				//SLTI set on less than immediate
				// 001010 rs [25:21] rt [20:16] immediate [15:0]
				// rt <- (rs < immediate)
				//if GPR[rs] < sign_extend(immediate)
				//then GPR[rt] ← 0GPRLEN-1|| 1
				//else
				//GPR[rt] ← 0GPRLEN
				//endif
				rs_sel <= instr_in[25:21];
				EX_wr_sel <= instr_in[20:16];
				alu_op <= 5'b10000;
				alu_en <= 1'b1;
				EX_wb_en <= 1'b1; //rt has to be written back to
				EX_mem_en <= 1'b0;
			end
			6'b000000 : begin  //SPECIAL
				case(ID_instr_reg[5:0])
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
						rs_sel <= ID_instr_reg[25:21];
						//pc <= rs;
						alu_en <= 1'b1;
						alu_op <= 5'b00100;
						EX_wb_en <= 1'b0;
						EX_mem_en <= 1'b0;
					end
					6'b001010 : begin
						//LI rt, immediate
						//load immediate to reg (not in ISA - is this upper or lower? or full 32 bit?)
						//immediate in lower 16
						//upper 16: 2402 = 0010 1000 0000 0010
						EX_wr_sel <= instr_in[25:21];
						EX_wr_reg <= {16'b0, instr_in[15:0]};
						EX_wb_en <= 1'b1;
						alu_en <= 1'b0;
						alu_op <= 5'b0;
						EX_mem_en <= 1'b0;
					end
					6'b000101 : begin
						//MOVE rd, rs
						//move register to register
						//03a0f021 03c0e821 : 000000 rs rt rd 000000 opcode
						//0000 0011 0000 0000 0000 0000 1000 0101
						EX_wr_sel = instr_in[15:11];
						rs_sel = instr_in[26:21];
						EX_wr_reg = rs;
						EX_wb_en <= 1'b1;
						alu_en <= 1'b0;
						alu_op <= 5'b0;
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
					default: begin
					//6'b100001 : begin //ADDU
						//ADDU rt, rd, rs
						//add unsigned word (pg 48)
						//[31:26]: 000000 & [5:0]: 100001
						rs_sel <= ID_instr_reg[25:21]; //op1
						rd_sel <= ID_instr_reg[20:16]; //op2
						EX_wr_sel <= ID_instr_reg[15:11]; //destination

						alu_en <= 1'b1;
						EX_wb_en  <= 1'b1;
						alu_op <= 5'b00011;
						EX_mem_en <= 1'b0;
						sign_extend_en <= 1'b0;
					end 
				endcase
			end
			6'b011100 : begin	//SPECIAL2
				case(instr_in[5:0])
					6'b000010 : begin
						//MUL
						// 011100 (SPECIAL2) rs [25:21] rt [20:16] rd[15:11] 00000 000010 (MUL)
						//rd <- rs * rt;
						rs_sel <= ID_instr_reg[20:16];
						EX_wr_sel = ID_instr_reg[15:11];
						rd_sel <= ID_instr_reg[25:21];
						alu_en <= 1'b1;
						alu_op <= 5'b11000;
						EX_wb_en <= 1'b1;
						EX_mem_en <= 1'b0;
					end
				endcase
			end
		endcase
		EX_en <= 1'b1;
		end else begin
			if(IF_en) begin
				WB_wr_sel		<= 5'h1E; //fp_init = R30
				WB_wr_reg_data  <= sp_init;
				WB_reg_wr_en    <= 1'b1;
			end
		end
	end // ID
	
	always @ (posedge clk)
	begin: EX
		if(EX_en) begin		
			//execute arithmetic and instruction, eg. address = base + offset
			//run if alu_en is set
			//adds alu_1 and alu_2
			//output to alu_out
			if(alu_op == 5'b10000) begin //Fo some reason isnt getting hit in the case statement No clue
				if(EX_instr_reg[15]) begin
					if(alu_rs < {16'hffff, EX_instr_reg[15:0]}) begin
						ME_wr_reg <= 32'b1;
						alu_out <= 32'b1;
				end else begin
					ME_wr_reg <= 32'b0;
					alu_out <= 32'b0;
				end	
				end else begin
					if(alu_rs < {16'h0000, EX_instr_reg[15:0]}) begin
						ME_wr_reg <= 32'b1;
						alu_out <= 32'b1;
					end else begin
						ME_wr_reg <= 32'b0;
						alu_out <= 32'b0;
					end	
				end
			end
			if(alu_en == 1'b1) begin
				case(alu_op)
					5'b01000: begin //SLL
						alu_out <= alu_rs << EX_instr_reg[10:6];
						ME_wr_reg <= alu_rs << EX_instr_reg[10:6];
					end
					5'b11000: begin //MUL
						alu_out <= alu_rs * alu_rd;
						ME_wr_reg <= alu_rs * alu_rd;
					end
					5'b10000 : begin //SLTI
						if(EX_instr_reg[15]) begin
							if(alu_rs < {16'hffff, EX_instr_reg[15:0]}) begin
								ME_wr_reg <= 32'b1;
						end else begin
							ME_wr_reg <= 32'b0;
						end	
						end else begin
							if(alu_rs < {16'h0000, EX_instr_reg[15:0]}) begin
								ME_wr_reg <= 32'b1;
								alu_out <= 32'b1;
							end else begin
								ME_wr_reg <= 32'b0;
							end	
						end
					end
					5'b00010: begin //SUBU
						alu_out <= alu_rs - alu_rd;
						ME_wr_reg <= alu_rs - alu_rd;
					end
					5'b00011: begin //ADDU
							ME_wr_sel <= EX_wr_sel;
							ME_wr_reg <= alu_rs + alu_rd;
							alu_out = alu_rs + alu_rd;
						end
					5'b00001: begin //ADDIU
						if(sign_extend_en) begin
							if(EX_instr_reg[15]) begin
								alu_out  = alu_rs + { 16'hffff, EX_instr_reg[15:0]};
								data_out = alu_rd;
								if (~EX_mem_en) begin
									ME_wr_reg <= alu_rs + { 16'hffff, EX_instr_reg[15:0]};
								end else begin
									if(ME_instr_reg[15:11] == ME_instr_reg[20:16]) begin
										data_out <= ME_wr_reg;
									end else if(ME_instr_reg[15:11] == ME_instr_reg[25:21]) begin
										data_out <= ME_wr_reg;
									end else begin
										ME_wr_reg <= alu_rd;
									end
								end
							end else begin
								alu_out = alu_rs + { 16'h0000, EX_instr_reg[15:0]};
								data_out = alu_rd;
								if (~EX_mem_en) begin
									ME_wr_reg <= alu_rs + { 16'h0000, EX_instr_reg[15:0]};
								end else begin
									if(ME_instr_reg[15:11] == ME_instr_reg[20:16]) begin
										data_out <= ME_wr_reg;
									end else if(ME_instr_reg[15:11] == ME_instr_reg[25:21]) begin
										data_out <= ME_wr_reg;
									end else begin
										ME_wr_reg <= alu_rd;
									end
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
							alu_out <= alu_rs + alu_rd;
							ME_wr_reg <= alu_rs + alu_rd;
						end	
					end
					5'b00100: begin //JUMP
						pc <= alu_rs;
					end	
				endcase
			end
			if(~(ID_instr_reg[31:26] == 6'b000100) && (alu_op != 5'b00100) && !stall) begin
				// branch delay
				pc <= pc + 4;//increment PC after done with instr_in
			end 
			ME_wb_en <= EX_wb_en;
			ME_mem_en <= EX_mem_en;
			ME_reg_wr_en <= EX_reg_wr_en;
			ME_wr_sel <= EX_wr_sel; // This isn't the case for ADDU
			ME_instr_reg <= EX_instr_reg;
			ME_en <= 1'b1;
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
	
	always @ (rs)
	begin 
		if(!stall && stall_deasserted) begin
			alu_rs = rs;
			stall_deasserted <= 1'b0;
		end else
		if(rs_sel == ME_wr_sel && WB_en) begin
			alu_rs = ME_wr_reg;
		end else if(WB_wr_sel == ME_wr_sel) begin
			alu_rs = rs; // Forward the more recent data
		end else if(rs_sel == WB_wr_sel) begin
			alu_rs = WB_wr_reg_data;
		end else begin
			alu_rs = rs;
		end
	end
	
	always @ (rd)
	begin
		if(!stall && stall_deasserted) begin
			alu_rd = rd;
			stall_deasserted <= 1'b0;
		end else if(rd_sel == ME_wr_sel && WB_en) begin
			alu_rd = ME_wr_reg;
		end else if(WB_wr_sel == ME_wr_sel) begin
			alu_rd = rd; // Forward the more recent data
		end else if(rd_sel == WB_wr_sel) begin
			alu_rd = WB_wr_reg_data;
		end else begin
			alu_rd = rd;
		end
	end
endmodule