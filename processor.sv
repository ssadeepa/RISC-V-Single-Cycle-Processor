// RISC-V Single-Cycle Processor Implementation

module processor(input  logic        clk, rst,
                 output logic [31:0] write_data, address, 
                 output logic        mem_write);

  logic [31:0] prog_counter, instruction, read_data;
  
  // Instantiate CPU and memories
  core cpu(clk, rst, prog_counter, instruction, mem_write, address, 
           write_data, read_data);
  instruction_memory imem(prog_counter, instruction);
  data_memory dmem(clk, mem_write, address, write_data, read_data);
endmodule

module core(input  logic        clk, rst,
            output logic [31:0] prog_counter,
            input  logic [31:0] instruction,
            output logic        mem_write,
            output logic [31:0] alu_result, write_data,
            input  logic [31:0] read_data);

  logic       alu_src_a, alu_src_b;
  logic       reg_write, jump, zero_flag;
  logic [1:0] result_mux_select;
  logic [2:0] immediate_select;
  logic [2:0] alu_ctrl;

  control_unit cu(instruction[6:0], instruction[14:12], instruction[30], zero_flag,
                  result_mux_select, mem_write, pc_source,
                  alu_src_a, alu_src_b, reg_write, jump,
                  immediate_select, alu_ctrl);
  datapath dp(clk, rst, result_mux_select, pc_source,
              alu_src_a, alu_src_b, reg_write,
              immediate_select, alu_ctrl,
              zero_flag, prog_counter, instruction,
              alu_result, write_data, read_data);
endmodule

module control_unit(input  logic [6:0] opcode,
                    input  logic [2:0] funct3,
                    input  logic       funct7_bit5,
                    input  logic       zero_flag,
                    output logic [1:0] result_mux_select,
                    output logic       mem_write,
                    output logic       pc_source, 
                    output logic       alu_src_a, alu_src_b,  
                    output logic       reg_write, jump,
                    output logic [2:0] immediate_select,  
                    output logic [2:0] alu_ctrl);     

  logic [1:0] alu_op;
  logic       branch;

  main_decoder md(opcode, result_mux_select, mem_write, branch,
                  alu_src_a, alu_src_b, reg_write, jump, immediate_select, alu_op);
  alu_decoder  ad(opcode[5], funct3, funct7_bit5, alu_op, alu_ctrl);

  assign pc_source = branch & zero_flag | jump;
endmodule

module main_decoder(input  logic [6:0] opcode,
                    output logic [1:0] result_mux_select,
                    output logic       mem_write,
                    output logic       branch, 
                    output logic       alu_src_a, alu_src_b, 
                    output logic       reg_write, jump,
                    output logic [2:0] immediate_select,   
                    output logic [1:0] alu_op);

  logic [12:0] controls;

  assign {reg_write, immediate_select, alu_src_a, alu_src_b, mem_write,
          result_mux_select, branch, alu_op, jump} = controls;

  always_comb
    case(opcode)
      // reg_write_imm_select_alu_src_a_alu_src_b_mem_write_result_mux_branch_alu_op_jump
      7'b0000011: controls = 13'b1_000_0_1_0_01_0_00_0; // lw
      7'b0100011: controls = 13'b0_001_0_1_1_00_0_00_0; // sw
      7'b0110011: controls = 13'b1_xxx_0_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 13'b0_010_0_0_0_00_1_01_0; // beq
      7'b0010011: controls = 13'b1_000_0_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 13'b1_011_0_1_0_10_0_00_1; // jal
      7'b0110111: controls = 13'b1_100_1_1_0_00_0_00_0; // lui
      default:    controls = 13'bx_xxx_x_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module alu_decoder(input  logic       opcode_bit5,
                   input  logic [2:0] funct3,
                   input  logic       funct7_bit5, 
                   input  logic [1:0] alu_op,
                   output logic [2:0] alu_ctrl);

  logic  is_subtraction;
  assign is_subtraction = funct7_bit5 & opcode_bit5;  // TRUE for R-type subtract instruction

  always_comb
    case(alu_op)
      2'b00:                alu_ctrl = 3'b000; // addition
      2'b01:                alu_ctrl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (is_subtraction) 
                            alu_ctrl = 3'b001; // sub
                          else          
                            alu_ctrl = 3'b000; // add, addi
                 3'b010:    alu_ctrl = 3'b101; // slt, slti
                 3'b110:    alu_ctrl = 3'b011; // or, ori
                 3'b111:    alu_ctrl = 3'b010; // and, andi
                 3'b100:    alu_ctrl = 3'b100; // xor, xori
                 default:   alu_ctrl = 3'bxxx; // Undefined
               endcase
    endcase
endmodule

module datapath(input  logic        clk, rst,
                input  logic [1:0]  result_mux_select, 
                input  logic        pc_source, 
                input  logic        alu_src_a, alu_src_b, 
                input  logic        reg_write,
                input  logic [2:0]  immediate_select,  
                input  logic [2:0]  alu_ctrl,
                output logic        zero_flag,
                output logic [31:0] prog_counter,
                input  logic [31:0] instruction,
                output logic [31:0] alu_result, write_data,
                input  logic [31:0] read_data);

  logic [31:0] next_pc, pc_plus4, branch_target;
  logic [31:0] immediate_ext;
  logic [31:0] source_a, source_b;
  logic [31:0] reg_output_a;  
  logic [31:0] alu_output;

  // Next PC Logic
  reg_file #(32) pc_reg(clk, rst, next_pc, prog_counter); 
  adder       pc_increment(prog_counter, 32'd4, pc_plus4);
  adder       branch_add(prog_counter, immediate_ext, branch_target);
  mux2 #(32)  pc_mux(pc_plus4, branch_target, pc_source, next_pc);
 
  // Register File Logic
  register_file rf(clk, reg_write, instruction[19:15], instruction[24:20], 
                   instruction[11:7], alu_output, reg_output_a, write_data);
  sign_extender ext(instruction[31:7], immediate_select, immediate_ext);

  // ALU Logic
  mux2 #(32)  source_a_mux(reg_output_a, 32'b0, alu_src_a, source_a); 
  mux2 #(32)  source_b_mux(write_data, immediate_ext, alu_src_b, source_b);
  alu         alu_unit(source_a, source_b, alu_ctrl, alu_result, zero_flag);
  mux3 #(32)  result_mux(alu_result, read_data, pc_plus4, result_mux_select, alu_output);
endmodule

module register_file(input  logic        clk, 
                     input  logic        we3, 
                     input  logic [ 4:0] a1, a2, a3, 
                     input  logic [31:0] wd3, 
                     output logic [31:0] rd1, rd2);

  logic [31:0] regs[31:0];

  // Three ported register file
  // Read two ports combinationally (a1/rd1, a2/rd2)
  // Write third port on rising edge of clock (a3/wd3/we3)
  // Register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) regs[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? regs[a1] : 0;
  assign rd2 = (a2 != 0) ? regs[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] sum);

  assign sum = a + b;
endmodule

module sign_extender(input  logic [31:7] instr,
                     input  logic [2:0]  immsrc,   
                     output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
      3'b000:   immext = {{20{instr[31]}}, instr[31:20]};  // I-type
      3'b001:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};  // S-type (stores)
      3'b010:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};  // B-type (branches)
      3'b011:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};  // J-type (jal)
      3'b100:   immext = {instr[31:12], 12'b0};  // U-type (lui)
      default:  immext = 32'bx; // Undefined
    endcase             
endmodule

module reg_file #(parameter WIDTH = 8)
                 (input  logic             clk, rst,
                  input  logic [WIDTH-1:0] d, 
                  output logic [WIDTH-1:0] q);

  always_ff @(posedge clk or posedge rst)
    if (rst) q <= 0;
    else     q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             sel, 
              output logic [WIDTH-1:0] y);

  assign y = sel ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       sel, 
              output logic [WIDTH-1:0] y);

  assign y = sel[1] ? d2 : (sel[0] ? d1 : d0); 
endmodule

module instruction_memory(input  logic [31:0] addr,
                          output logic [31:0] rd);

  logic [31:0] memory[63:0];

  initial
      $readmemh("C:/z/Electronics/P_IMP/Github/RISC_V_Single_Cycle_Core/RISC_V_Single_Cycle_Core.srcs/sim_1/new/instruction_mem.mem", memory);

  assign rd = memory[addr[31:2]]; // word aligned
endmodule

module data_memory(input  logic        clk, we,
                   input  logic [31:0] addr, wd,
                   output logic [31:0] rd);

  logic [31:0] memory[63:0];

  assign rd = memory[addr[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) memory[addr[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alu_ctrl,
           output logic [31:0] result,
           output logic        zero_flag);

  logic [31:0] conditional_b, sum_result;
  logic        overflow;       
  logic        is_add_sub_op;  

  assign conditional_b = alu_ctrl[0] ? ~b : b;
  assign sum_result = a + conditional_b + alu_ctrl[0];
  assign is_add_sub_op = ~alu_ctrl[2] & ~alu_ctrl[1] |
                         ~alu_ctrl[1] & alu_ctrl[0];

  always_comb
    case (alu_ctrl)
      3'b000:  result = sum_result;         // add
      3'b001:  result = sum_result;         // subtract
      3'b010:  result = a & b;              // and
      3'b011:  result = a | b;              // or
      3'b100:  result = a ^ b;              // xor
      3'b101:  result = sum_result[31] ^ overflow; // slt
      3'b110:  result = a << b[4:0];        // sll
      3'b111:  result = a >> b[4:0];        // srl
      default: result = 32'bx;
    endcase

  assign zero_flag = (result == 32'b0);
  assign overflow = ~(alu_ctrl[0] ^ a[31] ^ b[31]) & (a[31] ^ sum_result[31]) & is_add_sub_op;
  
endmodule
