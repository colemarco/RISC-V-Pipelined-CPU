`timescale 1ns/1ps
`default_nettype none

`include "alu_types.sv"
`include "rv32i_defines.sv"

module rv32i_pipelined_core(
  clk, rst, ena,
  instr_mem_addr, instr_mem_rd_data, instr_mem_wr_data, instr_mem_wr_ena,
  data_mem_addr, data_mem_rd_data, data_mem_wr_data, data_mem_wr_ena,
  PC
);

parameter PC_START_ADDRESS=0;

// Standard control signals.
input  wire clk, rst, ena; // <- worry about implementing the ena signal last.

// Memory interface.
output logic [31:0] instr_mem_addr;
input   wire [31:0] instr_mem_rd_data;
// Technically not needed
output logic [31:0] instr_mem_wr_data;
output logic instr_mem_wr_ena;
always_comb begin
  instr_mem_wr_ena = 1'b0;
  instr_mem_wr_data = {32{1'b0}};
end

output logic [31:0] data_mem_addr, data_mem_wr_data;
input wire [31:0] data_mem_rd_data;
output logic data_mem_wr_ena;

// Program Counter
output wire [31:0] PC;
wire [31:0] PC_old;
logic PC_ena;
logic [31:0] PC_next; 

always_comb begin
  PC_ena = ~stall_f;
end

// Program Counter Registers
register #(.N(32), .RESET(PC_START_ADDRESS)) PC_REGISTER (
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC_next), .q(PC)
);
register #(.N(32)) PC_OLD_REGISTER(
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC), .q(PC_old)
);

//  an example of how to make named inputs for a mux:
/*
    enum logic {MEM_SRC_PC, MEM_SRC_RESULT} mem_src;
    always_comb begin : memory_read_address_mux
      case(mem_src)
        MEM_SRC_RESULT : mem_rd_addr = alu_result;
        MEM_SRC_PC : mem_rd_addr = PC;
        default: mem_rd_addr = 0;
    end
*/

// Register file
logic [4:0] rd, rs1_d, rs2_d;
logic [31:0] rfile_wr_data;
wire [31:0] reg_data1, reg_data2;
register_file REGISTER_FILE(
  .clk(clk), 
  .wr_ena(reg_write_w), .wr_addr(rd_w), .wr_data(result_w),
  .rd_addr0(rs1_d), .rd_addr1(rs2_d),
  .rd_data0(reg_data1), .rd_data1(reg_data2)
);

// ALU and related control signals
// Feel free to replace with your ALU from the homework.
// logic [31:0] src_a, src_b;
alu_control_t alu_control;
wire [31:0] alu_result;
wire overflow, zero_e, equal;
alu_behavioural ALU (
  .a(src_a_e), .b(src_b_e), .result(alu_result),
  .control(alu_control_e),
  .overflow(overflow), .zero(zero_e), .equal(equal)
);

// Implement your multicycle rv32i CPU here!

// Control Unit *****************************************************
logic pc_src_e;
always_comb begin : PC_CONTROL
  pc_src_e = (zero_e && branch_e) || jump_e;
end



logic reg_write_d;
logic [1:0] result_src_d;
logic mem_write_d;
logic jump_d;
logic branch_d;
alu_control_t alu_control_d;
logic alu_src_d;
logic [1:0] alu_op;
enum logic [2:0] {EXTEND_INVALID, EXTEND_I, EXTEND_S, EXTEND_B, EXTEND_J} imm_src_d;

always_comb begin : MAIN_DECODER
  case(op)
    LW : begin
      reg_write_d = 1'b1;
      imm_src_d = EXTEND_I;
      alu_src_d = 1'b1;
      mem_write_d = 1'b0;
      result_src_d = 2'b01;
      branch_d = 1'b0;
      alu_op = 2'b00;
      jump_d = 1'b0;
    end
    S_TYPE : begin
      reg_write_d = 1'b0;
      imm_src_d = EXTEND_S;
      alu_src_d = 1'b1;
      mem_write_d = 1'b1;
      result_src_d = 2'bxx;
      branch_d = 1'b0;
      alu_op = 2'b00;
      jump_d = 1'b0;
    end
    R_TYPE : begin
      reg_write_d = 1'b1;
      imm_src_d = EXTEND_INVALID;
      alu_src_d = 1'b0;
      mem_write_d = 1'b0;
      result_src_d = 2'b00;
      branch_d = 1'b0;
      alu_op = 2'b10;
      jump_d = 1'b0;
    end
    B_TYPE : begin
      reg_write_d = 1'b0;
      imm_src_d = EXTEND_B;
      alu_src_d = 1'b0;
      mem_write_d = 1'b0;
      result_src_d = 2'bxx;
      branch_d = 1'b1;
      alu_op = 2'b01;
      jump_d = 1'b0;
    end
    I_TYPE : begin
      reg_write_d = 1'b1;
      imm_src_d = EXTEND_I;
      alu_src_d = 1'b1;
      mem_write_d = 1'b0;
      result_src_d = 2'b00;
      branch_d = 1'b0;
      alu_op = 2'b10;
      jump_d = 1'b0;
    end
    J_TYPE : begin
      reg_write_d = 1'b1;
      imm_src_d = EXTEND_J;
      alu_src_d = 1'bx;
      mem_write_d = 1'b0;
      result_src_d = 2'b10;
      branch_d = 1'b0;
      alu_op = 2'bxx;
      jump_d = 1'b1;
    end
    default : begin
      reg_write_d = 1'bx;
      imm_src_d = EXTEND_INVALID;
      alu_src_d = 1'bx;
      mem_write_d = 1'bx;
      result_src_d = 2'bx;
      branch_d = 1'bx;
      alu_op = 2'bxx;
      jump_d = 1'bx;
    end
  endcase
end

always_comb begin : ALU_DECODER
  case(alu_op)
    2'b00 : alu_control_d = ALU_ADD;
    2'b01 : alu_control_d = ALU_SUB;
    2'b10 : begin
      case(funct3)
        3'b000 : begin
          case ({op_raw[5], funct7[5]})
            2'b00 : alu_control_d = ALU_ADD;
            2'b01 : alu_control_d = ALU_ADD;
            2'b10 : alu_control_d = ALU_ADD;
            2'b11 : alu_control_d = ALU_SUB;
            default : alu_control_d = ALU_INVALID;
          endcase
        end
        3'b001 : alu_control_d = ALU_SLL;
        3'b010 : alu_control_d = ALU_SLT;
        3'b011 : alu_control_d = ALU_SLTU;
        3'b100 : alu_control_d = ALU_XOR;
        3'b101 : alu_control_d = ALU_SRA;
        3'b110 : alu_control_d = ALU_OR;
        3'b111 : alu_control_d = ALU_AND;
        default : alu_control_d = ALU_INVALID;
      endcase
    end
    default : alu_control_d = ALU_INVALID;
  endcase
end

// Hazard Unit ******************************************************
logic stall_f;
logic stall_d;
logic flush_d;
logic flush_e;
logic [1:0] forward_a_e;
logic [1:0] forward_b_e;
logic lw_stall;
always_comb begin : HAZARD_UNIT

  // Forward A
  // Forward From Memory stage
  if (((rs1_e == rd_m) & reg_write_m) & (rs1_e != 0))      forward_a_e = 2'b10;
  // Forward from Writeback stage
  else if (((rs1_e == rd_w) & reg_write_w) & (rs1_e != 0)) forward_a_e = 2'b01;
  // No forwarding
  else                                                     forward_a_e = 2'b00;
  
  // Forward B
  // Forward From Memory stage
  if (((rs2_e == rd_m) & reg_write_m) & (rs2_e != 0))      forward_b_e = 2'b10;
  // Forward from Writeback stage
  else if (((rs2_e == rd_w) & reg_write_w) & (rs2_e != 0)) forward_b_e = 2'b01;
  // No forwarding
  else                                                     forward_b_e = 2'b00;

  // Stall
  lw_stall = result_src_e[0] & ((rs1_d == rd_e) | (rs2_d == rd_e));
  stall_f = lw_stall;
  stall_d = lw_stall;
  flush_e = lw_stall;

  // Flush
  flush_d = pc_src_e;
  flush_e = lw_stall | pc_src_e;
end

// Fetch Stage ******************************************************
logic [31:0] pc_plus_4f;
always_comb begin : PCPLUS_4
  pc_plus_4f = PC + 4;
end

always_comb begin : PC_INPUT_MUX
  PC_next = pc_src_e ? pc_target_e : pc_plus_4f;
end

always_comb begin : INSTRUCTION_MEMORY
  instr_mem_addr = PC;
end

logic reg_d_ena;
logic reg_d_rst;
always_comb begin
  reg_d_ena = ~stall_d;
  reg_d_rst = flush_d;
end

logic [31:0] instruction;
register #(.N(32)) mem_rd_data_reg_d(
  .clk(clk), .ena(reg_d_ena), .rst(reg_d_rst), .d(instr_mem_rd_data), .q(instruction)
);

logic [31:0] pc_d;
register #(.N(32)) pc_f_reg_d(
  .clk(clk), .ena(reg_d_ena), .rst(reg_d_rst), .d(PC), .q(pc_d)
);

logic [31:0] pc_plus_4_d;
register #(.N(32)) pc_plus_4_reg_d(
  .clk(clk), .ena(reg_d_ena), .rst(reg_d_rst), .d(pc_plus_4f), .q(pc_plus_4_d)
);

// Decode Stage *****************************************************
logic [31:0] imm_ext_d;

always_comb begin : SIGN_EXTENDER
  case(imm_src_d)
    EXTEND_I : imm_ext_d = {{20{imm[31]}}, imm[31:20]};
    EXTEND_S : imm_ext_d = {{20{imm[31]}}, imm[31:25], imm[11:7]};
    EXTEND_B : imm_ext_d = {{20{imm[31]}}, imm[7], imm[30:25], imm[11:8], 1'b0};
    EXTEND_J : imm_ext_d = {{12{imm[31]}}, imm[19:12], imm[20], imm[30:21], 1'b0};
    EXTEND_INVALID : imm_ext_d = {32{1'bx}};
    default : imm_ext_d = {32{1'bx}};
  endcase
end

logic [31:0] imm;
logic [6:0] funct7;
logic [2:0] funct3;
logic [4:0] rd_d;
enum logic [3:0] {INVALID_OP, LW, S_TYPE, R_TYPE, B_TYPE, I_TYPE, J_TYPE} op;
logic [6:0] op_raw;
always_comb begin : REGISTER_DECODER
  // Added last 7 bits for convience in SIGN_EXTENDER
  imm = instruction[31:0];
  funct7 = instruction[31:25];
  rs2_d = instruction[24:20];
  rs1_d = instruction[19:15];
  funct3 = instruction[14:12];
  rd_d = instruction[11:7];
  op_raw = instruction[6:0];
  case(instruction[6:0])
    7'b0000011 : op = LW;
    7'b0100011 : op = S_TYPE;
    7'b0110011 : op = R_TYPE;
    7'b1100011 : op = B_TYPE;
    7'b0010011 : op = I_TYPE;
    7'b1101111 : op = J_TYPE;
    default : op = INVALID_OP;
  endcase
end

logic rst_reg_e;
always_comb begin
  rst_reg_e = rst | flush_e;
end
logic reg_write_e;
register #(.N(1)) reg_write_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(reg_write_d), .q(reg_write_e)
);

logic [1:0] result_src_e;
register #(.N(2)) result_src_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(result_src_d), .q(result_src_e)
);

logic mem_write_e;
register #(.N(1)) mem_write_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(mem_write_d), .q(mem_write_e)
);

logic jump_e;
register #(.N(1)) jump_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(jump_d), .q(jump_e)
);

logic branch_e;
register #(.N(1)) branch_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(branch_d), .q(branch_e)
);

alu_control_t alu_control_e;
register #(.N(4)) alu_control_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(alu_control_d), .q(alu_control_e)
);

logic alu_src_e;
register #(.N(1)) alu_src_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(alu_src_d), .q(alu_src_e)
);

logic [31:0] rd1_e;
register #(.N(32)) rd1_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(reg_data1), .q(rd1_e)
);

logic [31:0] rd2_e;
register #(.N(32)) rd2_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(reg_data2), .q(rd2_e)
);

logic [31:0] pc_e;
register #(.N(32)) pc_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(pc_d), .q(pc_e)
);

logic [4:0] rs1_e;
register #(.N(5)) rs1_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(rs1_d), .q(rs1_e)
);

logic [4:0] rs2_e;
register #(.N(5)) rs2_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(rs2_d), .q(rs2_e)
);

logic [4:0] rd_e;
register #(.N(5)) rd_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(rd_d), .q(rd_e)
);

logic [31:0] imm_ext_e;
register #(.N(32)) imm_ext_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(imm_ext_d), .q(imm_ext_e)
);

logic [31:0] pc_plus_4_e;
register #(.N(32)) pc_plus_4_reg_e(
  .clk(clk), .ena(1'b1), .rst(rst_reg_e), .d(pc_plus_4_d), .q(pc_plus_4_e)
);

// Execute Stage ****************************************************
logic [31:0] src_a_e;
logic [31:0] src_b_e;
logic [31:0] write_data_e;
always_comb begin : RD_MUXES
  case (forward_a_e)
    2'b00 : src_a_e = rd1_e;
    2'b01 : src_a_e = result_w;
    2'b10 : src_a_e = alu_result_m;
  endcase
  case (forward_b_e)
    2'b00 : write_data_e = rd2_e;
    2'b01 : write_data_e = result_w;
    2'b10 : write_data_e = alu_result_m;
  endcase
  case (alu_src_e)
    1'b0 : src_b_e = write_data_e;
    1'b1 : src_b_e = imm_ext_e;
  endcase
end

logic [31:0] pc_target_e;
always_comb begin : PC_TARGET_ADDER
  pc_target_e = pc_e + imm_ext_e;
end

logic reg_write_m;
register #(.N(1)) reg_write_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(reg_write_e), .q(reg_write_m)
);

logic [1:0] result_src_m;
register #(.N(2)) result_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(result_src_e), .q(result_src_m)
);

logic mem_write_m;
register #(.N(1)) mem_write_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(mem_write_e), .q(mem_write_m)
);

logic [31:0] alu_result_m;
register #(.N(32)) alu_result_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(alu_result), .q(alu_result_m)
);

logic [31:0] write_data_m;
register #(.N(32)) write_data_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(write_data_e), .q(write_data_m)
);

logic [4:0] rd_m;
register #(.N(5)) rd_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(rd_e), .q(rd_m)
);

logic [31:0] pc_plus_4_m;
register #(.N(32)) pc_plus_4_reg_m(
  .clk(clk), .ena(1'b1), .rst(rst), .d(pc_plus_4_e), .q(pc_plus_4_m)
);

// Memory Stage  ****************************************************
always_comb begin : DATA_MEMORY
  data_mem_addr = alu_result_m;
  data_mem_wr_data = write_data_m;
  data_mem_wr_ena = mem_write_m;
end
logic reg_write_w;
register #(.N(1)) reg_write_reg_w(
  .clk(clk), .ena(1'b1), .rst(rst), .d(reg_write_m), .q(reg_write_w)
);

logic [1:0] result_src_w;
register #(.N(2)) result_src_reg_w(
  .clk(clk), .ena(1'b1), .rst(rst), .d(result_src_m), .q(result_src_w)
);

logic [31:0] alu_result_w;
register #(.N(32)) alu_result_reg_w(
  .clk(clk), .ena(1'b1), .rst(rst), .d(alu_result_m), .q(alu_result_w)
);

logic [31:0] read_data_w;
register #(.N(32)) mem_rd_data2_reg_w(
  .clk(clk), .ena(1'b1), .rst(rst), .d(data_mem_rd_data), .q(read_data_w)
);

logic [4:0] rd_w;
register #(.N(5)) rd_reg_w(
  .clk(clk), .ena(1'b1), .rst(rst), .d(rd_m), .q(rd_w)
);

logic [31:0] pc_plus_4_w;
register #(.N(32)) pc_plus_4_reg_w(
  .clk(clk), .ena(1'b1), .rst(rst), .d(pc_plus_4_m), .q(pc_plus_4_w)
);

// Writeback Stage  *************************************************
logic [31:0] result_w;
always_comb begin : RESULT_MUX
  case (result_src_w)
    2'b00 : result_w = alu_result_w;
    2'b01 : result_w = read_data_w;
    2'b10 : result_w = pc_plus_4_w;
  endcase
end

endmodule
