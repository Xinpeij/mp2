`define BAD_MUX_SEL $display("Illegal mux select")


module datapath
import rv32i_types::*;
(
    input clk,
    input rst,
    input logic load_pc, // control to datapath
    input logic load_ir,
    input logic load_regfile,
    input logic load_mar,
    input logic load_mdr,
    input logic load_data_out,
    input pcmux::pcmux_sel_t pcmux_sel,
    input branch_funct3_t cmpop,
    input alumux::alumux1_sel_t alumux1_sel,
    input alumux::alumux2_sel_t alumux2_sel,
    input regfilemux::regfilemux_sel_t regfilemux_sel,
    input marmux::marmux_sel_t marmux_sel,
    input cmpmux::cmpmux_sel_t cmpmux_sel,
    input alu_ops aluop,
    input logic [3:0] mem_byte_enable,

    input rv32i_word mem_rdata, // memory to datapath
    output rv32i_word mem_wdata, // datapath to memory - signal used by RVFI Monitor
    output rv32i_word mem_address,

    output rv32i_opcode opcode, // datapath to control
    output [2:0] funct3,
    output [6:0] funct7,
    output logic br_en,
    output [4:0] rs1,
    output [4:0] rs2, 
    output logic [1:0] mem_align 
);

/******************* Signals Needed for RVFI Monitor *************************/
rv32i_word pcmux_out;
rv32i_word mdrreg_out;
/*****************************************************************************/
rv32i_word regfilemux_out;
rv32i_word marmux_out;
rv32i_word alumux1_out;
rv32i_word alumux2_out;
rv32i_word cmpmux_out;
rv32i_word alu_out;
rv32i_word pc_out;
logic [4:0] rd;
logic [31:0] i_imm;
logic [31:0] s_imm;
logic [31:0] b_imm;
logic [31:0] u_imm;
logic [31:0] j_imm;
rv32i_word rs1_out;
rv32i_word rs2_out;
rv32i_word mar_out;
rv32i_word mem_data;

typedef enum bit [1:0] {
    sw_mem = 2'b00,
    sh_mem = 2'b01,
    sb_mem = 2'b10
} mem_data_mux_sel_t;

/***************************** Registers *************************************/
// Keep Instruction register named `IR` for RVFI Monitor
ir IR(
    .clk    (clk),
    .rst    (rst),
    .load   (load_ir),
    .in     (mdrreg_out),
    .*
);

register MDR(
    .clk  (clk),
    .rst  (rst),
    .load (load_mdr),
    .in   (mem_rdata),
    .out  (mdrreg_out)
);

register MAR(
    .clk  (clk),
    .rst  (rst),
    .load (load_mar),
    .in   (marmux_out),
    .out  (mar_out)
);

pc_register PC(
    .clk    (clk),
    .rst    (rst),
    .load   (load_pc),
    .in     (pcmux_out),
    .out    (pc_out)
);

regfile regfile(
    .clk    (clk),
    .rst    (rst),
    .load   (load_regfile),
    .in     (regfilemux_out),
    .src_a  (rs1), .src_b  (rs2), .dest   (rd),
    .reg_a  (rs1_out), .reg_b (rs2_out)
);

register mem_data_out(
    .clk  (clk),
    .rst  (rst),
    .load (load_data_out),
    .in   (mem_data),
    .out  (mem_wdata)
);


/*****************************************************************************/

/******************************* ALU and CMP *********************************/
alu ALU(
    .aluop(aluop),
    .a(alumux1_out), .b(alumux2_out),
    .f(alu_out)
);

cmp CMP(
    .funct3(cmpop),
    .a(rs1_out), .b(cmpmux_out),
    .br_en(br_en)
);

/*****************************************************************************/

/******************************** Muxes **************************************/
always_comb begin : MUXES
    // We provide one (incomplete) example of a mux instantiated using
    // a case statement.  Using enumerated types rather than bit vectors
    // provides compile time type safety.  Defensive programming is extremely
    // useful in SystemVerilog.

    // kinda hack-y way to do this since they said we shouldn't expect 
    // non-aligned addresses 
    mem_address = mar_out & 32'hfffffffc; // must be 4 byte aligned
    mem_align = mar_out[1:0]; // modulo 4 for checking alignment
    // shift + mask bits
    mem_data = (rs2_out  << 8*mem_align) & ({{8{mem_byte_enable[3]}}, {8{mem_byte_enable[2]}}, {8{mem_byte_enable[1]}}, {8{mem_byte_enable[0]}}});
    
    unique case (pcmux_sel)
        pcmux::pc_plus4: pcmux_out = pc_out + 4;
        pcmux::alu_out: pcmux_out = alu_out; 
        pcmux::alu_mod2: pcmux_out = alu_out & 32'hfffffffe; // jal
        default: `BAD_MUX_SEL;
    endcase

    unique case (regfilemux_sel)
        regfilemux::alu_out: regfilemux_out = alu_out; 
        regfilemux::br_en: regfilemux_out = {31'b0, br_en};
        regfilemux::u_imm: regfilemux_out = u_imm;
        regfilemux::lw: regfilemux_out = mdrreg_out;
        regfilemux::pc_plus4: regfilemux_out = pc_out + 4;
        regfilemux::lb: begin 
            case(mem_align)
                2'b00: regfilemux_out = {{25{mdrreg_out[7]}}, mdrreg_out[6:0]};
                2'b01: regfilemux_out = {{25{mdrreg_out[15]}}, mdrreg_out[14:8]};
                2'b10: regfilemux_out = {{25{mdrreg_out[23]}}, mdrreg_out[22:16]};
                2'b11: regfilemux_out = {{25{mdrreg_out[31]}}, mdrreg_out[30:24]};
            endcase
        end
        regfilemux::lbu: begin 
            case(mem_align)
                2'b00: regfilemux_out = {24'b0, mdrreg_out[7:0]};
                2'b01: regfilemux_out = {24'b0, mdrreg_out[15:8]};
                2'b10: regfilemux_out = {24'b0, mdrreg_out[23:16]};
                2'b11: regfilemux_out = {24'b0, mdrreg_out[31:24]};
            endcase
        end
        regfilemux::lh: begin 
            case(mem_align)
                2'b00: regfilemux_out = {{17{mdrreg_out[15]}}, mdrreg_out[14:0]};
                2'b10: regfilemux_out = {{17{mdrreg_out[31]}}, mdrreg_out[30:16]};
            endcase
        end
        regfilemux::lhu: begin 
            case(mem_align)
                2'b00: regfilemux_out = {16'b0, mdrreg_out[15:0]};
                2'b10: regfilemux_out = {16'b0, mdrreg_out[31:16]};
            endcase
        end
        default: `BAD_MUX_SEL;
    endcase

    unique case (marmux_sel)
        marmux::pc_out: marmux_out = pc_out;
        marmux::alu_out: marmux_out = alu_out;
        default: `BAD_MUX_SEL;
    endcase

    // ALUMUX1
    unique case (alumux1_sel)
        alumux::pc_out: alumux1_out = pc_out;
        alumux::rs1_out: alumux1_out = rs1_out; 
        default: `BAD_MUX_SEL;
    endcase

    // ALUMUX2
    unique case (alumux2_sel)
        alumux::i_imm: alumux2_out = i_imm;
        alumux::u_imm: alumux2_out = u_imm;
        alumux::b_imm: alumux2_out = b_imm;
        alumux::s_imm: alumux2_out = s_imm;
        alumux::j_imm: alumux2_out = j_imm;
        alumux::rs2_out: alumux2_out = rs2_out;
        default: `BAD_MUX_SEL;
    endcase

    // CMPMUX
    unique case (cmpmux_sel)
        cmpmux::rs2_out: cmpmux_out = rs2_out;
        cmpmux::i_imm: cmpmux_out = i_imm;
    endcase

end

/*****************************************************************************/
endmodule : datapath
