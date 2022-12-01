
module control
import rv32i_types::*; /* Import types defined in rv32i_types.sv */
(
    input clk,
    input rst,
    input rv32i_opcode opcode, // datapath to control
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic br_en,
    input logic [4:0] rs1,
    input logic [4:0] rs2,
    input logic mem_resp, // memory to control
    input logic [1:0] mem_align,
    output logic load_pc, // control to datapath
    output logic load_ir,
    output logic load_regfile,
    output logic load_mar,
    output logic load_mdr,
    output logic load_data_out, 
    output pcmux::pcmux_sel_t pcmux_sel,
    output branch_funct3_t cmpop,
    output alumux::alumux1_sel_t alumux1_sel,
    output alumux::alumux2_sel_t alumux2_sel,
    output regfilemux::regfilemux_sel_t regfilemux_sel,
    output marmux::marmux_sel_t marmux_sel,
    output cmpmux::cmpmux_sel_t cmpmux_sel,
    output alu_ops aluop,
    // output logic [1:0] mem_data_mux_sel,

    output logic mem_read, // control to memory
    output logic mem_write, 
    output logic [3:0] mem_byte_enable

);

/***************** USED BY RVFIMON --- ONLY MODIFY WHEN TOLD *****************/
logic trap;
logic [4:0] rs1_addr, rs2_addr;
logic [3:0] rmask, wmask;

branch_funct3_t branch_funct3;
store_funct3_t store_funct3;
load_funct3_t load_funct3;
arith_funct3_t arith_funct3;

assign arith_funct3 = arith_funct3_t'(funct3);
assign branch_funct3 = branch_funct3_t'(funct3);
assign load_funct3 = load_funct3_t'(funct3);
assign store_funct3 = store_funct3_t'(funct3);
assign rs1_addr = rs1;
assign rs2_addr = rs2;

always_comb
begin : trap_check
    trap = '0;
    rmask = '0;
    wmask = '0;

    case (opcode)
        op_lui, op_auipc, op_imm, op_reg, op_jal, op_jalr:;

        op_br: begin
            case (branch_funct3)
                beq, bne, blt, bge, bltu, bgeu:;
                default: trap = '1;
            endcase
        end

        op_load: begin
            case (load_funct3)
                lw: rmask = 4'b1111;
                lh, lhu: rmask = mem_byte_enable /* Modify for MP1 Final */ ;
                lb, lbu: rmask = mem_byte_enable /* Modify for MP1 Final */ ;
                default: trap = '1;
            endcase
        end

        op_store: begin
            case (store_funct3)
                sw: wmask = 4'b1111;
                sh: wmask = mem_byte_enable /* Modify for MP1 Final */ ;
                sb: wmask = mem_byte_enable /* Modify for MP1 Final */ ;
                default: trap = '1;
            endcase
        end

        default: trap = '1;
    endcase
end
/*****************************************************************************/

enum int unsigned {
    /* List of states */
    FETCH1, FETCH2, FETCH3, 
    DECODE, 
    IMM, LUI, AUIPC, BR, CALC_ADDR, REG,
    LD1, LD2, 
    ST1, ST2, 
    JAL, JALR,
    CALC_W
} state, next_state;

/************************* Function Definitions *******************************/
/**
 *  You do not need to use these functions, but it can be nice to encapsulate
 *  behavior in such a way.  For example, if you use the `loadRegfile`
 *  function, then you only need to ensure that you set the load_regfile bit
 *  to 1'b1 in one place, rather than in many.
 *
 *  SystemVerilog functions must take zero "simulation time" (as opposed to 
 *  tasks).  Thus, they are generally synthesizable, and appropraite
 *  for design code.  Arguments to functions are, by default, input.  But
 *  may be passed as outputs, inouts, or by reference using the `ref` keyword.
**/

/**
 *  Rather than filling up an always_block with a whole bunch of default values,
 *  set the default values for controller output signals in this function,
 *   and then call it at the beginning of your always_comb block.
**/
function void set_defaults();
    load_pc = '0;
    load_ir = '0;
    load_regfile = '0;
    load_mar = '0;
    load_mdr = '0;
    load_data_out = '0;
    aluop = alu_ops'(funct3);
    mem_read = '0;
    mem_write = '0;
    mem_byte_enable = 4'b1111;
    // mem_data_mux_sel = 2'b00;
    pcmux_sel = pcmux::pc_plus4;
    cmpop = branch_funct3_t'(funct3);
    alumux1_sel = alumux::rs1_out;
    alumux2_sel = alumux::i_imm;
    regfilemux_sel = regfilemux::alu_out;
    marmux_sel = marmux::pc_out;
    cmpmux_sel = cmpmux::rs2_out;
endfunction

/**
 *  Use the next several functions to set the signals needed to
 *  load various registers
**/
function void loadPC(pcmux::pcmux_sel_t sel);
    load_pc = 1'b1;
    pcmux_sel = sel;
endfunction

function void loadRegfile(regfilemux::regfilemux_sel_t sel);
    load_regfile = 1'b1;
    regfilemux_sel = sel;
endfunction

function void loadMAR(marmux::marmux_sel_t sel);
    load_mar = 1'b1;
    marmux_sel = sel;
endfunction

function void loadMDR();
    load_mdr = 1'b1;
    mem_read = 1'b1;
endfunction

function void loadMemData();
    load_data_out = 1'b1;
    setByteEnable(funct3, 1'b0, mem_align);
    // case(store_funct3_t'(funct3))
    //     sw: mem_data_mux_sel = 2'b00;
    //     sh: mem_data_mux_sel = 2'b01;
    //     sb: mem_data_mux_sel = 2'b10;
    // endcase
endfunction

function void setALU(alumux::alumux1_sel_t sel1, alumux::alumux2_sel_t sel2, logic setop, alu_ops op);
    /* Student code here */
    alumux1_sel = sel1;
    alumux2_sel = sel2;
    if (setop)
        aluop = op; // else default value
endfunction

function automatic void setCMP(cmpmux::cmpmux_sel_t sel, logic setop, branch_funct3_t op);
    cmpmux_sel = sel;
    if (setop)
        cmpop = op;
endfunction

function void setLUI();
    loadRegfile(regfilemux::u_imm); // load_regfile <- 1
                                    // regfilemux_sel <- 2
    loadPC(pcmux::pc_plus4);   // PC <- PC + 4
endfunction

function void setAUIPC();
    setALU(alumux::pc_out, alumux::u_imm, 1'b1, alu_add);
    loadRegfile(regfilemux::alu_out); // RD <- PC + U_IMM
    loadPC(pcmux::pc_plus4);   // PC <- PC + 4
endfunction

function void setBR(); 
    loadPC(pcmux::pcmux_sel_t'(br_en)); // PC <- PC + (br_en ? b_imm : 4)
    setALU(alumux::pc_out, alumux::b_imm, 1'b1, alu_add);
    setCMP(cmpmux::rs2_out, 1'b1, branch_funct3_t'(funct3));
endfunction

// calc_addr
function void setCALC_ADDR();
    loadMAR(marmux::alu_out);
    if (opcode == op_load) begin 
        setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
    end
    else if (opcode == op_store) begin 
        setALU(alumux::rs1_out, alumux::s_imm, 1'b1, alu_add);
        loadMemData();
    end
endfunction

// st1
function void setST1(); 
    mem_write = 1'b1;
    setByteEnable(funct3, 1'b0, mem_align);
endfunction

// st2
function void setST2(); 
    loadPC(pcmux::pc_plus4);
    setByteEnable(funct3, 1'b0, mem_align);
endfunction

// ld
function void setLD1(load_funct3_t op); 
    loadMDR();
    setByteEnable(funct3, 1'b1, mem_align);
endfunction

function void setLD2(load_funct3_t op); 
    case(op) // rd <- MDR
        lb: loadRegfile(regfilemux::lb);
        lh: loadRegfile(regfilemux::lh);
        lw: loadRegfile(regfilemux::lw);
        lbu: loadRegfile(regfilemux::lbu);
        lhu: loadRegfile(regfilemux::lhu);            
    endcase
    loadPC(pcmux::pc_plus4);   // PC <- PC + 4
    setByteEnable(funct3, 1'b1, mem_align);
endfunction

// imm
function void setIMM(arith_funct3_t op);
    case(op) 
        slt: begin  // slti
            setCMP(cmpmux::i_imm, 1'b1, blt);
            loadRegfile(regfilemux::br_en); 
        end
        sltu: begin  // sltiu
            setCMP(cmpmux::i_imm, 1'b1, bltu);
            loadRegfile(regfilemux::br_en); 
        end
        sr: begin // srai & srli (same opcode, diff func7)
            loadRegfile(regfilemux::alu_out); // default for regfilemux_sel
            if (funct7) // if func7 7'b0100000, srai
                setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_sra);
            else 
                setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_srl);
        end
        default: begin 
            loadRegfile(regfilemux::alu_out); // default for regfilemux_sel
            setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_ops'(op));
        end
    endcase
    loadPC(pcmux::pc_plus4); 
endfunction

function void setREG(arith_funct3_t op);
    case(op) 
        add: begin  // add & sub
            loadRegfile(regfilemux::alu_out); 
            if (funct7) // if func7 7'b0100000, subtract
                setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sub);
            else 
                setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_add);
        end
        sr: begin // srai & srli (same opcode, diff func7)
            loadRegfile(regfilemux::alu_out); 
            if (funct7) // if func7 7'b0100000, srai
                setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sra);
            else 
                setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_srl);
        end
        slt: begin 
            setCMP(cmpmux::rs2_out, 1'b1, blt);
            loadRegfile(regfilemux::br_en); 
        end
        sltu: begin 
            setCMP(cmpmux::rs2_out, 1'b1, bltu);
            loadRegfile(regfilemux::br_en); 
        end
        default: begin 
            setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_ops'(op));
            loadRegfile(regfilemux::alu_out); 
        end
    endcase
    loadPC(pcmux::pc_plus4); 
endfunction

// load = 1 if load_funct, else store_funct
function void setByteEnable(logic [2:0] op, logic load, logic [1:0] mem_align);
    mem_byte_enable = 4'b1111;
    if (load) begin
        case(load_funct3_t'(op))
            lw: mem_byte_enable = 4'b1111;
            lh, lhu: begin 
                case(mem_align) // alignment for half words
                    2'b00: mem_byte_enable = 4'b0011;
                    2'b10: mem_byte_enable = 4'b1100;
                endcase
            end
            lb, lbu: begin // alignment for each possible byte
                case(mem_align)
                    2'b00: mem_byte_enable = 4'b0001;
                    2'b01: mem_byte_enable = 4'b0010;
                    2'b10: mem_byte_enable = 4'b0100;
                    2'b11: mem_byte_enable = 4'b1000;
                endcase
            end
        endcase
    end
    else begin 
        case(store_funct3_t'(op))
            sw: mem_byte_enable = 4'b1111;
            sh: begin 
                case(mem_align) // alignment for half words
                    2'b00: mem_byte_enable = 4'b0011;
                    2'b10: mem_byte_enable = 4'b1100;
                endcase
            end
            sb: begin // alignment for each possible byte
                case(mem_align)
                    2'b00: mem_byte_enable = 4'b0001;
                    2'b01: mem_byte_enable = 4'b0010;
                    2'b10: mem_byte_enable = 4'b0100;
                    2'b11: mem_byte_enable = 4'b1000;
                endcase
            end
        endcase
    end
endfunction

function void setJAL(); 
    setALU(alumux::pc_out, alumux::j_imm, 1'b1, alu_add);
    loadPC(pcmux::alu_out);   // PC <- PC + j_imm
    loadRegfile(regfilemux::pc_plus4);
endfunction

function void setJALR(); 
    setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
    loadPC(pcmux::alu_mod2);   // PC <- rs1 + i_imm
    loadRegfile(regfilemux::pc_plus4);
endfunction

/*****************************************************************************/

always_comb
begin : state_actions
    /* Default output assignments */
    set_defaults();
    /* Actions for each state */
    unique case (state)
        FETCH1: loadMAR(marmux::pc_out);  // MAR <- PC
        FETCH2: loadMDR();      // MDR <- M[MAR]
        FETCH3: load_ir = 1'b1; // IR <- MDR
        DECODE: ; 
        IMM: setIMM(arith_funct3_t'(funct3));
        LUI: setLUI();
        AUIPC: setAUIPC();
        BR: setBR();
        REG: setREG(arith_funct3_t'(funct3)); 
        CALC_ADDR: setCALC_ADDR();
        CALC_W: loadMemData();
        LD1: setLD1(load_funct3_t'(funct3));
        LD2: setLD2(load_funct3_t'(funct3));
        ST1: setST1();
        ST2: setST2();
        JAL: setJAL();
        JALR: setJALR();
    endcase
end

always_comb
begin : next_state_logic
    /* Next state information and conditions (if any)
     * for transitioning between states */
    case(state)
        FETCH1: next_state = FETCH2;

        FETCH2: begin 
            if (mem_resp)
                next_state = FETCH3; 
            else
                next_state = FETCH2;
        end

        FETCH3: next_state = DECODE;

        DECODE: begin 
            unique case(opcode)
                op_lui: next_state = LUI; //load upper immediate (U type)
                op_auipc: next_state = AUIPC; //add upper immediate PC (U type)
                op_jal: next_state = JAL; //jump and link (J type)
                op_jalr: next_state = JALR; //jump and link register (I type)
                op_br: next_state = BR; //branch (B type)
                op_load, op_store: next_state = CALC_ADDR; //store (S type)
                op_imm: next_state = IMM; //arith ops with register/immediate operands (I type)
                op_reg: next_state = REG; //arith ops with register operands (R type)
                default: next_state = FETCH1;
            endcase
        end

        IMM, LUI, AUIPC, BR, LD2, ST2, REG, JAL, JALR: next_state = FETCH1; 
        
        CALC_ADDR: begin 
            unique case(opcode)
                op_load: next_state = LD1;
                op_store: next_state = CALC_W;
                default: next_state = FETCH1;
            endcase
        end

        // remove if not useful
        CALC_W: next_state = ST1;

        LD1: begin 
            if (mem_resp)
                next_state = LD2;
            else 
                next_state = LD1;
        end

        ST1: begin 
            if (mem_resp)
                next_state = ST2;
            else 
                next_state = ST1;
        end

    endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
    /* Assignment of next state on clock edge */
    if (rst == 1'b1) // rst signal
        state <= FETCH1;
    else 
        state <= next_state;
end

endmodule : control
