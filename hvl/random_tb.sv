
/**
 * Generates constrained random vectors with which to drive the DUT.
 * Recommended usage is to test arithmetic and comparator functionality,
 * as well as branches.
 *
 * Randomly testing load/stores will require building a memory model,
 * which you can do using a SystemVerilog associative array:
 *     logic[7:0] byte_addressable_mem [logic[31:0]]
 *   is an associative array with value type logic[7:0] and
 *   key type logic[31:0].
 * See IEEE 1800-2017 Clause 7.8
**/
module random_tb
import rv32i_types::*;
(
    tb_itf.tb itf,
    tb_itf.mem mem_itf
);

/**
 * SystemVerilog classes can be defined inside modules, in which case
 *   their usage scope is constrained to that module
 * RandomInst generates constrained random test vectors for your
 * rv32i DUT.
 * As is, RandomInst only supports generation of op_imm opcode instructions.
 * You are highly encouraged to expand its functionality.
**/
class RandomInst;
    rv32i_reg reg_range[$];
    arith_funct3_t arith3_range[$];
    load_funct3_t load3_range[$];
    store_funct3_t store3_range[$];

    /** Constructor **/
    function new();
        arith_funct3_t af3;
        load_funct3_t lf3;
        store_funct3_t sf3;

        af3 = af3.first;
        lf3 = lf3.first;
        sf3 = sf3.first;

        for (int i = 0; i < 32; ++i)
            reg_range.push_back(i);
        do begin
            arith3_range.push_back(af3);
            af3 = af3.next;
        end while (af3 != af3.first);

        do begin
            load3_range.push_back(lf3);
            lf3 = lf3.next;
        end while (lf3 != lf3.first);

        do begin
            store3_range.push_back(sf3);
            sf3 = sf3.next;
        end while (sf3 != sf3.first);

    endfunction

    function rv32i_word immediate(
        ref rv32i_reg rd_range[$] = reg_range,
        ref arith_funct3_t funct3_range[$] = arith3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [31:20] i_imm;
                rv32i_reg rs1;
                logic [2:0] funct3;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;

        word.rvword = '0;
        word.i_word.opcode = op_imm;

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));

        // set funct3
        do begin
            word.i_word.funct3 = $urandom();
        end while (!(word.i_word.funct3 inside {funct3_range}));

        // set rs1
        do begin
            word.i_word.rs1 = $urandom();
        end while (!(word.i_word.rs1 inside {rs1_range}));

        // set immediate value
        word.i_word.i_imm = $urandom();

        return word.rvword;
    endfunction

    function rv32i_word load(
        ref rv32i_reg rd_range[$] = reg_range,
        ref load_funct3_t funct3_range[$] = load3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [11:0] i_imm;
                rv32i_reg rs1;
                logic [2:0] funct3;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;
        int modulus = 4;

        word.rvword = '0;
        word.i_word.opcode = op_load;

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));

        // set funct3
        // word.i_word.funct3 = 3'b010; // lw
        do begin
            word.i_word.funct3 = $urandom();
        end while (!(word.i_word.funct3 inside {funct3_range}));

        // set rs1
        do begin
            word.i_word.rs1 = $urandom();
        end while (!(word.i_word.rs1 inside {rs1_range}));

        // set immediate value
        word.i_word.i_imm = $urandom();  
        // make sure immediate + rs1 divisible by 4
        case(word.i_word.funct3)
            lb, lbu: modulus = 1; // 1-aligned
            lh, lhu: modulus = 2; // 2-aligned
            lw: modulus = 4; // 4-aligned
        endcase

        while ((itf.registers[word.i_word.rs1] + word.i_word.i_imm) % modulus != 0) begin
            word.i_word.i_imm = word.i_word.i_imm + 1;
        end 

        return word.rvword;
    endfunction

    function rv32i_word store(
        ref rv32i_reg rd_range[$] = reg_range,
        ref store_funct3_t funct3_range[$] = store3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [11:5] s_imm11_5;
                rv32i_reg rs2;
                rv32i_reg rs1;
                logic [2:0] funct3;
                logic [4:0] s_imm4_0;
                rv32i_opcode opcode;
            } i_word;
        } word;
        logic [11:0] s_imm;
        int modulus;
        word.rvword = '0;
        word.i_word.opcode = op_store;

        // set rs2
        do begin
            word.i_word.rs2 = $urandom();
        end while (!(word.i_word.rs2 inside {rs1_range}));
        
        // set rs1
        do begin
            word.i_word.rs1 = $urandom();
        end while (!(word.i_word.rs1 inside {rs1_range}));

        // set funct3
        word.i_word.funct3 = 3'b000; // sw
        // do begin
        //     word.i_word.funct3 = $urandom();
        // end while (!(word.i_word.funct3 inside {funct3_range}));

        // set immediate value
        s_imm = $urandom();
        case(word.i_word.funct3)
            sb: modulus = 1; // 1-aligned
            sh: modulus = 2; // 2-aligned
            sw: modulus = 4; // 4-aligned
        endcase
        // make sure immediate + rs1 divisible by 4
        while ((itf.registers[word.i_word.rs1] + s_imm) % modulus != 0) begin
            s_imm = s_imm + 1;
        end 
        word.i_word.s_imm11_5 = s_imm[11:5];
        word.i_word.s_imm4_0 = s_imm[4:0];

        return word.rvword;
    endfunction

    function rv32i_word register(
        ref rv32i_reg rd_range[$] = reg_range,
        ref arith_funct3_t funct3_range[$] = arith3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [6:0] cons;
                rv32i_reg rs2;
                rv32i_reg rs1;
                logic [2:0] funct3;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;

        word.rvword = '0;
        word.i_word.opcode = op_reg;

        // set rs2
        do begin
            word.i_word.rs2 = $urandom();
        end while (!(word.i_word.rs2 inside {rs1_range}));
        
        // set rs1
        do begin
            word.i_word.rs1 = $urandom();
        end while (!(word.i_word.rs1 inside {rs1_range}));

        // set funct3
        do begin
            word.i_word.funct3 = $urandom();
        end while (!(word.i_word.funct3 inside {funct3_range}));

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));

        // set constant value
        word.i_word.cons = $urandom() & 7'b0100000;

        return word.rvword;
    endfunction

    function rv32i_word br(
        ref rv32i_reg rd_range[$] = reg_range,
        ref arith_funct3_t funct3_range[$] = arith3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        logic [12:2] b_imm;
        union {
            rv32i_word rvword;
            struct packed {
                logic b12;
                logic [10:5] b10_5;
                rv32i_reg rs2;
                rv32i_reg rs1;
                logic [2:0] funct3;
                logic [4:1] b4_1;
                logic b11;
                rv32i_opcode opcode;
            } i_word;
        } word;

        word.rvword = '0;
        word.i_word.opcode = op_br;

        // set funct3
        do begin
            word.i_word.funct3 = $urandom();
        end while (!(word.i_word.funct3 inside {funct3_range}));

        // set rs1
        do begin
            word.i_word.rs1 = $urandom();
        end while (!(word.i_word.rs1 inside {rs1_range}));

        // set rs1
        do begin
            word.i_word.rs2 = $urandom();
        end while (!(word.i_word.rs2 inside {rs1_range}));

        // set immediate value
        b_imm = 11'($urandom());
        word.i_word.b12 = b_imm[12];
        word.i_word.b11 = b_imm[11];
        word.i_word.b10_5 = b_imm[10:5];
        word.i_word.b4_1 = {b_imm[4:2], 1'b0};

        return word.rvword;
    endfunction

    function rv32i_word lui(
        ref rv32i_reg rd_range[$] = reg_range,
        ref arith_funct3_t funct3_range[$] = arith3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [31:12] i_imm;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;

        word.rvword = '0;
        word.i_word.opcode = op_lui;

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));

        // set immediate value
        word.i_word.i_imm = $urandom;

        return word.rvword;
    endfunction

    function rv32i_word auipc(
        ref rv32i_reg rd_range[$] = reg_range,
        ref arith_funct3_t funct3_range[$] = arith3_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [31:12] i_imm;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;

        word.rvword = '0;
        word.i_word.opcode = op_auipc;

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));

        // set immediate value
        word.i_word.i_imm = $urandom();

        return word.rvword;
    endfunction

    function rv32i_word jal(
        ref rv32i_reg rd_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic j20;
                logic [10:1] j10_1;
                logic j11;
                logic [19:12] j19_12;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;

        int j_imm;
        word.rvword = '0;
        word.i_word.opcode = op_jal;

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));

        // set immediate value
        j_imm = $urandom();  
        while ((dut.datapath.pc_out + j_imm) % 4 != 0) begin
            j_imm = j_imm + 1;
        end 
        word.i_word.j20 = j_imm[20];
        word.i_word.j10_1 = j_imm[10:1];
        word.i_word.j11 = j_imm[11];
        word.i_word.j19_12 = j_imm[19:12];

        return word.rvword;
    endfunction

    function rv32i_word jalr(
        ref rv32i_reg rd_range[$] = reg_range,
        ref rv32i_reg rs1_range[$] = reg_range
    );
        union {
            rv32i_word rvword;
            struct packed {
                logic [11:0] i_imm;
                rv32i_reg rs1;
                logic [2:0] funct3;
                logic [4:0] rd;
                rv32i_opcode opcode;
            } i_word;
        } word;

        word.rvword = '0;
        word.i_word.opcode = op_jalr;

        // Set rd register
        do begin
            word.i_word.rd = $urandom();
        end while (!(word.i_word.rd inside {rd_range}));
        
        // set funct3
        word.i_word.funct3 = 3'b000;

        // Set rd register
        do begin
            word.i_word.rs1 = $urandom();
        end while (!(word.i_word.rs1 inside {rs1_range}));

        // set immediate value
        word.i_word.i_imm = $urandom();  
        while ((itf.registers[word.i_word.rs1] + word.i_word.i_imm) % 4 != 0) begin
            word.i_word.i_imm = word.i_word.i_imm + 1;
        end 

        return word.rvword;
    endfunction

endclass

RandomInst generator = new();

task immediate_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting Immediate Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.immediate();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing Immediate Tests");
endtask

task register_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting Register Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.register();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing Register Tests");
endtask

task load_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting Load Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read); // load lw instruction
        mem_itf.mcb.rdata <= generator.load();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;

        @(mem_itf.mcb iff mem_itf.mcb.read); // memory response
        mem_itf.mcb.rdata <= 32'hecebcafe;
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;    end
    $display("Finishing Load Tests");
endtask

task store_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting Store Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read); // load lw instruction
        mem_itf.mcb.rdata <= generator.store();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;

        @(mem_itf.mcb iff mem_itf.mcb.write); // memory response
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;    end
    $display("Finishing Store Tests");
endtask

task br_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting Branch Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.br();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing Branch Tests");
endtask

task lui_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting LUI Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.lui();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing LUI Tests");
endtask

task jal_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting JAL Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.jal();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing JAL Tests");
endtask

task jalr_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting JALR Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.jalr();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing JALR Tests");
endtask

task auipc_tests(input int count);
    @(posedge itf.clk iff itf.rst == 1'b0)
    $display("Starting AUIPC Tests");
    repeat (count) begin
        @(mem_itf.mcb iff mem_itf.mcb.read);
        mem_itf.mcb.rdata <= generator.auipc();
        mem_itf.mcb.resp <= 1;
        @(mem_itf.mcb) mem_itf.mcb.resp <= 1'b0;
    end
    $display("Finishing AUIPC Tests");
endtask

initial begin
    load_tests(100);
    store_tests(100); 
    jal_tests(100);
    jalr_tests(100);
    br_tests(100);
    auipc_tests(100);
    immediate_tests(100); 
    register_tests(100);
    lui_tests(100);
    
    @(posedge itf.clk);
    $finish;
end

endmodule : random_tb