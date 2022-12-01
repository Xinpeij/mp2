
module cmp
import rv32i_types::*;
(
    input [2:0] funct3, // cmpop
    input [31:0] a, // rs1
    input [31:0] b, // rs2 or i_imm
    output logic br_en
);

always_comb
begin
    case(funct3)
        beq: br_en = (a == b) ? 1'b1 : 1'b0; // true if equal
        bne: br_en = (a != b) ? 1'b1 : 1'b0;
        blt: br_en = $signed(a) < $signed(b) ? 1'b1 : 1'b0;
        bge: br_en = $signed(a) >= $signed(b) ? 1'b1 : 1'b0;
        bltu: br_en = (a < b) ? 1'b1 : 1'b0;
        bgeu: br_en = (a >= b) ? 1'b1 : 1'b0;
    endcase
end

endmodule : cmp
