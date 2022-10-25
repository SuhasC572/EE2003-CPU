module alu32(
    input [5:0] op,      
    input [31:0] rv1,    // First operand
    input [31:0] rv2,    // Second operand
    output [31:0] rvout  // Output 
);
    reg rvout;
    always @(rv1 or rv2 or op) begin
      case(op[3:1])                                                                                       //Blocking statements below because of combinational ALU
        3'b000: rvout = (2'b11 == {op[4],op[0]}) ?  rv1 - rv2 : rv1 + rv2;                                //ADD / SUB    -add
        3'b001: rvout = $signed(rv1) << $signed(rv2[4:0]);                                                //SLL   -shift logical left
        3'b010: rvout = ($signed(rv2) > $signed(rv1))?32'h1:32'h0;                                        //SLT   -if rv2>rv1, else 0 signed no. immediate
        3'b011: rvout = (rv2>rv1)?32'h1:32'h0;                                                            //SLTU  -if rv2>rv1, else 0 unsigned no. immediate
        3'b100: rvout = rv1 ^ rv2;                                                                        //XOR   -bitwise
        3'b101: rvout = (op[0]) ? $signed(rv1) >>> $signed(rv2[4:0]):$signed(rv1) >> $signed(rv2[4:0]);   // SRA / SRL
        3'b110: rvout = rv1 | rv2;                                                                        //OR    -bitwise
        3'b111: rvout = rv1 & rv2;                                                                        //AND   -bitwise
      endcase
    end

endmodule
