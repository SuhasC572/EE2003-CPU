module cpu (
    input clk,
    input reset,
    output [31:0] iaddr,
    input [31:0] idata,
    output [31:0] daddr,
    input [31:0] drdata,
    output [31:0] dwdata,
    output [3:0] dwe
);
reg [31:0] iaddr;
wire  memread;
wire  Memtoreg;
wire  [5:0] ALUop;
wire alusrc;
wire regwrite;
wire [3:0] memwrite;
wire [31:0] regwdata;
wire [31:0] datawire1;
wire [31:0] datawire2;
wire [31:0] imm;
wire [6:0] opcode;
wire [2:0] var3;
integer i;
wire [2:0] branch;

// ALU - Wires
wire [31:0] alu_out;

// ldata - Wires
wire [31:0] ldata;

assign var3 = idata[14:12];
assign opcode = idata[6:0];


assign branch = (opcode == 7'b1100011 && (var3 == 3'b000 || var3 == 3'b101 || var3 == 3'b111))? 3'b010: // Branch if Zero
					      (opcode == 7'b1100011 && (var3 == 3'b001 || var3 == 3'b100 || var3 == 3'b110))? 3'b001: // Branch if Non-zero
    					  (opcode == 7'b1101111 )? 3'b011 :        //JALR function                                                        
    					  (opcode == 7'b1100111 )? 3'b100 :                                                             
                3'b000;  

assign ALUop = (opcode == 7'b0010111)? 6'b010000 :                      // For AUIPC 
                (opcode == 7'b1100011)?
                (var3 == 3'b000 || var3 == 3'b001 )? 6'b010001 :    // Branch - Subtract
                (var3 == 3'b100 || var3 == 3'b101 )? 6'b010101 :    // Branch - Signed Compare
                (var3 == 3'b110 || var3 == 3'b111 )? 6'b010111      // Branch - Unsigned Compare
                :{1'b0,idata[5],idata[14:12],idata[30]}                 // Normal ALU
                :{1'b0,idata[5],idata[14:12],idata[30]};   


assign regwrite = (opcode == 7'b0000011 || opcode == 7'b0010011 || opcode == 7'b0110011 );
assign memwrite = (opcode == 7'b0100011 && var3 == 3'b010) ? 4'b1111 :
                  (opcode == 7'b0100011 && var3
                 == 3'b001) ? 4'b0011 :
                  (opcode == 7'b0100011 && var3
                 == 3'b000) ? 4'b0001 : 4'b0000;


assign Memtoreg = (opcode == 7'b0000011);// Memtoreg
assign memread = (opcode == 7'b0000011); // MemRead 
assign alusrc = (opcode == 7'b0010011) ; //alusrc 
assign ALUop = {1'b0,idata[5],idata[14:12],idata[30]}; //ALUop

    reg [31:0] register[31:0];
    // Initialise all registers to zero
    initial begin
      for  (i=0; i<32; i=i+1) begin
          register[i] = 0;
      end
    end
    // Read value from Registers
    assign datawire1 = register[idata[19:15]];
    assign datawire2 = register[idata[24:20]];
    // Write value to registers
    always@(posedge clk)
      if(regwrite)
        begin
          register[idata[11:7]] <= regwdata;
          // Added Line
          register[0] <= 0;

        end

assign var3 = idata[14:12];
assign opcode = idata[6:0];

assign imm = (opcode == 7'b0100011) ? {{20{idata[31]}},idata[31:25],idata[11:7]} :  // Store operations
             (opcode == 7'b0000011 && (var3
             == 3'b100 || var3
             == 3'b101)) ?     // Load Unsigned
                {{20{1'b0}},idata[31:20]} : {{20{idata[31]}},idata[31:20]};         // Load Signed

alu32 ALU32(
  .op(ALUop),
	.rv1(datawire1),
	.rv2( alusrc ? imm : datawire2),
  .rvout(alu_out)
	);

assign dwe = (memwrite == 4'b1111 && daddr[1:0]== 2'b00) ? 4'b1111:
  				(memwrite == 4'b0011 && daddr[1:0]== 2'b00) ? 4'b0011:
  				(memwrite == 4'b0011 && daddr[1:0]== 2'b10) ? 4'b1100:
  				(memwrite == 4'b0001 && daddr[1:0]== 2'b00) ? 4'b0001:
  				(memwrite == 4'b0001 && daddr[1:0]== 2'b01) ? 4'b0010:
  				(memwrite == 4'b0001 && daddr[1:0]== 2'b10) ? 4'b0100:
  				(memwrite == 4'b0001 && daddr[1:0]== 2'b11) ? 4'b1000: 4'b0000;

assign dwdata = datawire2 ;
assign daddr = (( 32'h00000fff ) & (datawire1+imm));// Daddr = ( rs1 + imm % 4096 ) 
assign ldata =  ((idata[14:12] === 3'b001 || idata[14:12] === 3'b101) && Memtoreg) ?  // LH , LHU
                ( 32'h0000fff & (drdata >> (daddr[1:0] * 8))) :
                ((idata[14:12] === 3'b000 || idata[14:12] === 3'b100) && Memtoreg) ?  // LB , LBU
                ( 32'h000000ff & (drdata >> (daddr[1:0] * 8))) :
                drdata;                                                              // LW

// If load operation -> ldata , else alu_out
assign regwdata = (reset) ? 0 : Memtoreg ? ldata  : alu_out;
always @(posedge clk) begin
    if (reset) begin
        iaddr <= 0;
    end else begin
        iaddr <= iaddr + 4;

    end
end

endmodule