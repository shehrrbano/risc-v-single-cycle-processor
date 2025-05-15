// Program Counter
module ProgramCounter(clk, rst, nextAddr, currAddr);
 input clk, rst;
 input [31:0] nextAddr;
 output reg [31:0] currAddr;
 always @(posedge clk or posedge rst)
 begin
 if (rst)
 currAddr <= 32'd0;
 else
 currAddr <= nextAddr;
 end
endmodule

// Instruction Memory
module instruction_memory(pc, instruction);
 input [31:0] pc;
 output [31:0] instruction;
 reg [31:0] imem [63:0];
 integer i;
 initial begin
 for (i = 0; i < 64; i = i + 1)
 imem[i] = 32'h00000013;
 imem[0] = 32'h00500113;
 imem[1] = 32'h00300193;
 imem[2] = 32'h002081b3;
 imem[3] = 32'h00312233;
 imem[4] = 32'hfe310ae3;
 end
 assign instruction = imem[pc[31:2]];
endmodule

//Register File
module RegFile (clk,reset, reg_write,read_reg1,read_reg2, write_reg,write_data,read_data1,read_data2
);
 input clk, reset, reg_write;
 input [4:0] read_reg1, read_reg2,
write_reg;
 input [31:0] write_data;
 output [31:0] read_data1, read_data2;
reg [31:0] registers [31:0];
 integer k;
 always @(posedge clk or posedge reset)
begin
 if (reset) begin
 for (k = 0; k < 32; k = k + 1)
 registers[k] <= 32'b0;
 end
 else if (reg_write) begin
 registers[write_reg] <= write_data;
 end
 end
 assign read_data1 = registers[read_reg1];
 assign read_data2 = registers[read_reg2];
endmodule

// Immediate Generator
module ImmGen (Opcode, Instr, Imm_Out); 
input [6:0] Opcode; 
input [31:0] Instr; 
output reg [31:0] Imm_Out; 
always @(*) 
begin 
case (Opcode) 
7'b0000011: Imm_Out <= {{20{Instr [31]}}, Instr[31:20]}; //I-type 
7'b0100011: Imm_Out <= {{20{Instr [31]}}, Instr [31:25], Instr [11:7]}; //S-type 
7'b1100011: Imm_Out <= {{19{Instr[31]}}, Instr [7], Instr [30:25], Instr[11:8],1'b0}; //SB-type 
endcase 
end 
endmodule 

// Control Unit
module Control_Unit(opcode, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
input [6:0] opcode;
output reg Branch, MemRead, MemtoReg;
output reg [1:0] ALUOp;
output reg MemWrite, ALUSrc, RegWrite;
always @ (*)
begin
case (opcode)
 7'b0000011: begin Branch = 1'b0; MemRead = 1'b1; MemtoReg = 1'b1; ALUOp =
2'b00; MemWrite = 1'b0; ALUSrc = 1'b1; RegWrite = 1'b1; end
 7'b0100011: begin Branch = 1'b0; MemRead = 1'b0; MemtoReg = 1'bx; ALUOp =
2'b00; MemWrite = 1'b1; ALUSrc = 1'b1; RegWrite = 1'b0; end
 7'b1100011: begin Branch = 1'b1; MemRead = 1'b0; MemtoReg = 1'bx; ALUOp =
2'b01; MemWrite = 1'b0; ALUSrc = 1'b0; RegWrite = 1'b0; end
 7'b0110011: begin Branch = 1'b0; MemRead = 1'b0; MemtoReg = 1'b0; ALUOp =
2'b10; MemWrite = 1'b0; ALUSrc = 1'b0; RegWrite = 1'b1; end
endcase
end
endmodule

// ALU
module ALU_unit (A, B, Control_in, ALU_Result, zero);
module ALU_32bit( 
    input [31:0] a, b, 
    input [3:0] alu_control, 
    output reg [31:0] result, 
    output zero 
);
    always @(*) 
    begin 
        case(alu_control) 
            4'b0000: result = a & b; 
            4'b0001: result = a | b; 
            4'b0010: result = a + b; 
            4'b0110: result = a - b; 
            4'b0011: result = ~a;   
            4'b1101: result = a ^ b; 
            4'b0100: result = ~(a & b); 
            4'b0111: result = ~(a | b); 
            4'b1001: result = a << 1; 
            4'b1010: result = a >> 1; 
            4'b1011: result = $signed(a) >>> 1; 
            4'b1000: result = (a < b) ? 32'b1 : 32'b0; 
        default: result = 32'b0; 
        endcase 
end 
assign zero = (result == 32'b0) ? 1'b1 : 1'b0; 
endmodule 

// ALU Control
module ALU_Control( 
    input [1:0] PC_IN, 
    input [5:0] PC_OUT, 
    output reg [3:0] Control_Output 
); 
    always @(*) begin 
        case (PC_IN) 
            2'b00: Control_Output = 4'b0010; 
            2'b01: Control_Output = 4'b0110; 
            2'b10: begin 
                case (PC_OUT) 
                    6'b100000: Control_Output = 4'b0010; 
                    6'b100010: Control_Output = 4'b0110; 
                    6'b100100: Control_Output = 4'b0000; 
                    6'b100101: Control_Output = 4'b0001; 
                    6'b100110: Control_Output = 4'b1101; 
                    6'b100111: Control_Output = 4'b0111; 
                    6'b101010: Control_Output = 4'b1000; 
                    6'b000000: Control_Output = 4'b1001; 
                    6'b000010: Control_Output = 4'b1010; 
                    6'b000011: Control_Output = 4'b1011; 
                    default: Control_Output = 4'b0000; 
endcase 
end 
default: Control_Output = 4'b0000; 
endcase 
end 
endmodule 

// Data Memory
 module data_memory(clk, reset, Mread, Mwrite, Adder, Wdata, Mout);
input clk, reset, Mread, Mwrite;
input [31:0] Adder, Wdata;
output [31:0] Mout;
reg [31:0] D_Mem[63:0];
integer i;
always @(posedge clk)
begin
 if (reset) begin
 for (i = 0; i < 64; i = i + 1) begin
 D_Mem[i] = 32'h00000000;
 end
 end
 else if (Mwrite) begin
 D_Mem[i] = Wdata;
 end
end
assign Mout = Mread ? D_Mem[i] : 32'b0;
endmodule


//MUX
module Mux1 (sel, A, B, Mux_out);
input sel;
input [31:0] A, B;
output [31:0] Mux_out;
assign Mux_out (se1==1) ? A: B;
endmodule


// Adder
module Adder (I1, I2, Sum_out);
input [31:0] I1, I2;
output [31:0] Sum_out;
assign Sum_out = I1 + I2;
endmodule

//All Module 
module top (clk, reset);
input clk, reset;

//Program Counter
 ProgramCounter PC(.clk(clk), .rst(reset), .nextAddr(), .currAddr());

//Instruction Memory
 instruction_memory Inst_Memory(.pc(), .instruction());

//Register File
 RegFile Register_File(.clk(clk), .reset(reset), .reg_write(), .read_reg1(), .read_reg2(), .write_reg(), .write_data(), .read_data1(), .read_data2()
);

//Immediate Generator
 ImmGen Imm_Gen( .Opcode(), .Instr(), .Imm_Out());

//Control Unit
 Control_Unit CU( .opcode(), .Branch(), .MemRead(), .MemtoReg(), .ALUOp(), .MemWrite(), .ALUSrc(), .RegWrite());

//ALU
 ALU_unit ALUU( .A(), .B(), .Control_in(), .ALU_Result(), .zero());

//ALU Control
 module ALU_Control( 
    input [1:0] PC_IN, 
    input [5:0] PC_OUT, 
    output reg [3:0] Control_Output 
); 

//Data Memory 
  data_memory DM(.clk(clk), .reset(reset), .Mread(), .Mwrite(), .Adder(), .Wdata(), .Mout());

 //MUX
  Mux1 M1(.sel(), .A(), .B(), .Mux_out());

 //Adder
  Adder adder(.I1(), .I2(), .Sum_out()); 

 //ALU Module
  top (.clk(clk), .reset(reset)); 
