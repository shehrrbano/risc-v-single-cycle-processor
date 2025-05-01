// Program Counter

module Program_Counter(clk, reset, PC_in, PC_out);

input clk, reset;

input [31:0] PC_in;

output reg [31:0] PC_out;

always (posedge clk or posedge reset)

begin

if(reset)

PC_out <= 32'600;

else

PC_out <= PC_in;

end

endmodule

// PC + 4

module PCplus4 (fromPC, NextoPC);

input [31:0] fromPC;

output [31:0] NextoPC;

assign NextoPC = 4+ fromPC;

endmodule

// Instruction Memory

module Intruction_Mem(clk, reset, read_address, instruction_out);

input clk, reset;

input [31:0] read_address;

output reg [31:0] instruction_out;

integer k;

reg [31:0] I_Mem[63:0];

always (posedge clk or posedge reset)

begin

if(reset)

begin

for(k= 0; k<64; k=k+1) begin

I_Mem[k] <= 32'600;

end

end

else

instruction_out <= I_Mem[read_address];

end

endmodule

module Reg_File(clk, reset, RegWrite, Rs1, Rs2, Rd, Write_data, read_datal, read_data2);

input clk, reset, RegWrite;

input [4:0] Rs1, Rs2, Rd;

input [31:0] Write_data;

output [31:0] read_datal, read_data2;

integer k;

reg [31:0] Registers [31:0];

always (posedge clk or posedge reset)

begin

if (reset)

begin

for(k=0; k<32; k=k+1)begin Registers[k] <= 32'b00; end

end

else if(RegWrite)begin

Registers [Rd] <= Write_data;

end

end

assign read_datal Registers[Rs1];

assign read_data2 Registers [Rs2];

endmodule

// Immediate Generator

module ImmGen (Opcode, instruction, ImmExt);

input [6:0] Opcode;

input [31:0] instruction;

output [31:0] ImmExt;

always @(*)

begin

case (Opcode)

7'b0000011: ImmExt = {{20{instruction [31]}}, instruction[31:20]};
7'b0100011: ImmExt= {{20(instruction [31]}}, instruction [31:25], instruction [11:7]}; 
7'b1100011: ImmExt = {{19(instruction[31]}}, instruction [31], instruction [30:25], instruction[11:8],1'b0};

endcase

end

endmodule

// Control Unit

module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);

input [6:0] instruction;

output Branch, MemRead, MemtoReg, Memwrite, ALUSrc, RegWrite;

output [1:0] ALUOP;

always (*)

begin

case(instruction)

7'60110011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOP} <= 8'b001000_01;

7'b0000011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOP} <= 8'b111100_00;

7'60100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOP} <= 8'6100010_00;

7'b1100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b000001_01;

endcase
end
endmodule

// ALU

module ALU_unit (A, B, Control_in, ALU_Result, zero);

input [31:0] A, B;

input [3:0] Control_in;

output reg zero;

output reg [31:0] ALU_Result;

always (Control_in, or A or B)

begin

case(Control_in)

4'b0000: begin zero <= 8; ALU_Result <= A & B; end

4'b0001: begin zero <= 8; ALU_Result <= A B; end

4'b0010: begin zero <= 8; ALU_Result <= A + B; end

4'b0110: begin if(A==B) zero <= 1; else zero <=0; ALU_Result <= A - B; end

endcase
end
endmodule

// ALU Control

module ALU_Control (ALUOP, fun7, fun3, Control_out);

input fun7;

input [2:0] fun3;

input [1:0] ALUOP;

output reg [3:0] Control_out;

always (*)

begin

case((ALUOP, fun7, fun3))

6'600_0_000: Control_out <= 4'b0010;

6'b01_0_000: Control_out <= 4'b0110;

6'b10_0_000: Control_out <= 4°b0010;

6'b10_1_000: Control_out <= 4'b0110;

6'b10_0_111: Control_out <= 4'b0000;

6'b10_0_110: Control_out <= 4'b0001;

end

endcase

endmodule

// Data Memory

module Data_Memory(clk, reset, MemWrite, MemRead, read_address, Write_data, MemData_out);

input clk, reset, Memwrite, MemRead;

input [31:0] read_address, Write_data;

output [31:0] MemData_out;

integer k;

reg [31:0] D_Memory [63:0];

always (posedge clk or posedge reset)

begin

if(reset)

begin

for(k=0; k<64; k=k+1)begin

D_Memory[k] <= 32'b00;

end

end

else if (MemWrite) begin

D_Memory [read_address] <= Write_data;

end

end

assign MemData_out = (MemRead) ? D_Memory [read_address]: 32'b00;

endmodule

// Multiplexers

//Mux 1
module Mux1 (sell, A1, B1, Mux1_out);

input sell;

input [31:0] A1, B1;

output [31:0] Mux1_out;

assign Mux1_out (sel1==1'b0) ? A1: B1;

endmodule

//Mux 2

module Mux2(sel2, A2, B2, Mux2_out);

input sel2;

input [31:0] A2, B2;

output [31:0] Mux2_out;

assign Mux2_out = (sel2==1'b0) ? A2: B2;

endmodule

//Mux 3

module Mux3(sel3, A3, B3, Mux3_out);

input sel3;

input [31:0] A3, B3;

output [31:0] Mux3_out;

assign Mux2_out = (sel3==1'b0) ? A3: B3;

endmodule

// AND logic

module AND_logic(branch, zero, and_out);

input branch, zero;

output and_out;

assign and_out = branch & zero;

endmodule

// Adder

module Adder (in_1, in_2, Sum_out);

input [31:0] in_1, in_2;

output [31:0] Sum_out;

assign Sum_out = in_1 + in_2;

endmodule
