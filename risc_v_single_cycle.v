// Program Counter
module ProgramCounter(
    input clk, rst,
    input [31:0] nextAddr,
    output reg [31:0] currAddr
);
    always @(posedge clk) begin
        if (rst)
            currAddr <= 32'd0;
        else
            currAddr <= nextAddr;
    end
endmodule

// PC Adder
module pc_adder(
    input [31:0] pc_in,       
    output reg [31:0] pc_next 
);
    always @(*) begin
        pc_next <= pc_in + 4;
    end
endmodule

// PC Mux
module pc_mux(
    input [31:0] pc_in,       
    input [31:0] pc_branch,   
    input pc_select,          
    output reg [31:0] pc_out  
);
    always @(*) begin
        if (pc_select == 1'b0)
            pc_out <= pc_in;
        else
            pc_out <= pc_branch;
    end
endmodule

// Instruction Memory
module instruction_memory(
    input [31:0] pc,
    output [31:0] instruction
);
    reg [31:0] imem [63:0];
    integer i;
    
    initial begin
        for (i = 0; i < 64; i = i + 1)
            imem[i] = 32'b0;
        
        // R-type
        imem[0] = 32'b00000000000000000000000000000000;
        imem[1] = 32'b00000001100110000000011010110011;
        imem[2] = 32'b01000000001101000000001010110011;
        imem[3] = 32'b00000000001100010111000010110011;
        imem[4] = 32'b00000000010100011110001000110011;
        imem[5] = 32'b00000000010100011100001000110011;
        imem[6] = 32'b00000000010100011001001000110011;
        
        // I-type
        imem[10] = 32'b00000000001010101000101100010011;
        imem[11] = 32'b00000000001101000110010010010011;
        imem[12] = 32'b00000000010001000100010010010011;
        imem[13] = 32'b00000000010100010111000010010011;
        imem[14] = 32'b00000000011000011001001000010011;
        
        // L-type
        imem[20] = 32'b00000000111100010010010000000011;
        
        // S-type
        imem[22] = 32'b00000000111000110010011000100011;
        
        // B-type
        imem[23] = 32'b00000000100101001000011001100011;
        imem[24] = 32'b00000000100101001001011101100011;
        
        // J-type
        imem[27] = 32'b00000000001000010100000011101111;
    end
    
    assign instruction = imem[pc[7:2]];
endmodule

// Register File
module RegFile(
    input clk, reset, reg_write,
    input [4:0] read_reg1, read_reg2, write_reg,
    input [31:0] write_data,
    output [31:0] read_data1, read_data2
);
    reg [31:0] registers [31:0];
    
    initial begin
        registers[0] = 0;
        registers[1] = 15;
        registers[2] = 8;
        registers[3] = 25;
        registers[4] = 100;
        registers[5] = 7;
        registers[6] = 64;
        registers[7] = 13;
        registers[8] = 16;
        registers[9] = 9;
        registers[10] = 45;
        registers[11] = 21;
        registers[12] = 128;
        registers[13] = 33;
        registers[14] = 77;
        registers[15] = 55;
        registers[16] = 88;
        registers[17] = 120;
        registers[18] = 200;
        registers[19] = 150;
        registers[20] = 180;
        registers[21] = 95;
        registers[22] = 110;
        registers[23] = 75;
        registers[24] = 160;
        registers[25] = 85;
        registers[26] = 42;
        registers[27] = 66;
        registers[28] = 37;
        registers[29] = 144;
        registers[30] = 29;
        registers[31] = 199;
    end

    integer k;
    
always @(posedge clk) begin
    if (reg_write && !reset) begin
        registers[write_reg] <= write_data;
    end
end
    
    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];
endmodule

// Control Unit
module Control_Unit(
    input [6:0] opcode,
    output reg Branch, MemRead, MemtoReg,
    output reg [1:0] ALUOp,
    output reg MemWrite, ALUSrc, RegWrite
);
    always @(*) begin
        case (opcode)
            7'b0000011: begin
                Branch <= 1'b0; MemRead <= 1'b1; MemtoReg <= 1'b1; 
                ALUOp <= 2'b00; MemWrite <= 1'b0; ALUSrc <= 1'b1; RegWrite <= 1'b1;
            end
            7'b0100011: begin
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b00; MemWrite <= 1'b1; ALUSrc <= 1'b1; RegWrite <= 1'b0;
            end
            7'b1100011: begin
                Branch <= 1'b1; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b01; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b0;
            end
            7'b0110011: begin
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b10; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b1;
            end
            7'b0010011: begin
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b10; MemWrite <= 1'b0; ALUSrc <= 1'b1; RegWrite <= 1'b1;
            end
            7'b1101111: begin
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b10; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b1;
            end
            default: begin
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0;
                ALUOp <= 2'b00; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b0;
            end
        endcase
    end
endmodule

// Immediate Generator
module ImmGen(
    input [31:0] Instr,
    output reg [31:0] Imm_Out
);
    wire [6:0] Opcode = Instr[6:0];
    
    always @(*) begin
        case (Opcode)
            7'b0010011: Imm_Out <= {{20{Instr[31]}}, Instr[31:20]};
            7'b0000011: Imm_Out <= {{20{Instr[31]}}, Instr[31:20]};
            7'b0100011: Imm_Out <= {{20{Instr[31]}}, Instr[31:25], Instr[11:7]};
            7'b1100011: Imm_Out <= {{19{Instr[31]}}, Instr[31], Instr[7], Instr[30:25], Instr[11:8], 1'b0};
            7'b1101111: Imm_Out <= {{11{Instr[31]}}, Instr[19:12], Instr[20], Instr[30:21], 1'b0};
            default: Imm_Out <= 32'b0;
        endcase
    end
endmodule

// ALU
module ALU_32bit(
    input [31:0] a, b,
    input [3:0] alu_control,
    output reg [31:0] result,
    output zero
);
    always @(a or b or alu_control) begin
        case(alu_control)
            4'b0000: result <= a + b;
            4'b0001: result <= a - b;
            4'b0010: result <= a & b;
            4'b0011: result <= a | b;
            4'b0100: result <= a ^ b;
            4'b0101: result <= a << b[4:0];
            4'b0110: result <= a >> b[4:0];
            default: result <= 32'b0;
        endcase
    end
    
    assign zero = (result == 32'b0) ? 1'b1 : 1'b0;
endmodule

// ALU Control
module ALU_Control(
    input [1:0] ALUOp,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] ALUcontrol_Out
);
    always @(*) begin
        case (ALUOp)
            2'b00: begin
                ALUcontrol_Out <= 4'b0000;
            end
            2'b01: begin
                ALUcontrol_Out <= 4'b0001;
            end
            2'b10: begin
                case (funct3)
                    3'b000: begin
                        if (funct7 == 7'b0000000)
                            ALUcontrol_Out <= 4'b0000;
                        else if (funct7 == 7'b0100000)
                            ALUcontrol_Out <= 4'b0001;
                        else
                            ALUcontrol_Out <= 4'b0000;
                    end
                    3'b111: ALUcontrol_Out <= 4'b0010;
                    3'b110: ALUcontrol_Out <= 4'b0011;
                    3'b100: ALUcontrol_Out <= 4'b0100;
                    3'b001: ALUcontrol_Out <= 4'b0101;
                    3'b101: begin
                        if (funct7 == 7'b0000000)
                            ALUcontrol_Out <= 4'b0110;
                        else if (funct7 == 7'b0100000)
                            ALUcontrol_Out <= 4'b0111;
                        else
                            ALUcontrol_Out <= 4'b0110;
                    end
                    3'b010: ALUcontrol_Out <= 4'b1000;
                    default: ALUcontrol_Out <= 4'b0000;
                endcase
            end
            default: ALUcontrol_Out <= 4'b0000;
        endcase
    end
endmodule

// ALU Mux
module MUX2to1(
    input [31:0] input0,
    input [31:0] input1,
    input select,
    output reg [31:0] out
);
    always @(*) begin
        out <= select ? input1 : input0;
    end
endmodule

// Data Memory
module Data_Memory(
    input clk,               
    input rst,               
    input MemRead,           
    input MemWrite,          
    input [31:0] address,    
    input [31:0] write_data, 
    output [31:0] read_data 
);
    reg [31:0] D_Memory [63:0];
    integer k;

    initial begin
        for (k = 0; k < 64; k = k + 1) begin
            D_Memory[k] = 32'b0;
        end
        D_Memory[15] = 65;
        D_Memory[17] = 56;
    end

    assign read_data = (MemRead) ? D_Memory[address[7:2]] : 32'b0;

    always @(posedge clk) begin
        if (rst) begin
            for (k = 0; k < 64; k = k + 1) begin
                D_Memory[k] <= 32'b0;
            end
            D_Memory[15] <= 65;
            D_Memory[17] <= 56;
        end else if (MemWrite) begin
            D_Memory[address[7:2]] <= write_data;
        end
    end
endmodule

// Branch Adder
module Branch_Adder(
    input [31:0] PC,
    input [31:0] offset,
    output reg [31:0] branch_target
);
    always @(*) begin
        branch_target <= PC + offset;
    end
endmodule

// RISC-V Top Module
module RISCV_Top(
    input clk, rst
);
    wire [31:0] pc_out_wire, pc_next_wire, pc_wire, instruction_wire;
    wire [31:0] read_data1_wire, read_data2_wire, write_data_wire;
    wire [31:0] branch_target_wire, imm_gen_wire, alu_mux_out_wire;
    wire [31:0] alu_result_wire, data_mem_out_wire;
    
    wire RegWrite_wire, ALUSrc_wire, MemRead_wire, MemWrite_wire;
    wire MemtoReg_wire, Branch_wire, Zero_wire, branch_select;
    wire [1:0] ALUOp_wire;
    wire [3:0] ALUcontrol_wire;

    assign branch_select = Branch_wire & Zero_wire;

    ProgramCounter PC(
        .clk(clk),
        .rst(rst),
        .nextAddr(pc_wire),
        .currAddr(pc_out_wire)
    );

    pc_adder PC_Adder(
        .pc_in(pc_out_wire),
        .pc_next(pc_next_wire)
    );

    Branch_Adder BA(
        .PC(pc_out_wire),
        .offset(imm_gen_wire),
        .branch_target(branch_target_wire)
    );

    pc_mux PC_Mux(
        .pc_in(pc_next_wire),
        .pc_branch(branch_target_wire),
        .pc_select(branch_select),
        .pc_out(pc_wire)
    );

    instruction_memory Instr_Mem(
        .pc(pc_out_wire),
        .instruction(instruction_wire)
    );

    Control_Unit Control(
        .opcode(instruction_wire[6:0]),
        .Branch(Branch_wire),
        .MemRead(MemRead_wire),
        .MemtoReg(MemtoReg_wire),
        .ALUOp(ALUOp_wire),
        .MemWrite(MemWrite_wire),
        .ALUSrc(ALUSrc_wire),
        .RegWrite(RegWrite_wire)
    );

    RegFile Register_File(
        .clk(clk),
        .reset(rst),
        .reg_write(RegWrite_wire),
        .read_reg1(instruction_wire[19:15]),
        .read_reg2(instruction_wire[24:20]),
        .write_reg(instruction_wire[11:7]),
        .write_data(write_data_wire),
        .read_data1(read_data1_wire),
        .read_data2(read_data2_wire)
    );

    ImmGen Imm_Gen(
        .Instr(instruction_wire),
        .Imm_Out(imm_gen_wire)
    );

    MUX2to1 ALU_Src_Mux(
        .input0(read_data2_wire),
        .input1(imm_gen_wire),
        .select(ALUSrc_wire),
        .out(alu_mux_out_wire)
    );

    ALU_Control ALU_Ctrl(
        .ALUOp(ALUOp_wire),
        .funct3(instruction_wire[14:12]),
        .funct7(instruction_wire[31:25]),
        .ALUcontrol_Out(ALUcontrol_wire)
    );

    ALU_32bit ALU(
        .a(read_data1_wire),
        .b(alu_mux_out_wire),
        .alu_control(ALUcontrol_wire),
        .result(alu_result_wire),
        .zero(Zero_wire)
    );

    Data_Memory Data_Mem(
        .clk(clk),
        .rst(rst),
        .MemRead(MemRead_wire),
        .MemWrite(MemWrite_wire),
        .address(alu_result_wire),
        .write_data(read_data2_wire),
        .read_data(data_mem_out_wire)
    );

    MUX2to1 WB_Mux(
        .input0(alu_result_wire),
        .input1(data_mem_out_wire),
        .select(MemtoReg_wire),
        .out(write_data_wire)
    );

endmodule

// RISC-V Testbench
module RISCV_Top_Tb;
    reg clk, rst;
    
    RISCV_Top UUT (
        .clk(clk), 
        .rst(rst)
    );
    
    initial begin
        clk = 0;
    end
    always #50 clk = ~clk; 
    
    initial begin
        rst = 1'b1;
        #100;
        rst = 1'b0; 
        #5000; 
        $finish; 
    end
endmodule 
