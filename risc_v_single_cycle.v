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
            imem[i] = 32'h00000013;  // NOP instruction
        
        // Sample program - load into instruction memory
        imem[0] = 32'h00500113;  // addi x2, x0, 5
        imem[1] = 32'h00300193;  // addi x3, x0, 3
        imem[2] = 32'h002081b3;  // add x3, x1, x2
        imem[3] = 32'h00312233;  // slt x4, x2, x3
        imem[4] = 32'hfe310ae3;  // beq x2, x3, -12
    end
    
    assign instruction = imem[pc[31:2]];
endmodule

// Register File
module RegFile(
    input clk, reset, reg_write,
    input [4:0] read_reg1, read_reg2, write_reg,
    input [31:0] write_data,
    output [31:0] read_data1, read_data2
);
    reg [31:0] registers [31:0];
    integer k;
    
    always @(posedge clk) begin
        if (reset) begin
            for (k = 0; k < 32; k = k + 1)
                registers[k] <= 32'b0;
        end
        else if (reg_write) begin
            if (write_reg != 5'b0)  // Don't write to x0
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
            7'b0000011: begin // Load
                Branch <= 1'b0; MemRead <= 1'b1; MemtoReg <= 1'b1; 
                ALUOp <= 2'b00; MemWrite <= 1'b0; ALUSrc <= 1'b1; RegWrite <= 1'b1;
            end
            7'b0100011: begin // Store
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'bx; 
                ALUOp <= 2'b00; MemWrite <= 1'b1; ALUSrc <= 1'b1; RegWrite <= 1'b0;
            end
            7'b1100011: begin // Branch
                Branch <= 1'b1; MemRead <= 1'b0; MemtoReg <= 1'bx; 
                ALUOp <= 2'b01; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b0;
            end
            7'b0110011: begin // R-type
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b10; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b1;
            end
            7'b0010011: begin // I-type (ADDI, etc.)
                Branch <= 1'b0; MemRead <= 1'b0; MemtoReg <= 1'b0; 
                ALUOp <= 2'b10; MemWrite <= 1'b0; ALUSrc <= 1'b1; RegWrite <= 1'b1;
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
    input [6:0] Opcode,
    input [31:0] Instr,
    output reg [31:0] Imm_Out
);
    always @(*) begin
        case (Opcode)
            7'b0010011: Imm_Out <= {{20{Instr[31]}}, Instr[31:20]}; // I-type 
            7'b0000011: Imm_Out <= {{20{Instr[31]}}, Instr[31:20]}; // I-type (load)
            7'b0100011: Imm_Out <= {{20{Instr[31]}}, Instr[31:25], Instr[11:7]}; // S-type
            7'b1100011: Imm_Out <= {{19{Instr[31]}}, Instr[31], Instr[7], Instr[30:25], Instr[11:8], 1'b0}; // SB-type
            default:    Imm_Out <= 32'b0;
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
    always @(*) begin
        case(alu_control)
            4'b0000: result <= a & b;
            4'b0001: result <= a | b;
            4'b0010: result <= a + b;
            4'b0110: result <= a - b;
            4'b0011: result <= ~a;
            4'b1101: result <= a ^ b;
            4'b0100: result <= ~(a & b);
            4'b0111: result <= ~(a | b);
            4'b1001: result <= a << 1;
            4'b1010: result <= a >> 1;
            4'b1011: result <= $signed(a) >>> 1;
            4'b1000: result <= ($signed(a) < $signed(b)) ? 32'b1 : 32'b0;
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
            2'b00: ALUcontrol_Out <= 4'b0010; // Load/Store: Add
            2'b01: ALUcontrol_Out <= 4'b0110; // Branch: Subtract
            2'b10: begin
                case (funct3)
                    3'b000: begin
                        if (funct7 == 7'b0000000)
                            ALUcontrol_Out <= 4'b0010; // ADD
                        else
                            ALUcontrol_Out <= 4'b0110; // SUB
                    end
                    3'b001: ALUcontrol_Out <= 4'b1001; // SLL
                    3'b010: ALUcontrol_Out <= 4'b1000; // SLT
                    3'b100: ALUcontrol_Out <= 4'b0000; // AND
                    3'b101: ALUcontrol_Out <= 4'b0001; // OR
                    3'b110: ALUcontrol_Out <= 4'b1101; // XOR
                    3'b111: ALUcontrol_Out <= 4'b0111; // NAND
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
module data_memory(
    input clk, reset, Mread, Mwrite,
    input [31:0] addr, Wdata,
    output [31:0] Mout
);
    reg [31:0] D_Mem[63:0];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 64; i = i + 1)
                D_Mem[i] <= 32'h00000000;
        end
        else if (Mwrite) begin
            D_Mem[addr[7:2]] <= Wdata;
        end
    end
    
    assign Mout = Mread ? D_Mem[addr[7:2]] : 32'b0;
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
    // Wire declarations
    wire [31:0] pc_out_wire, pc_next_wire, pc_wire, instruction_wire;
    wire [31:0] read_data1_wire, read_data2_wire, write_data_wire;
    wire [31:0] branch_target_wire, imm_gen_wire, alu_mux_out_wire;
    wire [31:0] alu_result_wire, data_mem_out_wire;
    
    wire RegWrite_wire, ALUSrc_wire, MemRead_wire, MemWrite_wire;
    wire MemtoReg_wire, Branch_wire, Zero_wire, branch_select;
    wire [1:0] ALUOp_wire;
    wire [3:0] ALUcontrol_wire;

    // Branch condition check
    assign branch_select = Branch_wire & Zero_wire;

    // Program Counter
    ProgramCounter PC(
        .clk(clk),
        .rst(rst),
        .nextAddr(pc_wire),
        .currAddr(pc_out_wire)
    );

    // PC Adder
    pc_adder PC_Adder(
        .pc_in(pc_out_wire),
        .pc_next(pc_next_wire)
    );

    // Branch Adder
    Branch_Adder BA(  // Different instance name to avoid conflict
        .PC(pc_out_wire),
        .offset(imm_gen_wire),
        .branch_target(branch_target_wire)
    );

    // PC Mux
    pc_mux PC_Mux(
        .pc_in(pc_next_wire),
        .pc_branch(branch_target_wire),
        .pc_select(branch_select),
        .pc_out(pc_wire)
    );

    // Instruction Memory
    instruction_memory Instr_Mem(
        .pc(pc_out_wire),
        .instruction(instruction_wire)
    );

    // Control Unit
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

    // Register File
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

    // Immediate Generator
    ImmGen Imm_Gen(
        .Opcode(instruction_wire[6:0]),
        .Instr(instruction_wire),
        .Imm_Out(imm_gen_wire)
    );

    // ALU Source Mux
    MUX2to1 ALU_Src_Mux(
        .input0(read_data2_wire),
        .input1(imm_gen_wire),
        .select(ALUSrc_wire),
        .out(alu_mux_out_wire)
    );

    // ALU Control
    ALU_Control ALU_Ctrl(
        .ALUOp(ALUOp_wire),
        .funct3(instruction_wire[14:12]),
        .funct7(instruction_wire[31:25]),
        .ALUcontrol_Out(ALUcontrol_wire)
    );

    // ALU
    ALU_32bit ALU(
        .a(read_data1_wire),
        .b(alu_mux_out_wire),
        .alu_control(ALUcontrol_wire),
        .result(alu_result_wire),
        .zero(Zero_wire)
    );

    // Data Memory
    data_memory Data_Mem(
        .clk(clk),
        .reset(rst),
        .Mread(MemRead_wire),
        .Mwrite(MemWrite_wire),
        .addr(alu_result_wire),
        .Wdata(read_data2_wire),
        .Mout(data_mem_out_wire)
    );

    // Memory to Register Mux
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
    
    // Instantiate processor
    RISCV_Top UUT (
        .clk(clk), 
        .rst(rst)
    );
    
    // Clock generation
    initial begin
        clk = 0;
    end
    always #50 clk = ~clk; 
    
    // Reset and simulation control
    initial begin
        rst = 1'b1;
        #100;
        rst = 1'b0; 
        #5000; 
        $finish; 
    end
endmodule
