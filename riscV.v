module ProgramCounter(clk, reset, PC_in, PC_out);
input clk,reset;
input [31:0] PC_in;
output reg [31:0] PC_out;

always @(posedge clk or posedge reset) begin
if(reset)
PC_out<=32'b0;
else 
PC_out<=PC_in;
end
endmodule

//PC +4

module PCplus4(fromPC, NextoPC);

input [31:0] fromPC;
output [31:0] NextoPC;

assign NextoPC = 4 + fromPC;
endmodule

//Instruction Memory 

module Instruction_Mem(clk,reset, read_address, instruction_out);

input clk,reset;
input [31:0] read_address;
output reg [31:0] instruction_out;

reg [31:0] I_mem[63:0];
integer i;
always @(posedge clk or posedge reset) begin
if(reset) begin 
for(i=0;i<64;i=i+1) begin
I_mem[i]<= 32'b0;
end
end
else 
instruction_out<=I_mem[read_address];
end
endmodule


//Register file

module Reg_File(clk, reset, Regwrite, Rs1, Rs2, Rd, Write_data, read_data1, read_data2);
input clk, reset, Regwrite;
input [4:0] Rs1, Rs2, Rd;
input [31:0] Write_data;
output [31:0] read_data1, read_data2;

reg [31:0] Registers[31:0];
integer i;


always @(posedge clk or posedge reset) begin
    if (reset) begin
        for (i = 0; i < 32; i = i + 1)
            Registers[i] <= 32'b0;
    end
    else if (Regwrite) begin
        Registers[Rd] <= Write_data;
    end
end


assign read_data1 = Registers[Rs1];
assign read_data2 = Registers[Rs2];

endmodule

//Immediate Generator

module ImmGen(Opcode, instruction, ImmExt);
input [6:0] Opcode;
input [31:0] instruction;
output reg [31:0] ImmExt;

always @(*) begin
case(Opcode)
7'b0000011 : ImmExt <= {{20{instruction[31]}}, instruction[31:20]};
7'b0100011: ImmExt <= {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
7'b1100011:ImmExt <={{19{instruction[31]}}, instruction[31], instruction[30:25], instruction[11:8],1'b0};
endcase
end

endmodule


//Control Unit

module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);

    input [6:0] instruction;
    output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    output reg [1:0] ALUOp;
    
    always @(*) begin
        case(instruction)
            7'b0110011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b001000_01;  // R-type
            7'b0000011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b111100_00;  // lw
            7'b0100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b100010_00;  // sw
            7'b1100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b000000_01;  // beq
        endcase
    end

endmodule

//ALU
module ALU_unit(A,B , Control_in, ALU_Result,zero);

input [31:0] A,B;
input [3:0] Control_in;
output reg zero;
output reg [31:0] ALU_Result;

always @(Control_in or A or B) begin
case(Control_in)
4'b0000: begin zero<=0; ALU_Result <= A&B; end
4'b0001: begin zero <=0; ALU_Result <= A|B; end
4'b0010: begin zero<=0; ALU_Result <=A+B; end
4'b0110: begin if(A==B) zero<=1; else zero<=0;ALU_Result<=A-B; end
endcase
end
endmodule


//ALU Control
module ALU_Control(ALUop, fun7,fun3, Control_out);

input fun7;
input [2:0] fun3;
input [1:0] ALUop;
output reg [3:0] Control_out;

always @(*) begin
case({ALUop, fun7, fun3})
6'b00_0_000: Control_out <= 4'b0010;
6'b01_0_000: Control_out <=4'b0110;
6'b10_0_000: Control_out <= 4'b0010;
6'b10_1_000: Control_out <= 4'b0110;
6'b10_0_111: Control_out <= 4'b0000;
6'b10_0_110: Control_out <= 4'b0001;
endcase
end

endmodule



//Data Memory 
module Data_Memory(clk,reset, MemWrite, MemRead, read_address, Write_data, MemData_out);

input clk,reset,MemWrite, MemRead;
input [31:0] read_address, Write_data;
output [31:0] MemData_out;
integer i;
reg [31:0] D_Memory[63:0];

always @(posedge clk or posedge reset) begin
if(reset) begin
for(i=0;i<64;i=i+1) begin
D_Memory[i] <= 32'b00;
end
end

else if(MemWrite) begin 
D_Memory[read_address] <= Write_data;
end
end

assign MemData_out = (MemRead) ? D_Memory[read_address] : 32'b00;

endmodule

//Multiplexers 
module Mux1(sel1, a1, b1 ,Mux1_out);

input sel1;
input [31:0] a1,b1;
output [31:0] Mux1_out;

assign Mux1_out =(sel1==1'b0) ? a1 : b1;
endmodule

//MUX2  
module Mux2(sel2, a2, b2 ,Mux2_out);

input sel2;
input [31:0] a2,b2;
output [31:0] Mux2_out;

assign Mux2_out =(sel2==1'b0) ? a2 : b2;
endmodule

//MUX3
module Mux3(sel3, a3, b3 ,Mux3_out);

input sel3;
input [31:0] a3,b3;
output [31:0] Mux3_out;

assign Mux3_out =(sel3==1'b0) ? a3 : b3;
endmodule

//AND logic 
module AND_logic(branch, zero , and_out);

input branch, zero;
output and_out;

assign and_out = branch & zero;

endmodule

//Adder

module Adder(in_1, in_2, Sum_out);

input [31:0] in_1,in_2;
output [31:0] Sum_out;

assign Sum_out =in_1 + in_2;

endmodule


//All module instantiate 

module top(clk, reset);
input clk, reset;
wire [31:0] PC_top,instruction_top,Rd1_top, Rd2_top, ImmExt_top,mux1_top,Sum_out_top, NextoPC_top, PCin_top, address_top, Memdata_top, WriteBack_top;
wire RegWrite_top, ALUSrc_top,zero_top,branch_top, sel2_top, MemtoReg_top, MemWrite_top, MemRead_top;
wire [1:0]ALUop_top;
wire [3:0] control_top;

//Program Counter
ProgramCounter PC(.clk(clk), .reset(reset), .PC_in(PCin_top), .PC_out(PC_top));

//PC Adder 
PCplus4 PC_Adder(.fromPC(PC_top), .NextoPC(NextoPC_top));

//iinstruction Memory 

Instruction_Mem Inst_Memory(.clk(clk),.reset(reset), .read_address(PC_top), .instruction_out(instruction_top));

//Register File
 Reg_File  Reg_File(.clk(clk), .reset(reset), .Regwrite(RegWrite_top), .Rs1(instruction_top[19:15]), .Rs2(instruction_top[24:20]), .Rd(instruction_top[11:7]), .Write_data(WriteBack_top), .read_data1(Rd1_top), .read_data2(Rd2_top));

//Immediate Genereator
ImmGen ImmGen(.Opcode(instruction_top[6:0]), .instruction(instruction_top), .ImmExt(ImmExt_top));

//Control Unit
 Control_Unit Control_unit(.instruction(instruction_top[6:0]), .Branch(branch_top), .MemRead(MemRead_top), .MemtoReg(MemtoReg_top), .ALUOp(ALUop_top), .MemWrite(MemWrite_top), .ALUSrc(), .RegWrite(RegWrite_top));

//ALU Control
ALU_Control ALU_Control(.ALUop(ALUop_top), .fun7(instruction_top[30]),.fun3(instruction_top[14:12]), .Control_out(control_top));

//ALU

ALU_unit(.A(Rd1_top),.B(ImmExt_top) , .Control_in(control_top), .ALU_Result(address_top),.zero(zero_top));

//ALU Mux
Mux1 ALU_mux(.sel1(ALUSrc_top), .a1(Rd2_top), .b1(ImmExt_top) ,.Mux1_out(ImmExt_top));


//Adder
Adder Adder(.in_1(PC_top), .in_2(ImmExt_top), .Sum_out(Sum_out_top));


//AND Gate
AND_logic AND(.branch(branch_top), .zero(zero_top) , .and_out(sel2_top));


//Mux
Mux2 Adder_mux(.sel2(sel2_top), .a2(NextoPC_top), .b2(Sum_out_top) ,.Mux2_out(PCin_top));

//Data Memory
Data_Memory Data_mem(.clk(clk),.reset(reset), .MemWrite(MemWrite_top), .MemRead(MemRead_top), .read_address(address_top), .Write_data(Rd2_top), .MemData_out(Memdata_top));

//Mux 
Mux3 Memory_mux(.sel3(MemtoReg_top), .a3(address_top), .b3(Memdata_top) ,.Mux3_out(WriteBack_top));

endmodule


