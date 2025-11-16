module DataBus(
    input [2:0] s,
    output reg [15:0] data,
    input [15:0] mem_data,
    input [11:0] AR, PC,
    input [15:0] DR, AC, IR, TR,
    input clk
);
always @(*) begin
    case(s)
        3'b001: data = AR;
        3'b010: data = PC;
        3'b011: data = DR;
        3'b100: data = AC;
        3'b101: data = IR;
        3'b110: data = TR;
        3'b111: data = mem_data;
        default: data = 16'b0;
    endcase
end
endmodule

module PC(input clk,
	  output reg [11:0] data, 
	  input [15:0] bus,
	  input [2:0] ctrl
	  );
initial data = 3'h000;
always @(posedge clk)
begin
    case (ctrl)
        3'b100: data = bus[11:0];
        3'b010: data = data + 1;
        3'b001: data = 16'b0;
    endcase
end
endmodule

module AR(input clk,
          output reg [11:0] data,
          input [15:0] bus,
          input [2:0] ctrl
         );
always @(posedge clk) begin
    case(ctrl)
        3'b100: data = bus[11:0];
        3'b010: data = data + 1;
        3'b001: data = 16'b0;
    endcase
end
endmodule

module DR(input clk,
          output reg [15:0] data,
          input [15:0] bus,
          input [2:0] ctrl
         );
always @(posedge clk) begin
    case(ctrl)
        3'b100: data = bus;
        3'b010: data = data + 1;
        3'b001: data = 16'b0;
    endcase
end
endmodule

module AC(input clk,
          output reg [15:0] data,
          input [15:0] AC_input, // Output of ALU
          input [2:0] ctrl
         );
always @(posedge clk) begin
    case(ctrl)
        3'b100: data = AC_input;
        3'b010: data = data + 1;
        3'b001: data = 16'b0;
    endcase
end
endmodule

module IR(input clk,
	  output reg [15:0] data, 
	  input [15:0] bus,
	  input ctrl  // only load
	  );
always @(posedge clk)
begin
    if(ctrl == 1) data = bus;
end
endmodule

module TR(input clk,
          output reg [15:0] data,
          input [15:0] bus,
          input [2:0] ctrl
         );
always @(posedge clk) begin
    case(ctrl)
        3'b100: data = bus;
        3'b010: data = data + 1;
        3'b001: data = 16'b0;
    endcase
end
endmodule

module alu(
    input clk,
    input [2:0] AC_ctrl,
    output reg E,
    input [15:0] DR, AC,
    output reg [15:0] AC_input,
    input [2:0] alu_s,
    input [1:0] E_ctrl
);
initial E = 0;
always @(*) begin
    case(E_ctrl)
        2'b01: E = 0;
        2'b10: E = ~E;
        default: ;
    endcase

    if (alu_s != 3'b000 && alu_s != 3'b111) begin
        case(alu_s) // 000 = no operation
            3'b001: AC_input = AC & DR; //AND
            3'b010: {E, AC_input} = AC + DR; // ADD
            3'b011: AC_input = DR; //LOD_DR
            3'b100: AC_input = ~AC; //COM
            3'b101: begin // SHR
                AC_input = {E, AC[15:1]}; // E is the extend bit 
                E = AC[0];
            end
            3'b110: begin// SHL
                AC_input = {AC[14:0], E};
                E = AC[15];
            end
        endcase
    end
end
endmodule

module ControlUnit(
    input clk,
    input E,
    input [15:0] AC, IR, DR,
    output reg [1:0] mem_ctrl,
    output reg [2:0] s, PC_ctrl, AR_ctrl, DR_ctrl, AC_ctrl, TR_ctrl, alu_s,
    output reg IR_load,
    output reg [1:0] E_ctrl
);

//________________________________control unit defines
`define FETCH    2'b00
`define DECODE   2'b01
`define INDIRECT 2'b10
`define EXECUTE  2'b11

initial begin
    s = 3'b010; alu_s = 3'b000; mem_ctrl = 2'b00; PC_ctrl = 3'b001;//pc
    AC_ctrl = 3'b001; DR_ctrl = 3'b001; TR_ctrl = 3'b001; IR_load = 0;//clear
end

reg [15:0] T; // internal registers of control unit
reg [7:0] D;
reg [3:0] sc;
reg is_register_ref, clr, inr, I;//  clr, inr are for sc

reg [1:0] state, nextstate;

initial begin
    is_register_ref = 1'b0; sc = 4'b0000; state = `FETCH; inr = 1; clr = 0;
    T = 16'b0; D = 8'b0;
end

always @(posedge clk or posedge clr) begin
    if (clr) begin
        sc <= 4'b0000;
        state <= `FETCH;
    end else if (inr) sc <= sc + 1;
end

always @(sc) begin
    T = 16'b0; // clearing the last garbage values
    T[sc] = 1'b1;
end

always @(nextstate) begin
    if (clr) state <= `FETCH;
    else state <= nextstate;
end

always @(posedge clk)
begin
    case(state)
	`FETCH:
	begin
		E_ctrl = 2'b0; clr = 0;
		AR_ctrl = 3'b0; AC_ctrl = 3'b0; PC_ctrl = 3'b0;
		alu_s = 3'b0; D = 8'b0;
		mem_ctrl = 2'b0; DR_ctrl = 3'b0; TR_ctrl = 3'b0;
		IR_load = 0; is_register_ref = 0;

		if (T[0]) begin// AR<--PC
			s = 3'b010;// This sets the bus selector to select the Program Counter (PC)
			AR_ctrl[2] = 1; // load AR
		end

		if (T[1]) begin // IR <--mem
			mem_ctrl = 2'b01;// Read 
			s = 3'b111;//  select the memory's output
			IR_load = 1;// enable loading from the bus to get the program line in IR
			PC_ctrl = 3'b010;// PC<--PC + 1
			nextstate <= `DECODE;
		end
	end
/*       3'b001: data = AR;
        3'b010: data = PC;
        3'b011: data = DR;
        3'b100: data = AC;
        3'b101: data = IR;
        3'b110: data = TR;
        3'b111: data = mem_data;*/
    `DECODE:
    begin
        PC_ctrl = 3'b000;
        IR_load = 0;
            // Now we have to get the address with the bite code
        if (T[2]) begin // decoding the opcode and getting the addr of the operand = D[opcode]=1 and AR <--IR
            #7// to simulate the hardware delay
            D[IR[14:12]] = 1; // decoding the opcode
            I = IR[15];
            s = 3'b101;// select IR
            AR_ctrl = 3'b100;//AR load enabled
            nextstate <= `INDIRECT;
        end
    end

    `INDIRECT:
    begin
        AR_ctrl = 3'b000;
        s = 3'b000; //nothing

        if (T[3]) begin // If it's not 111 we are sure it's a memory reference op
            if (~D[7] && I) begin  
                mem_ctrl = 2'b01;
                s = 3'b111;
                AR_ctrl = 3'b100;
            end
            if (D[7] && ~I) begin
                is_register_ref = 1;// $
            end
            nextstate <= `EXECUTE;
        end
    end
 `EXECUTE:
    begin
        s = 3'b000;
        AR_ctrl = 3'b000;
        mem_ctrl = 2'b00;

        if (D[0]) begin
            if (T[4]) begin s = 3'b111; DR_ctrl = 3'b100; end // from memory to DR
            if (T[5]) begin AC_ctrl = 3'b100; alu_s = 3'b001; clr <= 1; end //AND
        end

        if (D[1]) begin
            if (T[4]) begin s = 3'b111; DR_ctrl = 3'b100; end // ....
            if (T[5]) begin AC_ctrl = 3'b100; alu_s = 3'b010; clr <= 1; end// ADD
        end

        if (D[2]) begin
            if (T[4]) begin s = 3'b111; DR_ctrl = 3'b100; end
            if (T[5]) begin AC_ctrl = 3'b100; alu_s = 3'b011; clr <= 1; end // Load to AC
        end

        if (D[3]) begin // store AC in memory
            if (T[4]) begin s = 3'b100; mem_ctrl = 2'b10; clr <= 1; end //AC to bus and then bus writes to memory
        end

        if (D[4]) begin //  BUN(Branch Unconditionally to AR)  pc <= AR
            if (T[4]) begin s = 3'b001; PC_ctrl = 3'b100; clr <= 1; end 
        end

        if (D[5]) begin //BSA (Branch and save) = back up from pc before changing it first memory <=PC and AR++ (to go to the after the saved ret addr)and then pc <= AR
            if (T[4]) begin s = 3'b010; mem_ctrl = 2'b10; AR_ctrl = 3'b010; end
            if (T[5]) begin AR_ctrl = 3'b000; s = 3'b001; PC_ctrl = 3'b100; clr <= 1; end // stoping AR++ and pc <= AR
        end

        if (D[6]) begin // ISZ(increment and skip if zero)
            if (T[4]) begin mem_ctrl = 2'b01; s = 3'b111; DR_ctrl = 3'b100; end // DR load from memory
            if (T[5]) begin DR_ctrl = 3'b010; end //  DR++
            if (T[6]) begin
                DR_ctrl = 3'b000; s = 3'b011; mem_ctrl = 2'b10; // stops DR and DR to bus and then write to memory = mem++
                if (DR == 0) PC_ctrl = 3'b010; // DR ==0 : pc++
                clr <= 1;
            end
        end

        if (is_register_ref) begin
            if (IR[11]) AC_ctrl = 3'b001; //CLA (Clear AC) 1000 0000 0000
            if (IR[10]) E_ctrl = 1; //CLE (Clear E)
            if (IR[9]) begin AC_ctrl = 3'b100; alu_s = 3'b100; end //CMA (Complement AC)
            if (IR[8]) E_ctrl = 2; // CME (Complement E)
            if (IR[7]) begin AC_ctrl = 3'b100; alu_s = 3'b101; end// CIR (Circulate Right)
            if (IR[6]) begin AC_ctrl = 3'b100; alu_s = 3'b110; end
            if (IR[5]) AC_ctrl = 3'b010; // INC (Increment AC)
            if (IR[4]) if (~AC[15]) PC_ctrl = 3'b010;  //SPA (Skip on Positive AC)
            if (IR[3]) if (AC[15]) PC_ctrl = 3'b010;// 
            if (IR[2]) if (AC == 0) PC_ctrl = 3'b010;// SZA (Skip on Zero AC)
            if (IR[1]) if (~E) PC_ctrl = 3'b010;//SZE (Skip on Zero E)
            if (IR[0]) $finish;// Halt
            clr <= 1;
        end
    end
endcase
end
endmodule

module MemoryUnit(output reg [15:0] data, 
         input [11:0] AR, 
         input [1:0] ctrl, 
         input [15:0] bus, input1, input2,
         input clk,
         output reg [15:0] add, sub, mlt);

reg [15:0] mem [0:4095];

always @(posedge clk)
begin
    mem[3840] <= input1;
    mem[3841] <= input2;
    add <= mem[3842];
    sub <= mem[3843];
    mlt <= mem[3844];
end

always @(*)
begin
    mem[3845] = input1; // temp
    mem[3846] = input2; // temp
end

always @(*) begin // can't be merged by the top always because it might overwrite it as i tested it.
    if(ctrl[0] && ~ctrl[1]) data = mem[AR];
    else if(~ctrl[0] && ctrl[1]) mem[AR] = bus;
end
initial
begin
// Main program
mem[3844] = 16'b0;
mem[0] = 16'h5004;// calls the addition subroutine located at address 4.(BSA 4)
mem[1] = 16'h5009;//calls the subtraction located at 9(BSA 9)
mem[2] = 16'h5020;//calls the multiplication located at 32(BSA 32)
mem[3] = 16'h7001;// halts the computer

// Add
mem[4] = 16'h0001;
mem[5] = 16'h2F00;
mem[6] = 16'h1F01;
mem[7] = 16'h3F02;
mem[8] = 16'hC004;

// Sub
mem[9] = 16'h0002;
mem[10] = 16'h2F01;
mem[11] = 16'h7200;
mem[12] = 16'h1F00;
mem[13] = 16'h7008;
mem[14] = 16'h7020;
mem[15] = 16'h7010;
mem[16] = 16'h7200;
mem[17] = 16'h3F03;
mem[18] = 16'hC009;

// Mul
mem[32] = 16'h0003;
mem[33] = 16'h7400;
mem[34] = 16'h2F05;
mem[35] = 16'h7080;
mem[36] = 16'h3F05;
mem[37] = 16'h7002;
mem[38] = 16'h4028; 
mem[39] = 16'h402C; 
mem[40] = 16'h2F06; 
mem[41] = 16'h1F04; 
mem[42] = 16'h3F04; 
mem[43] = 16'h7400; 
mem[44] = 16'h2F06; 
mem[45] = 16'h7040; 
mem[46] = 16'h3F06; 
mem[47] = 16'h6F07; 
mem[48] = 16'h4021; 
mem[49] = 16'hC020;
mem[3847] = 16'hFFF5; 
end
endmodule


module dataPath(
    input  clk,
    input  [1:0]  mem, E_ctrl,
    input  [2:0]  AR_ctrl, s, PC_ctrl, DR_ctrl, AC_ctrl, TR_ctrl, alu_s,
    input  IR_load,
    input  [15:0] input1, input2,
	output wire [15:0] bus, mem_data, add, sub, mlt,
    output [11:0] AR, PC,
    output[15:0] DR, AC, IR, TR,
    output E
);
wire [15:0] AC_input;
DataBus bus1(s, bus, mem_data, AR, PC, DR, AC, IR, TR, clk);
MemoryUnit memory1(mem_data, AR, mem, bus, input1, input2, clk, add, sub, mlt);
PC PC1(clk, PC, bus, PC_ctrl);
AR AR1(clk, AR, bus, AR_ctrl);
DR DR1(clk, DR, bus, DR_ctrl);
AC AC1(clk, AC, AC_input, AC_ctrl);
IR IR1(clk, IR, bus, IR_load);
TR TR1(clk, TR, bus, TR_ctrl);
alu alu1(clk, AC_ctrl, E, DR, AC, AC_input, alu_s, E_ctrl);

endmodule

module cpu(
    input clk,
    input  [15:0] input1, input2,
    output [15:0] add, sub, mlt
);

wire E, IR_load;
wire [1:0] E_ctrl, mem;
wire [2:0] alu_s, AR_ctrl, s, PC_ctrl, DR_ctrl, AC_ctrl, TR_ctrl;
wire [11:0] AR, PC;
wire [15:0] bus, mem_data, DR, AC, IR, TR;

ControlUnit controlunit1(
    .clk(clk),.E(E),.AC(AC),.IR(IR),.DR(DR),.mem_ctrl(mem),.s(s),.PC_ctrl(PC_ctrl),
	.AR_ctrl(AR_ctrl),.DR_ctrl(DR_ctrl),.AC_ctrl(AC_ctrl),.IR_load(IR_load),.TR_ctrl(TR_ctrl),.alu_s(alu_s),.E_ctrl(E_ctrl)
);

dataPath datapath1(
    .clk(clk), .bus(bus), .mem(mem), .mem_data(mem_data),.AR_ctrl(AR_ctrl), .AR(AR), .s(s), .PC_ctrl(PC_ctrl), .PC(PC),
    .DR_ctrl(DR_ctrl), .DR(DR), .AC_ctrl(AC_ctrl), .AC(AC),.IR_load(IR_load), .IR(IR), .TR_ctrl(TR_ctrl), .TR(TR),
    .E(E), .alu_s(alu_s), .E_ctrl(E_ctrl),.input1(input1), .input2(input2), .add(add), .sub(sub), .mlt(mlt)
);
endmodule

module testbench_cpu;
reg clk;
reg [15:0] input1, input2;
wire [15:0] add, sub, mlt;

cpu cpu1(.clk(clk), .input1(input1), .input2(input2), .add(add), .sub(sub), .mlt(mlt));
always #20 clk = ~clk;
initial
begin
clk = 1'b0;
input1 = 50;
input2 = 30;
$monitor("input1: %d, input2: %d | add: %d, sub: %d, mlt: %d", input1, input2, add, sub, mlt);
end
endmodule