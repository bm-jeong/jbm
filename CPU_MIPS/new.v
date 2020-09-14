`timescale 1ns/10ps
module tb();

    reg CLK, RESET;
    wire [31:0] DATA;

    localparam PER = 20.0;
    localparam HPER = PER / 2;

    top top(
       .clk	(CLK    ),
       .reset	(RESET	),
       .data	(DATA	)
   ); 

    initial begin
	CLK = 0;
	RESET = 1;
	#(PER*5) RESET = 0;
    end

    always #(HPER) CLK <= ~CLK;

    initial begin
	$dumpfile("tb.vcd");
	$dumpvars(0, tb);
	#(PER*500) $finish;
    end
endmodule

module top(
    input clk, reset,
    output [31:0] data
);

    wire [31:0] instr;
    wire [5:0] addr;

    imem imem(
	.addr	(addr	),
	.instr	(instr	)
    );

    mips mips(
	.clk	(clk	),
	.reset	(reset	),
	.instr	(instr	),
	.addr	(addr	),
	.data	(data	)
    );

endmodule

module imem(
    input [5:0] addr,
    output [31:0] instr
);

    reg [31:0] ram[0:63];
    
    initial
	$readmemh("memfile.dat", ram);

    assign instr = ram[addr/4];


endmodule

module mips(
    input clk, reset,
    input [31:0] instr,
    output [31:0] data,
    output [5:0] addr	// pc addr
);

    wire [2:0] control;
    wire [31:0] b;
    
    // pc set
    reg [31:0] pc;

    always @(posedge clk) begin
	if(reset)
	    pc <= 0;
	else
	    pc <= pc + 4;
    end    

    assign addr = pc[5:0];

    // connect to regfile
    wire regwrite;
    wire [31:0] rd1, rd2;
    wire [31:0] writedata;

    // DUT SET
    regfile register(
	.rs	(instr[25:21]	),
	.rt	(instr[20:16]	),
	.rd	(instr[15:11]	),  // check !!!!
	.rd1	(rd1		),
	.rd2	(rd2		),
	.regwrite   (regwrite	)	
    );

    controller controller(
	.opcode	(instr[31:26]	),
	.funct	(instr[5:0]	),
	.rddst	(rddst		),
	.alusrc	(aluscr		),
	.memwrite   (memwrite	),
	.memread    (memread	),
	.jump	    (jump	),
	.branch	    (branch	),
	.regwrite   (regwrite	),
	.control    (control	)
    );

    alu alu(
	.control(control),
	.a	(rd1	),
	.b	(b	),  // check
	.result	(data	)
    );

    sing_ext ext(





endmodule


//CHECK AFTER!!!
module regfile(
    input regwrite,
    input [4:0] rs, rt, rd,
    input [31:0] writedata, // check
    output [31:0] rd1, rd2
);



endmodule
//CHECK AFTER!!!!


module controller(
    input [5:0] opcode, funct,
    output rddst, alusrc, memwrite, memread,
    output jump, branch, regwrite,
    output [2:0] control
);
    wire [1:0] aluop;

    dec dec(
	.opcode	(opcode		),
	.rddst	(rddst		),
	.alusrc	(aluscr		),
	.memwrite   (memwrite	),
	.memread    (memread	),
	.jump	    (jump	),
	.branch	    (branch	),
	.regwrite   (regwrite	),
	.aluop	    (aluop	)
    );

    aludec aludec(
	.funct	    (funct	),
	.aluop	    (aluop	),
	.control    (control	)
    );

endmodule

module dec(
    input [5:0] opcode,
    output rddst, alusrc, memwrite, memread,
    output jump, branch, regwrite,
    output [1:0] aluop
);

    reg [8:0] controls;

    always @* begin
	case(opcode)
	    6'h08 : controls = 9'b100000101;  // R type
	    6'h23: controls = 9'b010100100;  // lw
	    6'h2b: controls = 9'b011000000;  // sw
	    6'h04: controls = 9'b010001010;  // beq
	   // 6'h02: controls = 9'b01001010;  // jump
	endcase
    end

    assign {rddst, alusrc, memwrite, memread, jump, branch, regwrite, aluop} = controls;

endmodule

module aludec(
    input [1:0] aluop,
    input [5:0] funct,
    output [2:0] control
);

    reg [2:0] control;

    always @* begin
	case(aluop)
	    2'b00 : control = 3'b011;	// add (lw, sw)
	    2'b10 : control = 3'b111;	// sub (beq)
	    default : case(funct)
		6'h20 : control = 3'b011;   // add
		6'h22 : control = 3'b111;   // sub
		6'h24 : control = 3'b000;   // and
		6'h25 : control = 3'b001;   // or
		6'h2a : control = 3'b110;   // slt
	    endcase
	endcase
    end

endmodule

module alu(
    input [2:0] control,
    input [31:0] a,b,
    output [31:0] result
);

    wire [31:0] comple, sum;
    reg [31:0] result;

    assign comple = control[2] ? ~b : b;
    assign sum = a + comple + control[2];

    always @* begin
	case(control[1:0])
	    2'b11 : result = sum;
	    2'b00 : result = a & b;
	    2'b01 : result = a | b;
	    2'b10 : result = sum[31];
	endcase
    end	

    assign zero = (result == 32'b0);

endmodule

module sing_ext(
    input [15:0] offset,
    output [31:0] ext
);

    assign ext = offset[15] ? {16{offset[15],offset} : {0,offset};

endmodule

