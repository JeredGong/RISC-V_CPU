`include "ctrl_encode_def.v"

//123
module ctrl(Op, Funct7, Funct3, Zero, 
            RegWrite, MemWrite,
            EXTOp, ALUOp, NPCOp, 
            ALUSrc, GPRSel, WDSel,dm_ctrl,use_r1,use_r2,use_r3,illegal
            );
            
   input  [6:0] Op;       // opcode
   input  [6:0] Funct7;    // funct7
   input  [2:0] Funct3;    // funct3
   input        Zero;
   
   output       RegWrite; // control signal for register write
   output       MemWrite; // control signal for memory write
   output [6:0] EXTOp;    // control signal to signed extension
   output [4:0] ALUOp;    // ALU opertion
   output [2:0] NPCOp;    // next pc operation
   output       ALUSrc;   // ALU source for A
	output [2:0] dm_ctrl;// checked
   output [1:0] GPRSel;   // general purpose register selection
   output [1:0] WDSel;    // (register) write data selection
   output use_r1,use_r2,use_r3;//for foward unit to check if really need foward
   output illegal;// for illegal instruction exception
  // r format
    wire rtype  = ~Op[6]&Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0110011
    wire i_add  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // add 0000000 000
    wire i_sub  = rtype& ~Funct7[6]& Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sub 0100000 000
    wire i_or   = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]& Funct3[1]&~Funct3[0]; // or 0000000 110
    wire i_and  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]& Funct3[1]& Funct3[0]; // and 0000000 111
	wire i_sltu = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& Funct3[1]& Funct3[0]; 
	wire i_xor  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]&~Funct3[1]&~Funct3[0]; 
	wire i_srl  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]&~Funct3[1]& Funct3[0]; 
	wire i_sra  = rtype& ~Funct7[6]& Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]&~Funct3[1]& Funct3[0]; 
    wire i_sll  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]& Funct3[0]; 
    wire i_slt  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& Funct3[1]&~Funct3[0]; 

 // i format
   wire itype_l  = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0000011
	wire i_lb   =  itype_l&~Funct3[2]&~Funct3[1]&~Funct3[0]; 
	wire i_lbu   =  itype_l& Funct3[2]&~Funct3[1]&~Funct3[0]; 
	wire i_lh   =  itype_l&~Funct3[2]&~Funct3[1]& Funct3[0];
	wire i_lhu   =  itype_l& Funct3[2]&~Funct3[1]& Funct3[0]; 
	wire i_lw   =  itype_l&~Funct3[2]&	Funct3[1]&~Funct3[0]; 

// i format
    wire itype_r  = ~Op[6]&~Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0010011
    wire i_addi  =  itype_r& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // addi 000
    wire i_ori  =  itype_r& Funct3[2]& Funct3[1]&~Funct3[0]; // ori 110
	// 010
	wire i_slti  = itype_r&~Funct3[2]& Funct3[1]&~Funct3[0]; 
	 // 011
	 wire i_sltiu  = itype_r&~Funct3[2]& Funct3[1]& Funct3[0];
	// 100
	wire i_xori  = itype_r& Funct3[2]&~Funct3[1]&~Funct3[0]; 
	//111
	wire i_andi  = itype_r& Funct3[2]& Funct3[1]& Funct3[0]; 
	//001
	wire i_slli  = itype_r&~Funct3[2]&~Funct3[1]& Funct3[0]; 
	//101
	wire i_srli  = itype_r& Funct3[2]&~Funct3[1]& Funct3[0]&~Funct7[5]; 
	//010
	wire i_srai  = itype_r& Funct3[2]&~Funct3[1]& Funct3[0]& Funct7[5]; 
	 //jalr
	wire i_jalr =Op[6]&Op[5]&~Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];//jalr 1100111


  // s format
   wire stype  = ~Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//0100011
   wire i_sw   =  stype&~Funct3[2]& Funct3[1]&~Funct3[0]; // sw 010
	wire i_sh  =  stype&~Funct3[2]&~Funct3[1]& Funct3[0]; // sh 001
	wire i_sb  =  stype&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sb 000

  // sb format
   wire sbtype  = Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//1100011
   wire i_beq  = sbtype& ~Funct3[2]& ~Funct3[1]&~Funct3[0]; // beq 000
	wire i_bne  = sbtype& ~Funct3[2]& ~Funct3[1]& Funct3[0]; 
	
	wire i_blt  = sbtype&  Funct3[2]& ~Funct3[1]&~Funct3[0];
	 wire i_bltu  = sbtype&  Funct3[2]&  Funct3[1]&~Funct3[0];
	 
	wire i_bge  = sbtype&  Funct3[2]& ~Funct3[1]& Funct3[0]; 
	wire i_bgeu  = sbtype&  Funct3[2]&  Funct3[1]& Funct3[0]; 

  // lui
	wire i_lui  = ~Op[6]&Op[5]&Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];//0110111

  // auipc
	wire i_auipc  = ~Op[6]&~Op[5]&Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];//0010111
	
 // j format
   wire i_jal  = Op[6]& Op[5]&~Op[4]& Op[3]& Op[2]& Op[1]& Op[0];  // jal 1101111
  
 //csr format
 wire csr_type=Op[6]&Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];
 
 wire i_csrrc=csr_type&~Funct3[2]&Funct3[1]&Funct3[0];//011
 wire i_csrrci=csr_type&Funct3[2]&Funct3[1]&Funct3[0];//111
wire i_csrrs=csr_type&~Funct3[2]&Funct3[1]&~Funct3[0];//010
wire i_csrrsi=csr_type&Funct3[2]&Funct3[1]&~Funct3[0];//110
wire i_csrrw=csr_type&~Funct3[2]&~Funct3[1]&Funct3[0];//001  use alu_nop
wire i_csrrwi=csr_type&Funct3[2]&~Funct3[1]&Funct3[0];//101   use alu_lui

wire csrri_type= i_csrrci| i_csrrsi| i_csrrwi;//need to use imm

//illegal instuction dectect
 assign illegal=!(i_add|i_sub|i_or|i_and|i_sltu|i_xor|i_srl|i_sra|i_sll|i_slt|i_lb|i_lbu|i_lh|i_lhu| i_lw|
                           i_addi|i_ori|i_slti|i_sltiu|i_xori |i_andi|i_slli| i_srli|i_srai|
                           i_jalr|i_sw|i_sh|i_sb|
                            i_beq|i_bne|i_blt|i_bltu|i_bge|i_bgeu|
                            i_lui |i_auipc|i_jal| i_csrrc| i_csrrci|i_csrrs|i_csrrsi|i_csrrw|i_csrrwi);//43 inst
  // generate control signals
 assign RegWrite   = rtype | itype_l | itype_r | i_jalr | i_jal | i_lui | i_auipc|csr_type; // register write
  assign MemWrite   = stype;                           // memory write
  assign ALUSrc     = itype_l | itype_r | stype | i_jal | i_jalr | i_lui | i_auipc|csrri_type;   // ALU B is from instruction immediate


  // signed extension
  // EXT_CTRL_ITYPE_SHAMT 6'b100000
  // EXT_CTRL_ITYPE	      6'b010000
  // EXT_CTRL_STYPE	      6'b001000
  // EXT_CTRL_BTYPE	      6'b000100
  // EXT_CTRL_UTYPE	      6'b000010
  // EXT_CTRL_JTYPE	      6'b000001
  assign EXTOp[6]     =csrri_type;// also use as sign of  csr instruction
  assign EXTOp[5] 	 = i_slli | i_srli | i_srai;
  assign EXTOp[4]    = itype_l | i_ori | i_andi | i_jalr | i_addi | i_slti | i_sltiu | i_xori;  
  assign EXTOp[3]    = stype; 
  assign EXTOp[2]    = sbtype; 
  assign EXTOp[1]    = i_lui | i_auipc; 
  assign EXTOp[0]    = i_jal;         

  // WDSel_FromALU 2'b00
  // WDSel_FromMEM 2'b01
  // WDSel_FromPC  2'b10 
  //WDSel_FromCSR 2'b11
  assign WDSel[0] = itype_l|csr_type;
  assign WDSel[1] = i_jal | i_jalr|csr_type;

  // NPC_PLUS4   3'b000
  // NPC_BRANCH  3'b001
  // NPC_JUMP    3'b010
  // NPC_JALR	 3'b100
	assign NPCOp[0] = sbtype ;//& Zero;===========left to EXE to determin
	assign NPCOp[1] = i_jal;
	assign NPCOp[2] = i_jalr;
  
/*
`define ALUOp_nop 5'b00000
`define ALUOp_lui 5'b00001
`define ALUOp_auipc 5'b00010
`define ALUOp_add 5'b00011
`define ALUOp_sub 5'b00100
`define ALUOp_bne 5'b00101
`define ALUOp_blt 5'b00110
`define ALUOp_bge 5'b00111
`define ALUOp_bltu 5'b01000
`define ALUOp_bgeu 5'b01001
`define ALUOp_slt 5'b01010
`define ALUOp_sltu 5'b01011
`define ALUOp_xor 5'b01100
`define ALUOp_or 5'b01101
`define ALUOp_and 5'b01110
`define ALUOp_sll 5'b01111
`define ALUOp_srl,srli 5'b10000
`define ALUOp_sra,srai 5'b10001
`define ALUOp_and_neg 5'b10010
`define ALUOp_neg_and 5'b10011
*/
    
	assign ALUOp[0] = i_csrrwi|i_csrrs|i_csrrsi|itype_l | stype | i_addi | i_ori | i_add | i_or | i_lui | i_sll | i_sra | i_jalr | i_sltu |  i_sltiu | i_slli |i_srai | i_lui | i_bne | i_bge | i_bgeu|i_csrrc;
	assign ALUOp[1] = i_andi | i_jalr | itype_l | stype | i_addi | i_add | i_and | i_auipc | i_sll | i_jal | i_slt | i_sltu | i_slti | i_sltiu | i_slli | i_auipc | i_blt | i_bge|i_csrrc|i_csrrci;
	assign ALUOp[2] = i_csrrs|i_csrrsi|i_andi | i_and | i_ori | i_or | i_beq | i_sub | i_sll | i_xor | i_xori | i_slli |  i_blt | i_bge|i_bne;
	assign ALUOp[3] = i_csrrs|i_csrrsi|i_andi | i_and | i_ori | i_or | i_sll | i_xor | i_xori | i_slt | i_sltu | i_slti | i_sltiu | i_slli | i_bltu | i_bgeu;
	assign ALUOp[4] = i_srl | i_sra | i_srai | i_srli|i_csrrc|i_csrrci;


    assign dm_ctrl[2] = i_lbu;
    assign dm_ctrl[1] = i_lb | i_sb | i_lhu;
    assign dm_ctrl[0] = i_lh | i_sh | i_lb | i_sb;

    assign use_r1=rtype|itype_l|itype_r|i_jalr|stype|sbtype| i_csrrc|i_csrrs|i_csrrw;
    assign use_r2=rtype|stype|sbtype;
    assign use_r3=csr_type;
    
    
endmodule
