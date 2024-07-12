`include "ctrl_encode_def.v"
module SCPU(
    input      clk,            // clock
    input      reset,          // reset
    input MIO_ready,
    input [31:0]  inst,     // instruction
    input [31:0]  Data_in,     // data from data memory
   
    output    mem_w,          // output: memory write signal
    output [31:0] PC_out,     // PC address
      // memory write
    output [31:0] Addr_out,   // ALU output
    output [31:0] Data_out,// data to data memory
    output [2:0] Dm_ctrl,
    output CPU_MIO,
    input INT
    
//output [2:0] DMType
);
assign MIO_ready=CPU_MIO;
    wire        RegWrite;    // control signal to register write
    wire MemWrite;
    wire [6:0]       EXTOp;       // control signal to signed extension
    wire [4:0]  ALUOp;       // ALU opertion
    wire [2:0]  NPCOp;       // next PC operation

    wire [1:0]  WDSel;       // (register) write data selection
    wire [1:0]  GPRSel;      // general purpose register selection
   
    wire        ALUSrc;      // ALU source for A
    wire        Zero;        // ALU ouput zero

    wire [31:0] NPC;         // next PC
    wire [4:0]  rs1;          // rs
    wire [4:0]  rs2;          // rt
    wire [11:0] rs3;// for csr
    wire [4:0]  rd;          // rd
    wire [6:0]  Op;          // opcode
    wire [6:0]  Funct7;       // funct7
    wire [2:0]  Funct3;       // funct3
    wire [11:0] Imm12;       // 12-bit immediate
    wire [31:0] Imm32;       // 32-bit immediate
    wire [19:0] IMM;         // 20-bit immediate (address)
    wire [4:0]  A3;          // register address for write
    reg [31:0] WD;          // register write data
    wire [31:0] RD1,RD2,RD3;         // register data specified by rs
    wire [31:0] B;           // operator for ALU B
	
	
	wire [4:0]zimm;
	wire [4:0] iimm_shamt;
	wire [11:0] iimm,simm,bimm;
	wire [19:0] uimm,jimm;
	wire [31:0] immout;
wire[31:0] aluout;

wire MEM_csrr;
wire [11:0]MEM_rs3;
wire[31:0]MEM_RD3;

 wire WB_csrr;
 wire [11:0]WB_rs3;
 wire [31:0]WB_RD3;   

	
	assign iimm_shamt=inst_in[24:20];
	assign iimm=inst_in[31:20];
	assign simm={inst_in[31:25],inst_in[11:7]};
	assign bimm={inst_in[31],inst_in[7],inst_in[30:25],inst_in[11:8]};
	assign uimm=inst_in[31:12];
	assign jimm={inst_in[31],inst_in[19:12],inst_in[20],inst_in[30:21]};
   assign zimm=inst_in[19:15];
   
    assign Op = inst_in[6:0];  // instruction
    assign Funct7 = inst_in[31:25]; // funct7
    assign Funct3 = inst_in[14:12]; // funct3
    assign rs1 = inst_in[19:15];  // rs1
    assign rs2 = inst_in[24:20];  // rs2
    
    assign rs3=inst_in[31:20];//csr addr
    assign rd = inst_in[11:7];  // rd
    assign Imm12 = inst_in[31:20];// 12-bit immediate
    assign IMM = inst_in[31:12];  // 20-bit immediate
   wire [2:0]final_NPCOp={EXE_NPCOp[2:1],EXE_NPCOp[0]&Zero};
   wire jump_flush=(final_NPCOp!=3'b000);//when NPCOp!=000,jump
   wire stall;//not linked over
   
   
   
   
 
 // instantiation of pc unit
//IF====================================================
	PC U_PC(.clk(clk), .rst(reset),.stall(stall),.NPCOp(final_NPCOp) ,.NPC(NPC), .PC(PC_out) );//stall who gives
    /*exception*/	
    wire [31:0]IF_mtval=PC_out;
    wire [31:0] IF_mcause=32'b0;//currently, only pc misaligned in IF
    wire IF_PC_misaligned=PC_out[0]|PC_out[1];
    wire IF_exce=IF_PC_misaligned;// bus,  left place for expansion
//ID	=================================================================
	wire [31:0]ID_EX_PC;//ID_EX
	wire [31:0]inst_in;//mark,this is not acutally inst_in,but ramain the original for convenience
	wire [2:0]ddm_ctrl;
    /*exception*/
    wire [31:0]ID_pre_mcause;//used to connect IF_mcause,followed as well
    wire [31:0]ID_pre_mtval;
    wire [31:0]ID_pre_exce;
    
    wire [31:0]ID_mtval;//cur only illegal inst
    wire [31:0]ID_mcause;
    wire ID_illegal;//linked to ctrl
    wire ID_exce;
	GRE_array  IF_ID(clk,reset, !stall ,1'b0|exce_flush ,{IF_mtval,IF_mcause,IF_exce,                        PC_out[31:0],inst[31:0]},
	                                                                   {ID_pre_mtval,ID_pre_mcause,ID_pre_exce,ID_EX_PC[31:0],inst_in[31:0]});  
	                                                                   
	// instantiation of control unit
	wire use1,use2,use3;
	ctrl U_ctrl(
		.Op(Op), .Funct7(Funct7), .Funct3(Funct3), /*.Zero(branch),*/
		.RegWrite(RegWrite)/*pass to WB*/, .MemWrite(MemWrite)/*pass to MEM*/,
		.EXTOp(EXTOp)/*used below*/, .ALUOp(ALUOp)/*pass to EXE*/, .NPCOp(NPCOp)/*used below*/, 
		.ALUSrc(ALUSrc)/*pass to EXE*/, .GPRSel(GPRSel)/*unused*/, .WDSel(WDSel)/*pass to WB*/,.dm_ctrl(ddm_ctrl)/*pass to MEM*/
		,.use_r1(use1),.use_r2(use2)/*pass to EXE*/,.use_r3(use3)/*pass to EXE*/,.illegal(ID_illegal)
	);//remove zero for prediction executed in IF_ID
	assign ID_exce=ID_illegal|ID_pre_exce;
	assign ID_mcause=ID_pre_exce?ID_pre_mcause:8'h0000_0002;
	assign ID_mtval=ID_pre_exce?ID_pre_mtval:inst_in;
	EXT U_EXT(
		.iimm_shamt(iimm_shamt), .iimm(iimm), .simm(simm), .bimm(bimm),
		.uimm(uimm), .jimm(jimm),.zimm(zimm),
		.EXTOp(EXTOp), .immout(immout)
	);
	CSR U_csr(.clk(clk),
	.rst(reset),
	.RFWr(WB_csrr),//need EXTOp in WB
	.A1(rs3),
	.A2(WB_rs3),//use addr in WB
	.RD1(RD3),
	.WD(MEM_ID_band[95:64])//use new csr in WB,in fact ,its aluout
	);
	//forward jalr's RD1+IMM to NPC
	RF U_RF(
		.clk(clk), .rst(reset),
		.RFWr(MEM_ID_band[107]), //useMEM_WB
		.A1(rs1), .A2(rs2), 
		.A3(MEM_ID_band[100:96]), //use MEM_WB
		.WD(WD), //use MEM_WB
		.RD1(RD1), .RD2(RD2)
	);
	Hazard hazard(.ID_EXE_mr(ID_EXE_band[8]),.ID_EXE_rd(ID_EXE_band[4:0]),.rs1(rs1),.rs2(rs2),.stall(stall));
  //wire branch;
  wire [4:0]write_addr=inst_in[11:7];
  //alu_branch(RD1,RD2,ALUOp,branch);
// instantiation of alu unit
//EXE================================================================================
   wire [146:0] ID_EXE_band;
   wire use_r1,use_r2,use_r3;//check if really need foward
   wire [4:0]EXE_rs1,EXE_rs2;
   wire [11:0]EXE_rs3;
   wire [31:0]douta,doutb,doutc;
   wire [2:0]EXE_NPCOp;
   wire [31:0]EXE_RD3;
   wire csrri_type;
   wire [31:0]Foward_ina,Foward_inb;
   /*exception*/
    wire [31:0]EX_pre_mcause;//used to connect IF_mcause,followed as well
    wire [31:0]EX_pre_mtval;
    wire [31:0]EX_pre_exce;
    
    wire [31:0]EX_mtval;
    wire [31:0]EX_mcause;
    wire EX_misaligned;
    wire EX_load_mialigned,EX_store_misaligned;//linked to ctrl
    wire EX_exce;
    GRE_array  ID_EX(clk,reset, 1'b1 ,stall |jump_flush|exce_flush ,
                                    {ID_mtval,ID_mcause,ID_exce,                         rs3,          EXTOp[6]/*flag of csrri*/,use3/*flag of csrr*/,RD3[31:0],NPCOp[2:0],rs1,rs2,use1,use2,RD1[31:0]/*146:115*/,RD2[31:0]/*114:83*/,immout[31:0],ID_EX_PC[31:0],ALUOp[4:0],1'b0/*just lazy to revise*/,ALUSrc,RegWrite,MemWrite,WDSel[1:0],ddm_ctrl[2:0],write_addr[4:0]},
                                    {EX_pre_mtval,EX_pre_mcause,EX_pre_exce,EXE_rs3,csrri_type,                        use_r3,                    EXE_RD3,EXE_NPCOp,EXE_rs1,EXE_rs2,use_r1,use_r2,ID_EXE_band});
    // assign B = (ID_EXE_band[12]) ? ID_EXE_band[82:51] : ID_EXE_band[114:83];//assign B = (ALUSrc) ? immout : RD2;
    // alu U_alu(.A(ID_EXE_band[146:115]), .B(B), .ALUOp(ALUOp), .C(aluout), .Zero(Zero), .PC(PC_out));
   //assign Foward_ina=use_r3?(csrri_type?RD3[31:0]:):ID_EXE_band[146:115];

     Forward forward(
    .EX_MEM_WBSel(EX_ME_band[105:104]),
    .use_r1(use_r1),
    .use_r2(use_r2),
    .rs1(EXE_rs1),
    .rs2(EXE_rs2),
    .rd1(ID_EXE_band[146:115]),
    .rd2(ID_EXE_band[114:83]),
    .EX_MEM_aluout(EX_ME_band[95:64]),
    .MEM_WB_wd(WD),
    .EX_MEM_rd(EX_ME_band[100:96]),
    .MEM_WB_rd(MEM_ID_band[100:96]),
    .EX_MEM_rw(EX_ME_band[107]),
    .MEM_WB_rw(MEM_ID_band[107]),
    .douta(douta),
    .doutb(doutb),
    //for csrr insruction
    .use_r3(use_r3),
    .rs3(EXE_rs3),
    .rd3(EXE_RD3),
    .EX_MEM_csrr(MEM_csrr),
    .MEM_WB_csrr(WB_csrr),
     .MEM_WB_rs3(MEM_rs3),
     .EX_MEM_rs3(WB_rs3),
     .doutc(doutc)
    ); 
    wire [31:0]alua,alub;
    assign B = (ID_EXE_band[12]) ? ID_EXE_band[82:51] :doutb;//assign B = (ALUSrc) ? immout : RD2;
    assign alua=use_r3?(csrri_type?doutc:douta):douta;//when csrri, rs1=t,rs2=imm
    assign alub=use_r3?(csrri_type?B:doutc):B;
    NPC U_NPC(.PC(ID_EXE_band[50:19]), .NPCOp(final_NPCOp), .IMM(ID_EXE_band[82:51]), .NPC(NPC), .RD1(douta),.EXCE_NPC(exce_NPC),.exce(EX_exce));//
	alu U_alu(.A(alua), .B(alub), .ALUOp(ID_EXE_band[18:14]), .C(aluout), .Zero(Zero), .PC(ID_EXE_band[50:19]));
	
	wire word_misaligned=(ID_EXE_band[7:5]==3'b000/*word*/)&(aluout[0]|aluout[1]);//use dm_ctrl to judge
	wire half_misaligned=(ID_EXE_band[6]^ID_EXE_band[7]/*h | hu*/)&aluout[0];
	assign EX_misaligned=word_misaligned|half_misaligned;
	assign EX_load_mialigned=EX_misaligned&(ID_EXE_band[9:8]==2'b01);//use WD_Sel to judge
	assign EX_store_misaligned=EX_misaligned&ID_EXE_band[10];//use memWrite to judge
	 assign EX_exce=EX_load_mialigned|EX_store_misaligned|EX_pre_exce;
	assign EX_mcause=EX_pre_exce?EX_pre_mcause:(EX_store_misaligned?8'h0000_0006:8'h0000_0004);//default would assign load misaligned cause,won't matter for EX_exce
	assign EX_mtval=EX_pre_exce?EX_pre_mtval:aluout;
	wire exce_flush;
	wire exce_NPC;
	Exception U_exce(.pc(ID_EXE_band[50:19]),
	                                .cause(EX_mcause),
	                                .tval(EX_mtval),
	                                .flush(exce_flush),
	                                .NPC(exce_NPC));
//MEM=======================================================================

wire [107:0]EX_ME_band;

GRE_array  EX_ME(clk,reset, 1'b1  ,1'b0|exce_flush   ,
                                    {doutc/*csr after foward*/,EXE_rs3,use_r3,    ID_EXE_band[11:0],aluout[31:0], ID_EXE_band[50:19]/*PC*/,doutb/*ID_EXE_band[114:83]RD2*/},
                                    {MEM_RD3                      ,MEM_rs3,MEM_csrr,EX_ME_band});
  assign Addr_out=EX_ME_band[95:64];//aluout
    assign Data_out = EX_ME_band[31:0];  //rd2  
    assign mem_w=EX_ME_band[106];  
    assign Dm_ctrl[2:0]=EX_ME_band[103:101];   
 //WB        ===================================================================================   
 wire [107:0]MEM_ID_band;   
           
GRE_array  MEM_ID(clk,reset,1'b1  ,  1'b0,
                                    {MEM_RD3,MEM_csrr,MEM_rs3,EX_ME_band[107:32], Data_in},
                                    {WB_RD3,WB_csrr,WB_rs3,MEM_ID_band});
                                    
always @(*)//???????
begin
	case(MEM_ID_band[105:104])//WDSel
		`WDSel_FromALU: WD<=MEM_ID_band[95:64];//aluout
		`WDSel_FromMEM: WD<=MEM_ID_band[31:0];//data_in
		`WDSel_FromPC: WD<=MEM_ID_band[63:32]+4;//pc+4
		`WDSel_FromCSR:WD<=WB_RD3;
	endcase
end


endmodule