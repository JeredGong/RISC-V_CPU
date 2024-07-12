`include "ctrl_encode_def.v"
module NPC(PC, NPCOp, IMM, NPC,RD1,EXCE_PC,exce);  // next pc module
    
   input  [31:0] PC;        // pc
   input  [2:0]  NPCOp;     // next pc operation
   input  [31:0] IMM;       // immediate
	input [31:0] RD1;
	input [31:0]EXCE_PC;
	input exce;//flag of exception
   output reg [31:0] NPC;   // next pc
   
   wire [31:0] PCPLUS4;
   
   assign PCPLUS4 = PC + 4; // pc + 4
   
   always @(*) begin
   if (exce)NPC=EXCE_PC;
     else
      case (NPCOp)
          `NPC_PLUS4:  NPC = PCPLUS4;
          `NPC_BRANCH: NPC = PC+IMM;
          `NPC_JUMP:   NPC = PC+IMM;
		  `NPC_JALR:   NPC = RD1+IMM;
		  //`NPC_EXCE: NPC=EXCE_PC;discarded
          default:     NPC = PCPLUS4;
      endcase
   end // end always
   
endmodule
