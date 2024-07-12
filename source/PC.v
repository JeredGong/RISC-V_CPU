module PC( clk, rst,stall, NPC, NPCOp,PC );

  input              clk;
  input              rst;
  input         stall;
  input       [31:0] NPC;
  input [2:0]NPCOp;
  output reg  [31:0] PC;
  
 
  always @(negedge clk, posedge rst)
    if (rst) begin
      PC <= 32'h0000_0000;
    end
    else if(stall)
        PC<=PC;
    else if(NPCOp!=3'b000)
         PC <= NPC;
    else
        PC<=PC+4;

endmodule
