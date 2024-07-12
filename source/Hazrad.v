

module Hazard(
    ID_EXE_mr,ID_EXE_rd,rs1,rs2,stall
    );
    input ID_EXE_mr;
    input [4:0]ID_EXE_rd,rs1,rs2;
    output stall;
    
    assign datahazard_m=(rs1==ID_EXE_rd)||(rs2==ID_EXE_rd);
    assign stall=ID_EXE_mr&datahazard_m;
    
endmodule
