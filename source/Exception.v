`timescale 1ns / 1ps

module Exception(
        pc,cause,exce/*flag*/,tval,
        flush,NPC
    );
    input [31:0]pc,cause,tval;
    input exce;
    output flush;
    output [31:0]NPC;
    
    assign NPC=cause<<2;
    assign flush=exce;
endmodule
