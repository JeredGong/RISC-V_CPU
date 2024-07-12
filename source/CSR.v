`timescale 1ns / 1ps
`define mstatus   12'b001100000000
`define mie          12'b001100000100
`define mtvec      12'b001100000101
`define mscratch 12'b001101000000
`define mepc       12'b001101000001
`define mcause   12'b001101000010
`define mtval        12'b001101000011
`define mip           12'b001101000100

module CSR(
 input         clk, 
               input         rst,
               input         RFWr, 
               input  [11:0]  A1, A2, //addr in 12 bit for csr
               input  [31:0] WD, 
               output [31:0] RD1
    );
        reg [31:0] csr[15:0];
        integer i;
        always @(negedge clk, posedge rst)
    if (rst) begin    //  reset
      for (i=1; i<16; i=i+1)
        csr[i] <= 0; //  i
    end
      
    else 
      if (RFWr) begin
        csr[{A1[6],A1[2:0]}] <= WD;
             $display("x%d = %h", A2, WD);
      end
      
      
      assign RD1 =csr[{A1[6],A1[2:0]}];
endmodule
