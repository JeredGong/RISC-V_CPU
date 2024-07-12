

module GRE_array(
    input clk,
    input rst,
    input write_enable,
    input flush,
    input [0:249] in,
    output reg[0:249] out);
    
    always@(posedge clk,posedge rst)begin///???????????
        if (rst)
                out=0;        
        else if(write_enable)begin
            if(flush)
                out=0;
            else
                 out=in;
        end
       
     end
      
    // always@(posedge rst) begin
     //       out=0;
    //     end
     
  
endmodule
