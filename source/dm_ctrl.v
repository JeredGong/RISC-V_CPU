`timescale 1ns / 1ps


module dm_con(mem_w, Addr_in, Data_write, dm_ctrl, 
  Data_read_from_dm, Data_read, Data_write_to_dm, wea_mem);  
  
  input mem_w;
  input [31:0]Addr_in;
  input [31:0]Data_write;
  input [2:0]dm_ctrl;
  input [31:0]Data_read_from_dm;
  output reg[31:0]Data_read;
  output reg[31:0]Data_write_to_dm;
  output reg[3:0]wea_mem;
  
  `define dm_word 3'b000
       `define dm_halfword 3'b001
       `define dm_halfword_unsigned 3'b010
       `define dm_byte 3'b011
       `define dm_byte_unsigned 3'b100
  
        always @(*) begin
         case (mem_w)
         // for store
         1'b1: begin
             case (dm_ctrl)
           
             `dm_word: begin
             Data_read = Data_read_from_dm;    //useless
             Data_write_to_dm = Data_write;   
             wea_mem = 4'b1111;  
         end

            `dm_halfword_unsigned, `dm_halfword : begin
             Data_read = Data_read_from_dm;   
             if (Addr_in[1] == 1'b0) begin 
             wea_mem = 4'b0011;  
             Data_write_to_dm[31:16] = 16'b0;  
                 Data_write_to_dm[15:0] = Data_write[15:0];    
                 
                 
             end 
             else begin 
                 Data_write_to_dm[15:0] = 16'b0;
                 Data_write_to_dm[31:16] = Data_write[15:0];   
                 wea_mem = 4'b1100; 
             end
         end

             `dm_byte, `dm_byte_unsigned: begin
             Data_read = Data_read_from_dm;  //no use   
             case (Addr_in[1:0])
               
                 2'b00: begin
                 Data_write_to_dm[7:0] = Data_write[7:0]; 
                 Data_write_to_dm[31:8] = 24'b0;  
                 wea_mem = 4'b0001; 
             end
              
                 2'b01: begin
                 Data_write_to_dm[7:0] = 8'b0;   
                 Data_write_to_dm[15:8] = Data_write[7:0]; 
                 Data_write_to_dm[31:16] = 16'b0; 
                 wea_mem = 4'b0010; 
             end
                 
                 2'b10: begin
                 Data_write_to_dm[15:0] = 16'b0;   
                 Data_write_to_dm[23:16] = Data_write[7:0];    
                 Data_write_to_dm[31:24] = 8'b0;   
                 wea_mem = 4'b0100; 
             end
                
                 2'b11: begin
                 Data_write_to_dm[23:0] = 24'b0;  
                 Data_write_to_dm[31:24] = Data_write[7:0];    
                 wea_mem = 4'b1000;
             end
             endcase 
         end 
             
                  
                 default: begin//to deal with undefined cases
                 Data_read = Data_read_from_dm;      
                 Data_write_to_dm = Data_write;     
                 wea_mem = 4'b0000; 
             end
             endcase
         end
/////for load
         1'b0: begin
         case (dm_ctrl)

             `dm_word: begin
             Data_read = Data_read_from_dm;   
             Data_write_to_dm = Data_write;   //useless  
             wea_mem = 4'b0000; 
         end
         
            
             `dm_halfword: begin//need sign ext
             if (Addr_in[1] == 1'b0) begin 
                 Data_read[31:0] ={{16{Data_read_from_dm[15]}}, Data_read_from_dm[15:0]}; 
             
             end else begin 
                 Data_read[31:0] ={{16{Data_read_from_dm[31]}}, Data_read_from_dm[31:16]}; 
               
             end 
             Data_write_to_dm = Data_write;      
             wea_mem = 4'b0000; 
         end 
 
             `dm_halfword_unsigned: begin 
             if (Addr_in[1] == 1'b0) begin 
                 Data_read[31:0] = {16'b0,Data_read_from_dm[15:0]};
                 //Data_read[31:16] = 16'b0; 
             end else begin 
                 Data_read[31:0] = {16'b0,Data_read_from_dm[31:16]};    
                // Data_read[31:16] = 16'b0; 
             end 
             Data_write_to_dm = Data_write;      
             wea_mem = 4'b0000; 
         end 

             `dm_byte: begin 
             case (Addr_in[1:0]) 
   
                 2'b00: begin 
                 Data_read[31:0] = {{24{Data_read_from_dm[7]}},Data_read_from_dm[7:0]};     
                 //Data_read[31:8] = {{24{Data_read_from_dm[7]}}}; 
             end 
                 
                 2'b01: begin 
                 Data_read[31:0] = {{24{Data_read_from_dm[15]}},Data_read_from_dm[15:8]}; 
                // Data_read[31:8] = {{24{Data_read_from_dm[15]}}}; 
             end 
                 
                 2'b10: begin 
                 Data_read[31:0] = {{24{Data_read_from_dm[23]}},Data_read_from_dm[23:16]};  
                 //Data_read[31:8] = {{24{Data_read_from_dm[23]}}}; 
             end 
 
                 2'b11: begin 
                 Data_read[31:0] ={{24{Data_read_from_dm[31]}}, Data_read_from_dm[7:0]};                   
                 //Data_read[31:8] = {{24{Data_read_from_dm[31]}}}; 
             end
             endcase
             Data_write_to_dm = Data_write;   
             wea_mem = 4'b0000; 
         end
    
             `dm_byte_unsigned: begin
             case (Addr_in[1:0])
      
                 2'b00: begin
                 Data_read[31:0] = {24'b0,Data_read_from_dm[7:0]}; 
                // Data_read[31:8] = 24'b0; 
             end
                 
                 2'b01: begin
                 Data_read[31:0] =  {24'b0,Data_read_from_dm[15:8]}; 
                // Data_read[31:8] = 24'b0; 
             end
                
                 2'b10: begin
                 Data_read[31:0] =  {24'b0,Data_read_from_dm[23:16]};  
                 //Data_read[31:8] = 24'b0; 
             end
                 
                 2'b11: begin
                 Data_read[31:0] =  {24'b0,Data_read_from_dm[31:24]};    
                 //Data_read[31:8] = 24'b0; 
             end
             endcase 
             Data_write_to_dm = Data_write;     
             wea_mem = 4'b0000; 
         end 
             
             default: begin 
             Data_read = Data_read_from_dm;     
             Data_write_to_dm = Data_write;     
             wea_mem = 4'b0000; 
             end
             endcase
         end 
     endcase 
       //        
   
   end 
 
  endmodule
    //lb   101 sb 001
    //lh   110 sh 011
    //lw   100 sw 111
    //lbu 000   lhu 010
    //for load to assign wea_mem
    //wire [3:0]which_byte;
    //assign which_byte={dm_ctrl[2],dm_ctrl[2:0]};
    /*wire [3:0]l_s_tag;
    assign l_s_tag={mem_w,mem_w,mem_w,mem_w};
    assign wea_mem[3:0]=which_byte[3:0]&l_s_tag;
    */
  
    /*always@(*)begin
      case(dm_ctrl)
              //LB 
              3'b000:begin
                  assign wea_mem=4'b0000;//no byte to write
                  case (Data_read_from_dm[7])//load byte with extension
                      1'b0:assign Data_read[31:0]={24'b0,Data_read_from_dm[7:0]};
                      1'b1:assign Data_read[31:0]={24'b1,Data_read_from_dm[7:0]};
                  endcase
   
              end
              //LH 
              3'b001:begin
                              assign wea_mem=4'b0000;//no byte to write
                              case (Data_read_from_dm[15])//load byte with extension
                                  1'b0:assign Data_read[31:0]={16'b0,Data_read_from_dm[16:0]};
                                  1'b1:assign Data_read[31:0]={16'b1,Data_read_from_dm[16:0]};
                              endcase
              end             
              //LW  
              3'b010:begin
                              assign wea_mem=4'b0000;//no byte to write
                              assign Data_read[31:0]=Data_read_from_dm[31:0];
               end               
              //LBU 
              3'b011:begin
                              assign wea_mem=4'b0000;//no byte to write
                              assign Data_read[31:0]={24'b0,Data_read_from_dm[7:0]};
              end                
              //LHU 
              3'b100:begin
                              assign wea_mem=4'b0000;//no byte to write
                              assign Data_read[31:0]={16'b0,Data_read_from_dm[16:0]};
              end 
                 
              //SB 
              3'b101:begin
                              assign wea_mem=4'b0001;//no byte to write
                              assign Data_write_to_dm[31:0]={24'b0,Data_write[7:0]};
                                        
              end    
              //SH  
              3'b110:begin
                                          assign wea_mem=4'b0000;//no byte to write
                                          case (Data_read_from_dm[7])//load byte with extension
                                              1'b0:assign Data_read[31:0]={24'b0,Data_read[7:0]};
                                              1'b0:assign Data_read[31:0]={24'b0,Data_read[7:0]};
                                          endcase
                          end    
              //SW  
              3'b111:begin
                                          assign wea_mem=4'b0000;//no byte to write
                                          case (Data_read_from_dm[7])//load byte with extension
                                              1'b0:assign Data_read[31:0]={24'b0,Data_read[7:0]};
                                              1'b0:assign Data_read[31:0]={24'b0,Data_read[7:0]};
                                          endcase
                          end    
  endcase 
  end
*/

