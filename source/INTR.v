`timescale 1ns / 1ps

module INTR(
    IF_PC,ID_PC,EXE_PC,MEM_PC,WB_PC,
    /*for addr misaligned & instr|load|store access fault*/
    EXE_aluout,
    EXE_readORwrite,
    EXE_dm_ctrl,
    
    
    ID_flush,
    EXE_flush,
    INTR_PC
    );
    output reg ID_flush,EXE_flush;
    output[31:0]INTR_PC;
    reg [31:0]mcause;// [31]for INTR or EXCEP,[31:0]for actual cause  //cause refers toRISCV Reader-Chinese-v2p1 P102
    reg [31:0]mepc;  //precise INTR instruction PC
    reg [31:0]mip;//INTR pending,,serve as priority table 
    reg [31:0]mie;//INTR that can be handled  at present
    reg [31:0]mtval;// additional trap info
    reg [31:0]mtvec;//PC that cur INTR jumps to 
    reg [31:0]mscratch;//things need to preserve
    reg [31:0]mstatus;//global status [3]for mie i.e global interrupt enable
    reg [31:0]mep;
    input [31:0]IF_PC,ID_PC,EXE_PC,MEM_PC,WB_PC;
    
    
   
         integer RAM_size=1024,ROM_size=1024;
         input [31:0]EXE_aluout;
         input [1:0]EXE_readORwrite;//[1]for read,[0]for write
        input [2:0]EXE_dm_ctrl;
    
     /*for inst access fault in IF*/wire ROM_overflow=($unsigned(ROM_size)<$unsigned(IF_PC));
    
    /*instr addr misaligned in IF*/wire instr_addr_misaligned=IF_PC[1]|IF_PC[0];
        wire RAM_overflow=($unsigned(RAM_size)<$unsigned(EXE_aluout));
    /*load access fault in EXE*/wire load_overflow=ROM_overflow&EXE_readORwrite[1]; 
    /*store access fault in EXE*/wire store_overflow=ROM_overflow&EXE_readORwrite[0];
    /*load/store addr misaligned in EXE*/
        wire word_misaligned=EXE_aluout[0]|EXE_aluout[1]; 
        wire halfword_misaligned=EXE_aluout[0]; 
        wire is_misaligned=(word_misaligned&(EXE_dm_ctrl==3'b000))|( halfword_misaligned&(EXE_dm_ctrl[0]^EXE_dm_ctrl[1]));
    wire load_misaligned=is_misaligned&EXE_readORwrite[1];
    wire store_misaligned=is_misaligned&EXE_readORwrite[0];
    
    

   
    wire MEIP,SEIP,MTIP,STIP,MSIP,SSIP;//need to assign
    always@(*)begin//update by time?
        mip={20'b0,MEIP,1'b0,SEIP,1'b0,MTIP,1'b0,STIP,1'b0,MSIP,1'b0,SSIP};  
        mep={store_misaligned,load_overflow,store_overflow,load_misaligned,ROM_overflow,instr_addr_misaligned};
 
    end
    
    always@(*)begin
    if (mstatus[3]==1&&mip&&mep)begin//only when INTR&EXCE&enable can procede
    mstatus[3]=0;//prevent INTR occured during current INTR
    //MEIP
        if (mip[10]==1)begin
        
        end
    //SEIP
        else if(mip[8]==1)begin
        
        end
    //MTIP
        else if(mip[6]==1)begin
        
         end   
     //STIP
        else if(mip[4]==1)begin
        
         end    
     //MSIP
        else if(mip[2]==1)begin
        
         end
     //SSIP
        else if(mip[0]==1)begin
        
         end 
         else if(mep) begin
               if(store_misaligned)begin
                    mepc=EXE_PC;
                    mcause=8'h00000006;
                    mep[6]=0;//remove this pending
                    mie=8'h00000020;//seem to be meaningless    also need to be recover
                    mtval=EXE_aluout;//store the misaligned addr
                     ID_flush=1;
                    EXE_flush=1;
               end
               if(load_overflow)begin
                    mepc=EXE_PC;
                    mcause=8'h00000005;
                    mep=mep&8'hffff_ffdf;//remove this pending
                    mie=~(8'hffff_ffdf);//seem to be meaningless    also need to be recover
                    mtval=EXE_aluout;//store the overflow addr
                    ID_flush=1;
                    EXE_flush=1;
               end               
         end  
         
         mtvec=mcause<<2;//INTR table   Q:how to organize instruction order in coe  ,and how to start the cpu
         
         
         
         end //if mie  
    end//always
   
   
     
   

endmodule
