

module Forward(
    EX_MEM_WBSel,
    use_r1,use_r2,use_r3,
    rs1,rs2,rs3,
    rd1,rd2,rd3,
    MEM_WB_rs3,EX_MEM_rs3,
    EX_MEM_aluout,MEM_WB_wd,MEM_WB_csr,
    EX_MEM_rd,MEM_WB_rd, 
    EX_MEM_rw,MEM_WB_rw,
    EX_MEM_csrr,MEM_WB_csrr,
    douta,doutb,doutc
    );
    input [1:0]EX_MEM_WBSel;
    input use_r1,use_r2,use_r3;
    input [4:0]rs1;
    input [4:0]rs2;
    input [4:0]rs3;
    input [31:0]rd1,rd2,rd3,EX_MEM_aluout,MEM_WB_wd,MEM_WB_csr;
    input [4:0]EX_MEM_rd;
     input [4:0]MEM_WB_rd;
     input [4:0]MEM_WB_rs3;
     input [4:0]EX_MEM_rs3;
     input EX_MEM_rw,MEM_WB_rw,EX_MEM_csrr,MEM_WB_csrr;
     
     output [31:0]douta;
     output [31:0]doutb;
     output [31:0]doutc;
     
     wire datahazard_ex1=use_r1&(rs1==EX_MEM_rd)&(rs1!=5'b0)&EX_MEM_rw;
     wire datahazard_ex2=use_r2&(rs2==EX_MEM_rd)&(rs2!=5'b0)&EX_MEM_rw;
     wire datahazard_ex3=use_r3&(rs3==EX_MEM_rs3)&EX_MEM_csrr;
     
     wire datahazrad_me1=!datahazard_ex1&(rs1==MEM_WB_rd)&(rs1!=0)&MEM_WB_rw;
     wire datahazrad_me2=!datahazard_ex2&(rs2==MEM_WB_rd)&(rs2!=0)&MEM_WB_rw;
     wire datahazard_me3=!datahazard_ex3&(rs3==MEM_WB_rs3)&MEM_WB_csrr;
     
     wire foward_ex1= datahazard_ex1&(~EX_MEM_WBSel[0]);//if last is load,can't foward
     wire foward_ex2= datahazard_ex2&(~EX_MEM_WBSel[0]);
     wire foward_ex3= datahazard_ex3;
     
     wire foward_me1=datahazrad_me1;
     wire foward_me2=datahazrad_me2;
     wire foward_me3=datahazard_me3;
     
     assign douta=foward_ex1?EX_MEM_aluout:(foward_me1?MEM_WB_wd:rd1);
     assign doutb=foward_ex2?EX_MEM_aluout:(foward_me2?MEM_WB_wd:rd2);
     assign doutc=foward_ex3?EX_MEM_aluout:(foward_me3?MEM_WB_csr:rd3);
endmodule
