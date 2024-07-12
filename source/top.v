`timescale 1ns / 1ps


module top(
  /* input rstn,
    input [4:0] btn_i,
    input [15:0] sw_i,
    input clk,
    */
    output [7:0] disp_an_o,
    output [7:0] disp_seg_o,
    output [15:0] led_o
    );
   //for debug
    reg [4:0] btn_i;
        reg [15:0] sw_i;
        reg clk;
        reg rstn;
    initial begin  
            clk=1;
            rstn=0;
            #50000;
            rstn=1;
          btn_i=5'b00000;
          sw_i=16'b0;
       end
    always begin
             #(50000) clk = ~clk;
    end      
  
    //wire BTN[4:0]=btn_i[4:0];
    //wire SW[15:0]=sw_i[15:0];
    wire rst=!rstn;
    //wire _;
    //assign _=32'bz;
//for enter
    wire [4:0]BTN_out;
    wire [15:0]SW_out;
//for clk_div
    wire Clk_CPU;
    wire [31:0]clkdiv;
//for SPIO
    //wire EN;///////////////////////////Checked  Need to assign
    wire [31:0]P_Data;///////////////// Checked Need to assign
    assign P_Data[31:0]=Peripheral_in[31:0];
    wire [1:0]counter_set;
    wire [15:0]LED_out;
    wire [15:0]led;
    wire [13:0]GPIOf0;
//for counter_x
    wire [31:0]counter_val;//////////// Checked Need to assign
    assign counter_val[31:0]=Peripheral_in[31:0];
    wire counter_we;
    wire counter0_OUT;
    wire counter1_OUT;
    wire counter2_OUT;
    //wire [31:0] counter_out;
//for SCPU
    wire MIO_ready;//alias CPU_MIO
    wire [31:0]inst_in;////////////// Cheked Need to assign
    assign inst_in[31:0]=spo[31:0];
    wire [31:0]Data_in;
    assign Data_in[31:0]=Data_read[31:0];
    wire mem_w;
    wire [31:0]PC_out;
    wire [31:0]Addr_out;
    wire [31:0]Data_out;
    wire [2:0]dm_ctrl;
    wire INT;
    assign INT=counter0_OUT;
//for dm_controller
    wire [31:0]Data_write;////////// Checked no Need to assign
    wire [31:0]Data_read_from_dm;//////// Checked Need to assign
    assign Data_read_from_dm[31:0]=Cpu_data4bus[31:0];
    wire [31:0]Data_read;
    wire [31:0]Data_write_to_dm;
    wire [3:0]wea_mem;
//for RAM_B
    wire [9:0]addra;
    wire [31:0]douta;
    wire [31:0]spo;
//for MIO_BUS
    wire [31:0]counter_out=32'b0;
    wire [31:0]Cpu_data4bus;
    wire [31:0]ram_data_in;
    wire [9:0]ram_addr;
    wire [31:0]Peripheral_in;
    wire GPIOf0000000_we;
    wire GPIOe0000000_we;
    //wire counter_we;
//for Multi_8CH32
    wire [63:0]LES;
    assign LES[63:0]=64'hffffffffffffffff;
    wire [31:0]data0;/////////////////////Checked
    assign data0[31:0]=Peripheral_in[31:0];
    wire[31:0]data1;
    assign data1[31:0]={2'b0,PC_out[31:2]};// word addr
    wire[31:0]data2;
    assign data2[31:0]=spo[31:0];
    wire[31:0]data3;
    assign data3[31:0]=0;//attached to GND???
    wire[31:0]data4;
    assign data4[31:0]=Addr_out[31:0];
    wire[31:0]data5;
    assign data5[31:0]=Data_out[31:0];
    wire[31:0]data6;
    assign data6[31:0]=Cpu_data4bus[31:0];
    wire[31:0]data7;
    assign data7[31:0]=PC_out[31:0];
    wire [7:0]point_out;
    wire [7:0]LE_out;
    wire [31:0]Disp_num;
//for SSeg7
    wire [7:0]seg_an;
    wire [7:0]seg_sout;    
    Enter U_enter(clk, btn_i[4:0], 
                  sw_i[15:0],  BTN_out[4:0],  
                  SW_out[15:0]);  
    clk_div U_clk_div(clk,rst,SW_out[2],
                      clkdiv[31:0],Clk_CPU);
    SPIO U_spio(!Clk_CPU, rst, GPIOf0000000_we, P_Data[31:0], 
                counter_set[1:0], LED_out[15:0], led[15:0],);//last output emitted
    Counter_x U_counter_X(!Clk_CPU,rst,clkdiv[6],clkdiv[9],
                          clkdiv[11],counter_we,counter_val[31:0],
                          counter_set[1:0], counter0_OUT,
                          counter1_OUT, counter2_OUT,);//last output emitted
    /*SCPU U_scpu(Clk_CPU, rst, MIO_ready, inst_in[31:0], 
                Data_in[31:0], mem_w, PC_out[31:0], Addr_out[31:0], 
                Data_out[31:0], dm_ctrl[2:0], MIO_ready, INT);
    */
    SCPU U_scpu(Clk_CPU, rst, MIO_ready, inst_in[31:0], 
                    Data_in[31:0], mem_w, PC_out[31:0], Addr_out[31:0], 
                    Data_out[31:0], dm_ctrl[2:0], MIO_ready, INT);     
    /*dm_controller U_dm_controller(mem_w, Addr_out[31:0], Data_write[31:0], dm_ctrl[2:0], 
                                  Data_read_from_dm[31:0], Data_read[31:0], Data_write_to_dm[31:0], 
                                  wea_mem[3:0]);
    */
    dm_con U_dm_conr(mem_w, Addr_out[31:0], Data_write[31:0], dm_ctrl[2:0], 
                                  Data_read_from_dm[31:0], Data_read[31:0], Data_write_to_dm[31:0], 
                                  wea_mem[3:0]);
                                  
    RAM_B U_ram_b(!clk, wea_mem[3:0] , addra[9:0] ,
                 Data_write_to_dm[31:0] ,douta[31:0] );
    ROM_D U_rom_d( PC_out[11:2] , spo[31:0] );// Checked io emitted
    MIO_BUS U_mio_bus(clk, rst, BTN_out[4:0], SW_out[15:0], , mem_w, Data_out[31:0], Addr_out[31:0], 
                      douta[31:0], LED_out[15:0], counter_out[31:0], counter0_OUT,
                      counter1_OUT, counter2_OUT, Cpu_data4bus[31:0], 
                      Data_write[31:0], addra[9:0], , GPIOf0000000_we, 
                      GPIOe0000000_we, counter_we, Peripheral_in[31:0]);//output emitted
    Multi_8CH32 U_multi_8ch32(!Clk_CPU, rst, GPIOe0000000_we, SW_out[7:5], {clkdiv[31:0],clkdiv[31:0]}, 
                              LES[63:0], data0[31:0], data1[31:0], data2[31:0], 
                              data3[31:0], data4[31:0], data5[31:0], data6[31:0], data7[31:0], 
                              point_out[7:0], LE_out[7:0], Disp_num[31:0]);
                              //Checked don't know how to map clkdiv[31;0] to point_in[64:0]
    SSeg7 U_sseg7(clk, rst, SW_out[0], clkdiv[10], Disp_num[31:0], point_out[7:0], LE_out[7:0], seg_an[7:0], seg_sout[7:0]);
    assign disp_an_o[7:0]=seg_an[7:0];
    assign disp_seg_o[7:0]=seg_sout[7:0];
    assign led_o[15:0]=led[15:0];
    
                
    
endmodule
