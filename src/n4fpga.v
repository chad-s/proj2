`timescale 1ns / 1ps
module n4fpga(
   input             clk,           // 100Mhz clock input
   input             btnC,          // center pushbutton
   input             btnU,          // UP (North) pusbhbutton
   input             btnL,          // LEFT (West) pushbutton
   input             btnD,          // DOWN (South) pushbutton  - used for system reset
   input             btnR,          // RIGHT (East) pushbutton
   input             btnCpuReset,   // CPU reset pushbutton
   input    [15:0]   sw,            // slide switches on Nexys 4
   output   [15:0]   led,           // LEDs on Nexys 4   
   output            RGB1_Blue,     // RGB1 LED (LD16) 
   output            RGB1_Green,
   output            RGB1_Red,
   output            RGB2_Blue,     // RGB2 LED (LD17)
   output            RGB2_Green,
   output            RGB2_Red,
   output   [7:0]    an,            // Seven Segment display
   output   [6:0]    seg,
   output            dp,

   input             uart_rtl_rxd,  // USB UART Rx and Tx on Nexys 4
   output            uart_rtl_txd,   

   output   [7:0]    JA,            // PmodCLP data bus
   output   [7:0]    JB,            // PmodCLP control signals, bot row only
   output   [7:0]    JC,            // Debug signals, bot row only
   inout    [7:0]    JD             // PmodENC signals, bot row
                                    // PWM feedback hardware, top row
);

   ///////////////////////////////////////////////
   // NETS : world
   ///////////////////////////////////////////////
   wire              sysclk;
   wire              clk_100MHz;
   wire              sysreset_n, sysreset;
   wire              rotary_a, rotary_b, rotary_press, rotary_sw;
   wire  [7:0]       lcd_data;
   wire              lcd_rs, lcd_rw, lcd_e;

   ///////////////////////////////////////////////
   // NETS : embedded system
   ///////////////////////////////////////////////
   wire              pwm_out;       // PWM output from the axi_timer
   wire  [3:0]       debug;

   ///////////////////////////////////////////////
   // INTEGRATION : PmodCLP
   ///////////////////////////////////////////////
   assign JA = lcd_data;
   assign JB = {1'b0, lcd_e, lcd_rw, lcd_rs, 4'b0000};

   ///////////////////////////////////////////////
   // INTEGRATION : PmodENC
   ///////////////////////////////////////////////
   assign rotary_a      = JD[5];
   assign rotary_b      = JD[4];
   assign rotary_press  = JD[6];
   assign rotary_sw     = JD[7];

   ///////////////////////////////////////////////
   // INTEGRATION : Feedback testbench
   ///////////////////////////////////////////////
   //assign light_int_fdback = JD[3];
   assign JD[3] = 1'b0;
   assign JD[2] = pwm_out;
   assign JD[1] = 1'b0;
   assign JD[0] = 1'b0;

   ///////////////////////////////////////////////
   // INTEGRATION : world
   ///////////////////////////////////////////////
   assign sysclk     = clk;
   assign sysreset_n = btnCpuReset;
   assign sysreset   = ~sysreset_n;

   // Debug signals are on both rows of JC
   assign JC = {debug, 4'b0000};

   ///////////////////////////////////////////////
   // SUBMODULES
   ///////////////////////////////////////////////
   system EMBSYS(
      .clk_100MHz_out(clk_100MHz),
      .debug_out_tri_o(debug),
      .led_out(led),
      .nexys4_btn_c_in(btnC),
      .nexys4_btn_d_in(btnD),
      .nexys4_btn_l_in(btnL),
      .nexys4_btn_r_in(btnR),
      .nexys4_btn_u_in(btnU),
      .nexys4_swt_in(sw),
      .pmod_clp_data_out(lcd_data),
      .pmod_clp_e_out(lcd_e),
      .pmod_clp_rs_out(lcd_rs),
      .pmod_clp_rw_out(lcd_rw),
      .pmod_enc_a_in(rotary_a),
      .pmod_enc_b_in(rotary_b),
      .pmod_enc_press_in(rotary_press),
      .pmod_enc_swt_in(rotary_sw),
      .pwm_out(pwm_out),
      .rgb1_b_out(RGB1_Blue),
      .rgb1_g_out(RGB1_Green),
      .rgb1_r_out(RGB1_Red),
      .rgb2_b_out(RGB2_Blue),
      .rgb2_g_out(RGB2_Green),
      .rgb2_r_out(RGB2_Red),
      .sev_seg_annode_out(an),
      .sev_seg_digits_out(seg),
      .sev_seg_points_out(dp),
      .sysclk(sysclk),
      .sysreset_n(sysreset_n),
      .uart_out_rxd(uart_rtl_rxd),
      .uart_out_txd(uart_rtl_txd)
   );

endmodule

