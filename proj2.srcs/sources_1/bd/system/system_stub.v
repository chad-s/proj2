// Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2015.2 (lin64) Build 1266856 Fri Jun 26 16:35:25 MDT 2015
// Date        : Sun Feb  7 10:20:22 2016
// Host        : Sandcrawler running 64-bit Ubuntu 14.04.3 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/chad/vmshare/project/ece544/proj2/proj2.srcs/sources_1/bd/system/system_stub.v
// Design      : system
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module system(clk_100MHz_out, debug_out_tri_o, led_out, nexys4_btn_c_in, nexys4_btn_d_in, nexys4_btn_l_in, nexys4_btn_r_in, nexys4_btn_u_in, nexys4_swt_in, pmod_clp_data_out, pmod_clp_e_out, pmod_clp_rs_out, pmod_clp_rw_out, pmod_enc_a_in, pmod_enc_b_in, pmod_enc_press_in, pmod_enc_swt_in, pwm_out, rgb1_b_out, rgb1_g_out, rgb1_r_out, rgb2_b_out, rgb2_g_out, rgb2_r_out, sev_seg_annode_out, sev_seg_digits_out, sev_seg_points_out, sysclk, sysreset_n, uart_out_rxd, uart_out_txd)
/* synthesis syn_black_box black_box_pad_pin="clk_100MHz_out,debug_out_tri_o[3:0],led_out[15:0],nexys4_btn_c_in,nexys4_btn_d_in,nexys4_btn_l_in,nexys4_btn_r_in,nexys4_btn_u_in,nexys4_swt_in[15:0],pmod_clp_data_out[7:0],pmod_clp_e_out,pmod_clp_rs_out,pmod_clp_rw_out,pmod_enc_a_in,pmod_enc_b_in,pmod_enc_press_in,pmod_enc_swt_in,pwm_out,rgb1_b_out,rgb1_g_out,rgb1_r_out,rgb2_b_out,rgb2_g_out,rgb2_r_out,sev_seg_annode_out[7:0],sev_seg_digits_out[6:0],sev_seg_points_out,sysclk,sysreset_n,uart_out_rxd,uart_out_txd" */;
  output clk_100MHz_out;
  output [3:0]debug_out_tri_o;
  output [15:0]led_out;
  input nexys4_btn_c_in;
  input nexys4_btn_d_in;
  input nexys4_btn_l_in;
  input nexys4_btn_r_in;
  input nexys4_btn_u_in;
  input [15:0]nexys4_swt_in;
  output [7:0]pmod_clp_data_out;
  output pmod_clp_e_out;
  output pmod_clp_rs_out;
  output pmod_clp_rw_out;
  input pmod_enc_a_in;
  input pmod_enc_b_in;
  input pmod_enc_press_in;
  input pmod_enc_swt_in;
  output pwm_out;
  output rgb1_b_out;
  output rgb1_g_out;
  output rgb1_r_out;
  output rgb2_b_out;
  output rgb2_g_out;
  output rgb2_r_out;
  output [7:0]sev_seg_annode_out;
  output [6:0]sev_seg_digits_out;
  output sev_seg_points_out;
  input sysclk;
  input sysreset_n;
  input uart_out_rxd;
  output uart_out_txd;
endmodule
