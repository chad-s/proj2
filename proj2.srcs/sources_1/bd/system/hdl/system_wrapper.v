//Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2015.2 (lin64) Build 1266856 Fri Jun 26 16:35:25 MDT 2015
//Date        : Mon Feb 15 02:19:55 2016
//Host        : Sandcrawler running 64-bit Ubuntu 14.04.3 LTS
//Command     : generate_target system_wrapper.bd
//Design      : system_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module system_wrapper
   (clk_100MHz_out,
    debug_out_tri_o,
    led_out,
    nexys4_btn_c_in,
    nexys4_btn_d_in,
    nexys4_btn_l_in,
    nexys4_btn_r_in,
    nexys4_btn_u_in,
    nexys4_swt_in,
    pmod_clp_data_out,
    pmod_clp_e_out,
    pmod_clp_rs_out,
    pmod_clp_rw_out,
    pmod_enc_a_in,
    pmod_enc_b_in,
    pmod_enc_press_in,
    pmod_enc_swt_in,
    pwm_det_ref_clk_in,
    pwm_det_reset_in,
    pwm_out,
    rgb1_b_out,
    rgb1_g_out,
    rgb1_r_out,
    rgb2_b_out,
    rgb2_g_out,
    rgb2_r_out,
    sev_seg_annode_out,
    sev_seg_digits_out,
    sev_seg_points_out,
    sysclk,
    sysreset_n,
    tsl235r_pulse_in,
    uart_out_rxd,
    uart_out_txd);
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
  input pwm_det_ref_clk_in;
  input pwm_det_reset_in;
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
  input tsl235r_pulse_in;
  input uart_out_rxd;
  output uart_out_txd;

  wire clk_100MHz_out;
  wire [3:0]debug_out_tri_o;
  wire [15:0]led_out;
  wire nexys4_btn_c_in;
  wire nexys4_btn_d_in;
  wire nexys4_btn_l_in;
  wire nexys4_btn_r_in;
  wire nexys4_btn_u_in;
  wire [15:0]nexys4_swt_in;
  wire [7:0]pmod_clp_data_out;
  wire pmod_clp_e_out;
  wire pmod_clp_rs_out;
  wire pmod_clp_rw_out;
  wire pmod_enc_a_in;
  wire pmod_enc_b_in;
  wire pmod_enc_press_in;
  wire pmod_enc_swt_in;
  wire pwm_det_ref_clk_in;
  wire pwm_det_reset_in;
  wire pwm_out;
  wire rgb1_b_out;
  wire rgb1_g_out;
  wire rgb1_r_out;
  wire rgb2_b_out;
  wire rgb2_g_out;
  wire rgb2_r_out;
  wire [7:0]sev_seg_annode_out;
  wire [6:0]sev_seg_digits_out;
  wire sev_seg_points_out;
  wire sysclk;
  wire sysreset_n;
  wire tsl235r_pulse_in;
  wire uart_out_rxd;
  wire uart_out_txd;

  system system_i
       (.clk_100MHz_out(clk_100MHz_out),
        .debug_out_tri_o(debug_out_tri_o),
        .led_out(led_out),
        .nexys4_btn_c_in(nexys4_btn_c_in),
        .nexys4_btn_d_in(nexys4_btn_d_in),
        .nexys4_btn_l_in(nexys4_btn_l_in),
        .nexys4_btn_r_in(nexys4_btn_r_in),
        .nexys4_btn_u_in(nexys4_btn_u_in),
        .nexys4_swt_in(nexys4_swt_in),
        .pmod_clp_data_out(pmod_clp_data_out),
        .pmod_clp_e_out(pmod_clp_e_out),
        .pmod_clp_rs_out(pmod_clp_rs_out),
        .pmod_clp_rw_out(pmod_clp_rw_out),
        .pmod_enc_a_in(pmod_enc_a_in),
        .pmod_enc_b_in(pmod_enc_b_in),
        .pmod_enc_press_in(pmod_enc_press_in),
        .pmod_enc_swt_in(pmod_enc_swt_in),
        .pwm_det_ref_clk_in(pwm_det_ref_clk_in),
        .pwm_det_reset_in(pwm_det_reset_in),
        .pwm_out(pwm_out),
        .rgb1_b_out(rgb1_b_out),
        .rgb1_g_out(rgb1_g_out),
        .rgb1_r_out(rgb1_r_out),
        .rgb2_b_out(rgb2_b_out),
        .rgb2_g_out(rgb2_g_out),
        .rgb2_r_out(rgb2_r_out),
        .sev_seg_annode_out(sev_seg_annode_out),
        .sev_seg_digits_out(sev_seg_digits_out),
        .sev_seg_points_out(sev_seg_points_out),
        .sysclk(sysclk),
        .sysreset_n(sysreset_n),
        .tsl235r_pulse_in(tsl235r_pulse_in),
        .uart_out_rxd(uart_out_rxd),
        .uart_out_txd(uart_out_txd));
endmodule
