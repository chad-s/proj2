-- Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2015.2 (lin64) Build 1266856 Fri Jun 26 16:35:25 MDT 2015
-- Date        : Sun Feb  7 10:20:23 2016
-- Host        : Sandcrawler running 64-bit Ubuntu 14.04.3 LTS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/chad/vmshare/project/ece544/proj2/proj2.srcs/sources_1/bd/system/system_stub.vhdl
-- Design      : system
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity system is
  Port ( 
    clk_100MHz_out : out STD_LOGIC;
    debug_out_tri_o : out STD_LOGIC_VECTOR ( 3 downto 0 );
    led_out : out STD_LOGIC_VECTOR ( 15 downto 0 );
    nexys4_btn_c_in : in STD_LOGIC;
    nexys4_btn_d_in : in STD_LOGIC;
    nexys4_btn_l_in : in STD_LOGIC;
    nexys4_btn_r_in : in STD_LOGIC;
    nexys4_btn_u_in : in STD_LOGIC;
    nexys4_swt_in : in STD_LOGIC_VECTOR ( 15 downto 0 );
    pmod_clp_data_out : out STD_LOGIC_VECTOR ( 7 downto 0 );
    pmod_clp_e_out : out STD_LOGIC;
    pmod_clp_rs_out : out STD_LOGIC;
    pmod_clp_rw_out : out STD_LOGIC;
    pmod_enc_a_in : in STD_LOGIC;
    pmod_enc_b_in : in STD_LOGIC;
    pmod_enc_press_in : in STD_LOGIC;
    pmod_enc_swt_in : in STD_LOGIC;
    pwm_out : out STD_LOGIC;
    rgb1_b_out : out STD_LOGIC;
    rgb1_g_out : out STD_LOGIC;
    rgb1_r_out : out STD_LOGIC;
    rgb2_b_out : out STD_LOGIC;
    rgb2_g_out : out STD_LOGIC;
    rgb2_r_out : out STD_LOGIC;
    sev_seg_annode_out : out STD_LOGIC_VECTOR ( 7 downto 0 );
    sev_seg_digits_out : out STD_LOGIC_VECTOR ( 6 downto 0 );
    sev_seg_points_out : out STD_LOGIC;
    sysclk : in STD_LOGIC;
    sysreset_n : in STD_LOGIC;
    uart_out_rxd : in STD_LOGIC;
    uart_out_txd : out STD_LOGIC
  );

end system;

architecture stub of system is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk_100MHz_out,debug_out_tri_o[3:0],led_out[15:0],nexys4_btn_c_in,nexys4_btn_d_in,nexys4_btn_l_in,nexys4_btn_r_in,nexys4_btn_u_in,nexys4_swt_in[15:0],pmod_clp_data_out[7:0],pmod_clp_e_out,pmod_clp_rs_out,pmod_clp_rw_out,pmod_enc_a_in,pmod_enc_b_in,pmod_enc_press_in,pmod_enc_swt_in,pwm_out,rgb1_b_out,rgb1_g_out,rgb1_r_out,rgb2_b_out,rgb2_g_out,rgb2_r_out,sev_seg_annode_out[7:0],sev_seg_digits_out[6:0],sev_seg_points_out,sysclk,sysreset_n,uart_out_rxd,uart_out_txd";
begin
end;
