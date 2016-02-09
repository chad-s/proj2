//
///////////////////////////////////////////////////////////////////////////////////////////
// Copyright � 2010-2013, Xilinx, Inc.
// This file contains confidential and proprietary information of Xilinx, Inc. and is
// protected under U.S. and international copyright and other intellectual property laws.
///////////////////////////////////////////////////////////////////////////////////////////
//
// Disclaimer:
// This disclaimer is not a license and does not grant any rights to the materials
// distributed herewith. Except as otherwise provided in a valid license issued to
// you by Xilinx, and to the maximum extent permitted by applicable law: (1) THESE
// MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX HEREBY
// DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY,
// INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT,
// OR FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable
// (whether in contract or tort, including negligence, or under any other theory
// of liability) for any loss or damage of any kind or nature related to, arising
// under or in connection with these materials, including for any direct, or any
// indirect, special, incidental, or consequential loss or damage (including loss
// of data, profits, goodwill, or any type of loss or damage suffered as a result
// of any action brought by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the possibility of the same.
//
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-safe, or for use in any
// application requiring fail-safe performance, such as life-support or safety
// devices or systems, Class III medical devices, nuclear facilities, applications
// related to the deployment of airbags, or any other applications that could lead
// to death, personal injury, or severe property or environmental damage
// (individually and collectively, "Critical Applications"). Customer assumes the
// sole risk and liability of any use of Xilinx products in Critical Applications,
// subject only to applicable laws and regulations governing limitations on product
// liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT ALL TIMES.
//
///////////////////////////////////////////////////////////////////////////////////////////
//
//
// Production definition of a 2K program for KCPSM6 in a 7-Series device using a 
// RAMB36E1 primitive.
//
// Note: The complete 12-bit address bus is connected to KCPSM6 to facilitate future code 
//       expansion with minimum changes being required to the hardware description. 
//       Only the lower 11-bits of the address are actually used for the 2K address range
//       000 to 7FF hex.  
//
// Program defined by 'C:\PSU_Projects\ECE544_Winter15_Projects\IP\ip_repo\PMod544IO_1.0\src\ECE544ifpgm.psm'.
//
// Generated by KCPSM6 Assembler: 28 Dec 2014 - 09:36:42. 
//
// Assembler used ROM_form template: ROM_form_7S_2K_14March13.v
//
//
module ECE544ifpgm (
input  [11:0] address,
output [17:0] instruction,
input         enable,
input         clk);
//
//
wire [15:0] address_a;
wire [35:0] data_in_a;
wire [35:0] data_out_a;
wire [15:0] address_b;
wire [35:0] data_in_b;
wire [35:0] data_out_b;
wire        enable_b;
wire        clk_b;
wire [7:0]  we_b;
//
//
assign address_a = {1'b1, address[10:0], 4'b1111};
assign instruction = {data_out_a[33:32],  data_out_a[15:0]};
assign data_in_a = {35'b000000000000000000000000000000000000, address[11]};
//
assign address_b = 16'b1111111111111111;
assign data_in_b = {2'h0,  data_out_b[33:32], 16'h0000, data_out_b[15:0]};
assign enable_b = 1'b0;
assign we_b = 8'h00;
assign clk_b = 1'b0;
//
RAMB36E1 # ( .READ_WIDTH_A              (18),
             .WRITE_WIDTH_A             (18),
             .DOA_REG                   (0),
             .INIT_A                    (36'h000000000),
             .RSTREG_PRIORITY_A         ("REGCE"),
             .SRVAL_A                   (36'h000000000),
             .WRITE_MODE_A              ("WRITE_FIRST"),
             .READ_WIDTH_B              (18),
             .WRITE_WIDTH_B             (18),
             .DOB_REG                   (0),
             .INIT_B                    (36'h000000000),
             .RSTREG_PRIORITY_B         ("REGCE"),
             .SRVAL_B                   (36'h000000000),
             .WRITE_MODE_B              ("WRITE_FIRST"),
             .INIT_FILE                 ("NONE"),
             .SIM_COLLISION_CHECK       ("ALL"),
             .RAM_MODE                  ("TDP"),
             .RDADDR_COLLISION_HWCONFIG ("DELAYED_WRITE"),
             .EN_ECC_READ               ("FALSE"),
             .EN_ECC_WRITE              ("FALSE"),
             .RAM_EXTENSION_A           ("NONE"),
             .RAM_EXTENSION_B           ("NONE"),
             .SIM_DEVICE                ("7SERIES"),
             .INIT_00                   (256'h10001D02DF03DE021E001F00D00050401080D00410000119D0041080D0001040),
             .INIT_01                   (256'h00B5DC802020D680016096048001D0001000006CD00110001B001C00D0005040),
             .INIT_02                   (256'h114E1020201A01390B6000A3DB40202BD6400098DB802027D680016096000C60),
             .INIT_03                   (256'hE10011201001E10011731001E10011791001E10011781001E10011651001E100),
             .INIT_04                   (256'h1001E100116C1001E10011651001E10011531001E10011001001E10011331001),
             .INIT_05                   (256'h11001001E10011741001E10011731001E10011651001E10011541001E1001166),
             .INIT_06                   (256'h150400E4002E600050002065160100E2206BD500A56000D8051006005000E100),
             .INIT_07                   (256'h015B00FC015600FC015600FC015600FC015600621028112000621020111100F5),
             .INIT_08                   (256'h155900E2154200E2152000E2154500E2155900E2154200ED150100F51507015B),
             .INIT_09                   (256'h8001DF03DE021E001F008000D2001280500000E400F51504015B00E2154500E2),
             .INIT_0A                   (256'h8000D20012804320430E430E430E430E331003104206320F02105000D2001200),
             .INIT_0B                   (256'hD10320E2D10220D8D10120D5D1009505311FD30413805000D200120080010D30),
             .INIT_0C                   (256'hD10B20F9D10A20F5D10920F1D10820EDD10720E9D10620E6D10520E4D10420D5),
             .INIT_0D                   (256'h55C0350F20D5010B5580350F20DED5105000D30413002102D10D20FFD10C20FC),
             .INIT_0E                   (256'h010B5580357F20D5010B5540353F20D5010B150220D5012920D5011220D5010B),
             .INIT_0F                   (256'h150020D5012E150320D5012E150220D5010B5508350720D5010B5504350320D5),
             .INIT_10                   (256'h01050134D506D40514005000D40574010134D405740120D5012E150120D5012E),
             .INIT_11                   (256'h010B150C013D010B1538014C014C5000013D01050134D506D40514045000013D),
             .INIT_12                   (256'h45063503500001420142010B1501500001420142010B1501013D010B1506013D),
             .INIT_13                   (256'h9101013911285000613A90011018500061369001400E10185000010B55104506),
             .INIT_14                   (256'h614D9301014213145000614893010142130A500061439201013D12195000613E),
             .INIT_15                   (256'h5000615C9401014C1432500061579401014C1419500061529401014C140D5000),
             .INIT_16                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_17                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_18                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_19                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_1A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_1B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_1C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_1D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_1E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_1F                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_20                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_21                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_22                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_23                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_24                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_25                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_26                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_27                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_28                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_29                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_2A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_2B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_2C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_2D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_2E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_2F                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_30                   (256'h231C3F000E20A310430E9301420E321E02D0310101D0F414F313F212F111F010),
             .INIT_31                   (256'hDF03DE02D10011801F001E00A31CCF40A31CCE30E31C410EBF008E2004F003E0),
             .INIT_32                   (256'h000000000000000000000000000000009001B010B111B212B313B414D1001100),
             .INIT_33                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_34                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_35                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_36                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_37                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_38                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_39                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_3A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_3B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_3C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_3D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_3E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_3F                   (256'h2300000000000000000000000000000000000000000000000000000000000000),
             .INIT_40                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_41                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_42                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_43                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_44                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_45                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_46                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_47                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_48                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_49                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_4A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_4B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_4C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_4D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_4E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_4F                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_50                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_51                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_52                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_53                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_54                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_55                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_56                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_57                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_58                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_59                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_5A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_5B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_5C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_5D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_5E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_5F                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_60                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_61                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_62                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_63                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_64                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_65                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_66                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_67                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_68                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_69                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_6A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_6B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_6C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_6D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_6E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_6F                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_70                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_71                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_72                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_73                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_74                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_75                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_76                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_77                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_78                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_79                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_7A                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_7B                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_7C                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_7D                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_7E                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INIT_7F                   (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_00                  (256'hAAAAA08228A6D20A1861861861861861861861860A333300CC0A34080A082288),
             .INITP_01                  (256'h28A2828282828AAA0A0CA3777777777777742288A0550428A828A8A222222222),
             .INITP_02                  (256'h0000000000000000B62D8B62D8B62D8B62D2D4A14AA2A8A28A2AAA8AAA28A28A),
             .INITP_03                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_04                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_05                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_06                  (256'h000000000000000000000000000000000000000000008008A80DDD50974402AA),
             .INITP_07                  (256'h8000000000000000000000000000000000000000000000000000000000000000),
             .INITP_08                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_09                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_0A                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_0B                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_0C                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_0D                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_0E                  (256'h0000000000000000000000000000000000000000000000000000000000000000),
             .INITP_0F                  (256'h0000000000000000000000000000000000000000000000000000000000000000))
 kcpsm6_rom( .ADDRARDADDR               (address_a),
             .ENARDEN                   (enable),
             .CLKARDCLK                 (clk),
             .DOADO                     (data_out_a[31:0]),
             .DOPADOP                   (data_out_a[35:32]), 
             .DIADI                     (data_in_a[31:0]),
             .DIPADIP                   (data_in_a[35:32]), 
             .WEA                       (4'h0),
             .REGCEAREGCE               (1'b0),
             .RSTRAMARSTRAM             (1'b0),
             .RSTREGARSTREG             (1'b0),
             .ADDRBWRADDR               (address_b),
             .ENBWREN                   (enable_b),
             .CLKBWRCLK                 (clk_b),
             .DOBDO                     (data_out_b[31:0]),
             .DOPBDOP                   (data_out_b[35:32]), 
             .DIBDI                     (data_in_b[31:0]),
             .DIPBDIP                   (data_in_b[35:32]), 
             .WEBWE                     (we_b),
             .REGCEB                    (1'b0),
             .RSTRAMB                   (1'b0),
             .RSTREGB                   (1'b0),
             .CASCADEINA                (1'b0),
             .CASCADEINB                (1'b0),
             .CASCADEOUTA               (),
             .CASCADEOUTB               (),
             .DBITERR                   (),
             .ECCPARITY                 (),
             .RDADDRECC                 (),
             .SBITERR                   (),
             .INJECTDBITERR             (1'b0),
             .INJECTSBITERR             (1'b0));
//
//
endmodule
//
////////////////////////////////////////////////////////////////////////////////////
//
// END OF FILE ECE544ifpgm.v
//
////////////////////////////////////////////////////////////////////////////////////