module tsl235r (
   input                clk,
   input                reset,
   input                tsl235r_in,
   output reg  [31:0]   frequency
);

   reg   [31:0]   hi_time;
   reg   [31:0]   lo_time;

   pulse_width_det PWDET (
      .ref_clk(clk),
      .pulse(tsl235r_in),
      .reset(reset),
      .hi_time(hi_time),
      .lo_time(lo_time)
   );


endmodule
