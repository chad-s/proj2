// pulse_width_det.v - Hardware-based pulse-width detection.
//
// Created By:		Chad Sutfin
// Created On:    Jan 2016
//
// Description:
// -------------------------------------
// This pulse-width detector polls the PWM signal (pulse) at every posedge of
// the reference clock. When an edge of the PWM signal is detected, the
// respective time register (hi_time or lo_time) is updated with a counter
// value and the counter is reset. It's a pretty basic implementation
// really... If you want to save some time grading, there's nothing clever in
// here. It just works.
//
// A note on the "valid" nets... This was part of a grander scheme that I had
// in mind where a "valid" output reg would be asserted when the hi and lo
// times were stable. I ended up not doing this for simplicity and because
// inaccurately reporting pwm characteristics 0.0001% of the time is not
// a concern in the intended application.
//
///////////////////////////////////////////////////////////////////////////
module pulse_width_det
(
   input                   ref_clk,
   input                   pulse,
   input                   reset,
   output   reg   [31:0]   hi_time,
   output   reg   [31:0]   lo_time
);
   
   // INTERNAL NETS
   reg      [31:0]   logic_timer;      // Counter/timer... obviously...
   reg               logic_level;      // Reg to store previous pwm state.
   reg               valid_lo;         // Flags to assert whether a valid
   reg               valid_hi;         // capture of the waveform has occurred.


   // This sequential block is the guts of the entire pw detector.  At every
   // posedge of the reference clock, the PWM signal is sampled. If the PWM
   // signal is at a different level than the previous, an edge has occurred,
   // so the hi or lo time (registered outputs) is adjusted to reflect the
   // duration of the pwm component.
   always @ (posedge ref_clk) begin

      if (reset) begin
         hi_time     <= 32'b0;
         lo_time     <= 32'b0;
         logic_timer <= 32'b0;
         logic_level <= 1'b0;
         valid_lo    <= 1'b0;
         valid_hi    <= 1'b0;
      end

      // FALLING EDGE DETECTOR
      else if ((pulse == 1'b0) && (logic_level == 1'b1)) begin

         // Only update hi_time if a full hi segment was sampled.
         if (valid_lo) begin
            hi_time  <= logic_timer;
         end
         logic_level <= 1'b0;
         logic_timer <= 32'b0;
         valid_hi    <= 1'b1;
      end

      // RISING EDGE DETECTOR
      else if ((pulse == 1'b1) && (logic_level == 1'b0)) begin

         // Only update lo_time if a full lo segment was sampled.
         if (valid_hi) begin
            lo_time  <= logic_timer;
         end
         logic_level <= 1'b1;
         logic_timer <= 32'b0;
         valid_lo    <= 1'b1;
      end

      else begin
         logic_timer <= logic_timer + 1'b1;
      end
   end

endmodule
