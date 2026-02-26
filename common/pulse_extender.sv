// ============================================================================
// Module: pulse_extender
// ----------------------------------------------------------------------------
// Key Features:
// - Rising edge detection of input pulse
// - Pulse stretching for configurable number of cycles
//
// Parameters:
// - EXTEND_CYCLES: Number of cycles to hold the output pulse active
// ============================================================================
`timescale 1 ns / 1 ps

module pulse_extender #(
  parameter int EXTEND_CYCLES = 3
) (
  input  logic clk,
  input  logic rst_n,
  input  logic pulse_in,
  output logic pulse_out
);

  logic [$clog2(EXTEND_CYCLES+1)-1:0] counter;
  logic                               pulse_active;
  
  // Detect rising edge of input pulse
  logic pulse_in_d1;
  logic pulse_start;
  
  always_ff @(posedge clk or negedge rst_n)
    if (~rst_n)
      pulse_in_d1 <= 1'b0;
    else
      pulse_in_d1 <= pulse_in;
  
  assign pulse_start = pulse_in & ~pulse_in_d1;
  
  // Counter logic for pulse extension
  always_ff @(posedge clk or negedge rst_n)
    if (~rst_n) begin
      counter <= {$clog2(EXTEND_CYCLES+1){1'b0}};
      pulse_active <= 1'b0;
    end else
      if (pulse_start) begin
        counter <= EXTEND_CYCLES - 1;
        pulse_active <= 1'b1;
      end else if (pulse_active && counter > 0)
        counter <= counter - 1;
      else if (pulse_active && counter == 0)
        pulse_active <= 1'b0;
  
  assign pulse_out = pulse_active;

endmodule
