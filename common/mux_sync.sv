// =============================================================================
// Module: mux_sync
// -----------------------------------------------------------------------------
// Description:
// Snchronizes a control signal en_in and its associated data
// dataIn from a source clock domain src_clk to a destination clock
// domain dst_clk. 
// 
// Key Features:
// - Pulse extension (optional)
// - Multi-flop synchronization for metastability protection SYNC_FLOPS
// - Edge detection logic for capturing the control signal transition
// - Transfers a data bus dataIn when control signal is asserted
// - Outputs synchronized enable en_sync_out and valid data dataOut
//
// Parameters:
// - DATA_WIDTH:     Width of data bus
// - EXTEND_CYCLES:  Cycles to extend pulse (if enabled, 0 is invalid)
// - SYNC_FLOPS:     Number of synchronizing flip-flops
// - EN_PULSE_EXT:   Enable pulse extension and edge-based output
// =============================================================================
`timescale 1 ns / 1 ps

module mux_sync #(
  parameter int DATA_WIDTH    = 32,
  parameter int EXTEND_CYCLES = 3,
  parameter int SYNC_FLOPS    = 2,
  parameter bit EN_PULSE_EXT  = 1'b1 // Controls pulse extender and output type
)(
  input  logic                  src_clk,
  input  logic                  dst_clk,  
  input  logic                  rst_n,
  input  logic                  en_in,
  output logic                  en_sync_out, // Pulse/Level => EN_PULSE_EXT=1/0
  input  logic [DATA_WIDTH-1:0] dataIn,
  output logic [DATA_WIDTH-1:0] dataOut
);

  logic extended_pulse;
  logic sync_ff_out;
  logic sync_ff_out_delayed;
  logic en_sync;
  logic data_capture;

  // Stage 1: Conditional Pulse Extension
  generate if (EN_PULSE_EXT)
    // Include pulse extender
    pulse_extender #(
      .EXTEND_CYCLES (EXTEND_CYCLES)
    ) u_pulse_extender (
      .clk       (src_clk       ),
      .rst_n     (rst_n         ),
      .pulse_in  (en_in         ),
      .pulse_out (extended_pulse)
    );
    else // Bypass pulse extender - pass through 
      assign extended_pulse = en_in;
  endgenerate

  // Stage 2: Synchronize signal 
  sync_ff #(
    .NUM_FLOPS (SYNC_FLOPS),
    .RST_VAL   (1'b0)
  ) u_enable_sync (
    .clk      (dst_clk       ),
    .rst_n    (rst_n         ),
    .async_in (extended_pulse),
    .sync_out (sync_ff_out   )
  );

  // Stage 3: Positive edge detector
  always_ff @(posedge dst_clk or negedge rst_n)
    if (!rst_n)
      sync_ff_out_delayed <= 1'b0;
    else
      sync_ff_out_delayed <= sync_ff_out;
  
  // Conditional output based on pulse extender parameter
  generate if (EN_PULSE_EXT)
    // When pulse extender is enabled, output pulse (edge detection)
    assign en_sync = sync_ff_out & ~sync_ff_out_delayed;
  else
    // When pulse extender is disabled, output the delayed sync signal (level)
    assign en_sync = sync_ff_out_delayed;
  endgenerate

  generate if (EN_PULSE_EXT)
    always_ff @(posedge dst_clk or negedge rst_n)
      if (~rst_n)
        en_sync_out <= 1'b0;
      else
        en_sync_out <= en_sync;
  else
    assign en_sync_out = en_sync;
  endgenerate

  assign data_capture = sync_ff_out & ~sync_ff_out_delayed;

  // Stage 4: Data capture
  always_ff @(posedge dst_clk or negedge rst_n)
    if (~rst_n)
      dataOut <= {DATA_WIDTH{1'b0}};
    else
      dataOut <= data_capture ? dataIn : dataOut;

endmodule








