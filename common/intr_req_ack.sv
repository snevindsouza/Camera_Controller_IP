// ============================================================================
// Module: intr_req_ack
// ----------------------------------------------------------------------------
// Description:
// It ensures proper handshaking between the request and acknowledge domains by:
// - Synchronizing the request signal into the acknowledge clock domain
// - Synchronizing the acknowledge signal into the request clock domain
//
// Key Features:
// - Two-way synchronization using generic synchronizer module `sync_ff`
// - Configurable number of synchronization stages and reset value
//
// Parameters:
// - NUM_FLOPS: Number of flip-flops used for synchronization (default: 2)
// - RST_VAL:   Reset value for synchronizer flip-flops (default: 0)
// ============================================================================
`timescale 1 ns / 1 ps

module intr_req_ack #(
  parameter int NUM_FLOPS = 2,  // Number of synchronization stages
  parameter bit RST_VAL   = 0   // Reset values
) (
  input  logic req_clk,         // Request clock domain
  input  logic ack_clk,         // Acknowledge clock domain
  input  logic rst_n,           // Active-low async reset
  input  logic req_signal,      // Request signal
  output logic req_signal_sync, // Synchronized request signal output
  input  logic ack_signal,      // Acknowledge signal
  output logic ack_signal_sync  // Synchronized acknowledge signal output
);
  
  // This synchronizes the request signal in the acknowledge clock domain
  sync_ff #(
    .NUM_FLOPS (NUM_FLOPS),
    .RST_VAL   (RST_VAL  )
  ) req_sync_inst (
    .clk      (ack_clk        ),
    .rst_n    (rst_n          ),
    .async_in (req_signal     ),
    .sync_out (req_signal_sync)
  );
 
  // This synchronizes the acknowledge signal in the request clock domain
  sync_ff #(
    .NUM_FLOPS (NUM_FLOPS),
    .RST_VAL   (RST_VAL  )
  ) ack_sync_inst (
    .clk      (req_clk        ),
    .rst_n    (rst_n          ),
    .async_in (ack_signal     ),
    .sync_out (ack_signal_sync)
  );

endmodule
