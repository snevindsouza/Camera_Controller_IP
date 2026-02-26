// -----------------------------------------------------------------------------
// Parameterized Synchronizer Module (1 or 2 or 3 flip-flops)
// - For clock domain crossing of single-bit or multi-bit signals
// -----------------------------------------------------------------------------
`timescale 1 ns / 1 ps

module sync_ff #(
  parameter  NUM_FLOPS = 2,          // Number of synchronization stages
  parameter  RST_VAL   = 1'b0,       // Reset values
  parameter  WIDTH     = 1
) (
  input  logic             clk,      // Destination clock
  input  logic             rst_n,    // Active-low async reset
  input  logic [WIDTH-1:0] async_in, // Asynchronous input signal
  output logic [WIDTH-1:0] sync_out  // Synchronized output
);

  logic [WIDTH-1:0][NUM_FLOPS-1:0] sync_reg;

  genvar i;
  generate for (i = 0; i < WIDTH; i++) begin : sync_chain
    if (NUM_FLOPS == 1) begin 
      always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
          sync_reg[i][0] <= RST_VAL;
        else
          sync_reg[i][0] <= async_in[i];
      assign sync_out[i] = sync_reg[i][0];
    end
    else begin
      always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
          sync_reg[i] <= {NUM_FLOPS{RST_VAL}};
        else
          sync_reg[i] <= {sync_reg[i][NUM_FLOPS-2:0], async_in[i]};
      assign sync_out[i] = sync_reg[i][NUM_FLOPS-1];
    end
  end
  endgenerate
endmodule
