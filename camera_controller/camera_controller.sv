//
//=================================================================================================
// IP Name: camera_controller
//
// Description: 
//   This IP extracted from **Mirafra's Ramanujan SOC** is a camera controller that interfaces
//   with the DVP (Digital Video Port) of an external camera sensor (ex. OmniVision), stages
//   incoming image data through internal SRAM ping-pong buffers and triggers an on-chip DMA
//   to transfer buffered data from either ping or pong camera buffers to larger full-frame
//   buffers sitting elsewhere on-chip.
//
// Key Features:
//   - DVP camera sensor interface (VSYNC, HREF, PCLK, 8-bit data)
//   - Camera Data Pipe management
//   - Camera ping-pong Buffer management
//   - DMA trigger management for data transfer
//   - Interrupts (Buffer over/under, Line/Frame xfer comp, line input done)
//   - AXI slave interface for DMA access to internal ping-pong buffers (not included here)
//   - APB slave interface for register programming (not included here)
//=================================================================================================
//
`timescale 1ns / 1ps

module camera_controller (

  input  logic        rst_n,     // Active-low reset
  output logic        cam_clk,   // Camera clock output (derived from DVP PCLK ~100MHz)

  // Camera Sensor DVP Interface
  input  logic        dvp_vsync, // Vertical sync - indicates start/end of frame
  input  logic        dvp_href,  // Horizontal reference - indicates valid line data
  input  logic        dvp_pclk,  // Pixel clock from camera sensor
  input  logic [7:0]  dvp_data,  // 8-bit pixel data bus

  // AXI Slave Read Request Channel
  input  logic [13:0] i_axi_s_cam_cntrl_araddr,
  input  logic [1:0]  i_axi_s_cam_cntrl_arburst,
  input  logic [11:0] i_axi_s_cam_cntrl_arid,
  input  logic [7:0]  i_axi_s_cam_cntrl_arlen,
  output logic        o_axi_s_cam_cntrl_arready,
  input  logic [2:0]  i_axi_s_cam_cntrl_arsize,
  input  logic        i_axi_s_cam_cntrl_arvalid,
  input  logic        i_axi_s_cam_cntrl_arlock,
  input  logic [2:0]  i_axi_s_cam_cntrl_arprot,
  input  logic [3:0]  i_axi_s_cam_cntrl_arqos,

  // AXI Slave Write Request Channel
  input  logic [13:0] i_axi_s_cam_cntrl_awaddr,
  input  logic        i_axi_s_cam_cntrl_awakeup,
  input  logic [1:0]  i_axi_s_cam_cntrl_awburst,
  input  logic [11:0] i_axi_s_cam_cntrl_awid,
  input  logic [7:0]  i_axi_s_cam_cntrl_awlen,
  output logic        o_axi_s_cam_cntrl_awready,
  input  logic [2:0]  i_axi_s_cam_cntrl_awsize,
  input  logic        i_axi_s_cam_cntrl_awvalid,
  input  logic        i_axi_s_cam_cntrl_awlock,
  input  logic [2:0]  i_axi_s_cam_cntrl_awprot,
  input  logic [3:0]  i_axi_s_cam_cntrl_awqos,

  // AXI Slave Write Data Channel
  input  logic [127:0] i_axi_s_cam_cntrl_wdata,
  input  logic         i_axi_s_cam_cntrl_wlast,
  output logic         o_axi_s_cam_cntrl_wready,
  input  logic [15:0]  i_axi_s_cam_cntrl_wstrb,
  input  logic         i_axi_s_cam_cntrl_wvalid,
  input  logic [1:0]   i_axi_s_cam_cntrl_wpoison,

  // AXI Slave Write Response Channel
  output logic [11:0] o_axi_s_cam_cntrl_bid,
  input  logic        i_axi_s_cam_cntrl_bready,
  output logic [1:0]  o_axi_s_cam_cntrl_bresp,
  output logic        o_axi_s_cam_cntrl_bvalid,

  // AXI Slave Read Response Channel
  output logic [127:0] o_axi_s_cam_cntrl_rdata,
  output logic [11:0]  o_axi_s_cam_cntrl_rid,
  output logic         o_axi_s_cam_cntrl_rlast,
  input  logic         i_axi_s_cam_cntrl_rready,
  output logic [1:0]   o_axi_s_cam_cntrl_rresp,
  output logic         o_axi_s_cam_cntrl_rvalid,
  output logic [1:0]   o_axi_s_cam_cntrl_rpoison,

  // APB Slave Interface
  input  logic        pclk,
  input  logic        presetn,
  input  logic [2:0]  pprot,
  input  logic        psel,
  input  logic        penable,
  input  logic        pwrite,
  input  logic [3:0]  pstrb,
  input  logic [31:0] paddr,
  input  logic [31:0] pwdata,
  output logic [31:0] prdata,
  output logic        pslverr,
  output logic        pready,

  // DMA Trigger Interface
  output logic       dma_trig_req,            // DMA trigger request
  output logic [1:0] dma_trig_req_type,       // Request type (line/frame transfer)
  input  logic       dma_trig_ack,            // DMA acknowledgment
  input  logic [1:0] dma_trig_ack_type,       // Acknowledgment type

  // Interrupts
  output logic       intr_line_tx_dn,         // Line transfer complete interrupt
  output logic       intr_line_in_dn,         // Line input complete interrupt
  output logic       intr_frm_tx_dn,          // Frame transfer complete interrupt
  output logic       intr_buf_over_err,       // Buffer overflow error interrupt
  output logic       intr_buf_undr_err,       // Buffer underrun error interrupt

  // AXI-5 sideband: Power Management and Clock Gating
  input  logic       i_axi_s_clk_qreqn,       // Clock request
  output logic       o_axi_s_clk_qacceptn,    // Clock accept
  output logic       o_axi_s_clk_qdeny,       // Clock deny
  output logic       o_axi_s_clk_qactive,     // Clock active
  input  logic       i_axi_s_pwr_qreqn,       // Power request
  output logic       o_axi_s_pwr_qacceptn,    // Power accept
  output logic       o_axi_s_pwr_qdeny,       // Power deny
  output logic       o_axi_s_pwr_qactive,     // Power active
  input  logic       i_axi_s_ext_gt_qreqn,    // External gate request
  output logic       o_axi_s_ext_gt_qacceptn, // External gate accept
  input  logic       i_axi_s_cfg_gate_resp,   // Configuration gate response
  input  logic       SCR1_CAM_SRAM_EMAS,      // Memory EMAS Control from SOC SCR
  input  logic [1:0] SCR1_CAM_SRAM_EMAW,      // Memory EMAW Control from SOC SCR
  input  logic [2:0] SCR1_CAM_SRAM_EMA,       // Memory EMA Control from SOC SCR

  // Debug Signals - Legacy debug mux-out for SOC
  output logic       dbg_cam_buffer_a_select, // Routing buffer_a_select to Debug Mux
  output logic       dbg_cam_buffer_a_full,		// Routing buffer_a_full to Debug Mux
  output logic       dbg_cam_buffer_b_full,		// Routing buffer_b_full to Debug Mux
  output logic       dbg_cam_buffer_a_empty, 	// Routing buffer_a_empty to Debug Mux
  output logic       dbg_cam_buffer_b_empty   // Routing buffer_b_empty to Debug Mux

);

  // Configuration signals from register interface to camera data pipe
  logic [1:0]   frmt_cam;               // Camera format selection
  logic [1:0]   res_cam;                // Camera resolution setting
  logic         buf_undr_err_mask;      // Buffer underrun error mask
  logic         buf_over_err_mask;      // Buffer overflow error mask
  logic         frm_tx_dn_mask;         // Frame transfer done mask
  logic         line_in_dn_mask;        // Line input done mask
  logic         line_tx_dn_mask;        // Line transfer done mask
  logic         dma_trig_mask;          // DMA trigger mask
  logic         en_cam;                 // Camera enable control
  logic         debug_sram_sel;
  logic         debug_en;

  logic         intr_line_tx_dn_cdp;    // Line Transfer Done interrupt
  logic         intr_line_in_dn_cdp;    // Line Input Done interrupt
  logic         intr_frm_tx_dn_cdp;     // Frame Transfer Done interrupt
  logic         intr_buf_over_err_cdp;  // Buffer Overflow Error interrupt
  logic         intr_buf_undr_err_cdp;  // Buffer Underflow Error interrupt

  // Interrupt Synchronization
  logic         intr_line_tx_dn_sync;   // Synchronized Line Transfer Done interrupt
  logic         intr_line_in_dn_sync;   // Synchronized Line Input Done interrupt
  logic         intr_frm_tx_dn_sync;    // Synchronized Frame Transfer Done interrupt
  logic         intr_buf_over_err_sync; // Synchronized Buffer Overflow Error interrupt
  logic         intr_buf_undr_err_sync; // Synchronized Buffer Underflow Error interrupt

  logic         intr_line_tx_dn_ack;    // Acknowledge for Line Transfer Done interrupt
  logic         intr_line_in_dn_ack;    // Acknowledge for Line Input Done interrupt
  logic         intr_frm_tx_dn_ack;     // Acknowledge for Frame Transfer Done interrupt
  logic         intr_buf_over_err_ack;  // Acknowledge for Buffer Overflow Error interrupt
  logic         intr_buf_undr_err_ack;  // Acknowledge for Buffer Underflow Error interrupt

  // Interrupts
  logic         dma_trig_ack_sync;      // Synchronized DMA acknowledgment
  logic [1:0]   dma_trig_ack_type_sync; // Synchronized DMA acknowledgment type
  logic         en_cam_sync;            // Synchronized camera enable
  logic         debug_en_sync;          // Synchronized debug enable

  // Memory interface - internal SRAM
  logic [8:0]   memaddr;                // Memory address
  logic [127:0] memd;                   // Memory write data
  logic [127:0] memq;                   // Memory read data
  logic         memcen;                 // Memory chip enable
  logic [15:0]  memwen;                 // Memory write enable

  // Pad unused address Address Bits (for width matching)
  logic [13:0]  padded_memaddr;         // Padded Memory address - with unused bits: [13],[3:0]


  assign memaddr = padded_memaddr[12:4];

  // Camera clock - use pixel clock from sensor
  assign cam_clk = dvp_pclk;

  // Synchronize DMA acknowledgment from dma clock to camera pixel clock
  mux_sync #(
    .DATA_WIDTH    (2),                       // 2-bit data for ack type
    .EXTEND_CYCLES (0),                       // No cycle extension
    .SYNC_FLOPS    (2),                       // 2-stage synchronizer
    .EN_PULSE_EXT  (1'b0)                     // No pulse extension
  ) u_sync_cam_ack (
    .src_clk     (1'b0),                      // Source clock (not used for level sync)
    .dst_clk     (dvp_pclk),                  // Destination clock (camera pixel clock)
    .rst_n       (rst_n),                     // Reset
    .en_in       (dma_trig_ack),              // Enable input
    .en_sync_out (dma_trig_ack_sync),         // Synchronized enable output
    .dataIn      (dma_trig_ack_type),         // Data input
    .dataOut     (dma_trig_ack_type_sync)     // Synchronized data output
  );

  // Synchronize camera enable from APB clock domain to camera pixel clock domain
  sync_ff #(
    .NUM_FLOPS (2),                           // 2-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value
  ) u_sync_ff (
    .clk      (dvp_pclk),                     // Destination clock
    .rst_n    (rst_n),                        // Reset
    .async_in (en_cam),                       // Asynchronous input from register
    .sync_out (en_cam_sync)                   // Synchronized output
  );

  // Synchronize debug enable from APB clock domain to camera pixel clock domain
  sync_ff #(
    .NUM_FLOPS (2),                           // 2-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value
  ) u_sync_en (
    .clk      (dvp_pclk),                     // Destination clock domain
    .rst_n    (rst_n),                        // Active-low reset
    .async_in (debug_en),                     // Asynchronous input
    .sync_out (debug_en_sync)                 // Synchronized output
  );

  //Synchronizes request and acknowledge signals across two clocks
  intr_req_ack #(
    .NUM_FLOPS (2),                           // Two-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value = 0
  ) u_intr_line_tx_dn (
    .req_clk        (dvp_pclk),               // Request side clock
    .ack_clk        (pclk),                   // Acknowledge side clock
    .rst_n          (rst_n),                  // Active-low reset
    .req_signal     (intr_line_tx_dn_cdp),    // Request signal from clk_a domain
    .req_signal_sync(intr_line_tx_dn_sync),   // Request signal synchronized into clk_b domain
    .ack_signal     (intr_line_tx_dn),        // Acknowledge signal from clk_b domain
    .ack_signal_sync(intr_line_tx_dn_ack)     // Acknowledge signal synchronized into clk_a domain
  );

  intr_req_ack #(
    .NUM_FLOPS (2),                           // Two-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value = 0
  ) u_intr_line_in_dn (
    .req_clk        (dvp_pclk),               // Request side clock
    .ack_clk        (pclk),                   // Acknowledge side clock
    .rst_n          (rst_n),                  // Active-low reset
    .req_signal     (intr_line_in_dn_cdp),    // Request signal from clk_a domain
    .req_signal_sync(intr_line_in_dn_sync),   // Request signal synchronized into clk_b domain
    .ack_signal     (intr_line_in_dn),        // Acknowledge signal from clk_b domain
    .ack_signal_sync(intr_line_in_dn_ack)     // Acknowledge signal synchronized into clk_a domain
  );

  intr_req_ack #(
    .NUM_FLOPS (2),                           // Two-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value = 0
  ) u_intr_frm_tx_dn (
    .req_clk        (dvp_pclk),               // Request side clock
    .ack_clk        (pclk),                   // Acknowledge side clock
    .rst_n          (rst_n),                  // Active-low reset
    .req_signal     (intr_frm_tx_dn_cdp),     // Request signal from clk_a domain
    .req_signal_sync(intr_frm_tx_dn_sync),    // Request signal synchronized into clk_b domain
    .ack_signal     (intr_frm_tx_dn),         // Acknowledge signal from clk_b domain
    .ack_signal_sync(intr_frm_tx_dn_ack)      // Acknowledge signal synchronized into clk_a domain
  );

  
  intr_req_ack #(
    .NUM_FLOPS (2),                           // Two-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value = 0
  ) u_intr_buf_over_err (
    .req_clk        (dvp_pclk),               // Request side clock
    .ack_clk        (pclk),                   // Acknowledge side clock
    .rst_n          (rst_n),                  // Active-low reset
    .req_signal     (intr_buf_over_err_cdp),  // Request signal from clk_a domain
    .req_signal_sync(intr_buf_over_err_sync), // Request signal synchronized into clk_b domain
    .ack_signal     (intr_buf_over_err),      // Acknowledge signal from clk_b domain
    .ack_signal_sync(intr_buf_over_err_ack)   // Acknowledge signal synchronized into clk_a domain
  );

  intr_req_ack #(
    .NUM_FLOPS (2),                           // Two-stage synchronizer
    .RST_VAL   (1'b0)                         // Reset value = 0
  ) u_intr_buf_undr_err (
    .req_clk        (dvp_pclk),               // Request side clock
    .ack_clk        (pclk),                   // Acknowledge side clock
    .rst_n          (rst_n),                  // Active-low reset
    .req_signal     (intr_buf_undr_err_cdp),  // Request signal from clk_a domain
    .req_signal_sync(intr_buf_undr_err_sync), // Request signal synchronized into clk_b domain
    .ack_signal     (intr_buf_undr_err),      // Acknowledge signal from clk_b domain
    .ack_signal_sync(intr_buf_undr_err_ack)   // Acknowledge signal synchronized into clk_a domain
  );

  //=============================================================================================
  // Camera Data Pipeline
  //=============================================================================================
  camera_data_pipe u_camera_data_pipe (
    .rst_n                   (rst_n),                   // input
    .dvp_vsync               (dvp_vsync),               // input
    .dvp_href                (dvp_href),                // input
    .dvp_pclk                (dvp_pclk),                // input
    .dvp_data                (dvp_data),                // input [7:0]
    .intr_line_tx_dn         (intr_line_tx_dn_cdp),     // output
    .intr_line_in_dn         (intr_line_in_dn_cdp),     // output
    .intr_frm_tx_dn          (intr_frm_tx_dn_cdp),      // output
    .intr_buf_over_err       (intr_buf_over_err_cdp),   // output
    .intr_buf_undr_err       (intr_buf_undr_err_cdp),   // output
    .intr_line_tx_dn_ack     (intr_line_tx_dn_ack),     // input
    .intr_frm_tx_dn_ack      (intr_frm_tx_dn_ack),      // input
    .intr_line_in_dn_ack     (intr_line_in_dn_ack),     // input
    .intr_buf_over_err_ack   (intr_buf_over_err_ack),   // input
    .intr_buf_undr_err_ack   (intr_buf_undr_err_ack),   // input
    .dma_trig_req            (dma_trig_req),            // output
    .dma_trig_req_type       (dma_trig_req_type),       // output [1:0]
    .dma_trig_ack            (dma_trig_ack_sync),       // input
    .dma_trig_ack_type       (dma_trig_ack_type_sync),  // input [1:0]
    .dma_trig_mask           (dma_trig_mask),           // input
    .memaddr                 (memaddr),                 // input [8:0]
    .memd                    (memd),                    // input [127:0]
    .memq                    (memq),                    // output [127:0]
    .memcen                  (memcen),                  // input
    .memwen                  (memwen),                  // input [15:0]
    .res                     (res_cam),                 // input [1:0]
    .frmt                    (frmt_cam),                // input [1:0
    .en_cam                  (en_cam_sync),             // input
    .line_tx_dn_mask         (line_tx_dn_mask),         // input
    .line_in_dn_mask         (line_in_dn_mask),         // input
    .frm_tx_dn_mask          (frm_tx_dn_mask),          // input
    .buf_over_err_mask       (buf_over_err_mask),       // input
    .buf_undr_err_mask       (buf_undr_err_mask),       // input
    .debug_sram_sel          (debug_sram_sel),          // input
    .debug_en                (debug_en_sync),           // input
    .SCR1_CAM_SRAM_EMAS      (SCR1_CAM_SRAM_EMAS),      // input
    .SCR1_CAM_SRAM_EMAW      (SCR1_CAM_SRAM_EMAW),      // input [1:0]
    .SCR1_CAM_SRAM_EMA       (SCR1_CAM_SRAM_EMA),       // input [2:0]
    .dbg_cam_buffer_a_select (dbg_cam_buffer_a_select), // output
    .dbg_cam_buffer_a_full   (dbg_cam_buffer_a_full),   // output
    .dbg_cam_buffer_b_full   (dbg_cam_buffer_b_full),   // output
    .dbg_cam_buffer_a_empty  (dbg_cam_buffer_a_empty),  // output
    .dbg_cam_buffer_b_empty  (dbg_cam_buffer_b_empty)   // output
  );

  //=============================================================================================
  // Register Interface
  //=============================================================================================
  cam_reg_apb_if_stub u_reg_interface (
    .frmt_cam_out                          (frmt_cam),               // output [1:0]
    .res_cam_out                           (res_cam),                // output [1:0]
    .intr_buffer_under_buf_und_out         (intr_buf_undr_err),      // output
    .intr_buffer_under_buf_und_in          (intr_buf_undr_err_sync), // input
    .intr_buffer_under_buf_und_hw_wr_en_in (intr_buf_undr_err_sync), // input
    .intr_buffer_over_buf_ovr_out          (intr_buf_over_err),      // output
    .intr_buffer_over_buf_ovr_in           (intr_buf_over_err_sync), // input
    .intr_buffer_over_buf_ovr_hw_wr_en_in  (intr_buf_over_err_sync), // input
    .intr_frm_trn_tx_cmt_out               (intr_frm_tx_dn),         // output
    .intr_frm_trn_tx_cmt_in                (intr_frm_tx_dn_sync),    // input
    .intr_frm_trn_tx_cmt_hw_wr_en_in       (intr_frm_tx_dn_sync),    // input
    .intr_line_input_in_cmt_out            (intr_line_in_dn),        // output
    .intr_line_input_in_cmt_in             (intr_line_in_dn_sync),   // input
    .intr_line_input_in_cmt_hw_wr_en_in    (intr_line_in_dn_sync),   // input
    .intr_line_trn_tx_cmt_out              (intr_line_tx_dn),        // output
    .intr_line_trn_tx_cmt_in               (intr_line_tx_dn_sync),   // input
    .intr_line_trn_tx_cmt_hw_wr_en_in      (intr_line_tx_dn_sync),   // input
    .intr_mask_buf_under_out               (buf_undr_err_mask),      // output
    .intr_mask_buf_over_out                (buf_over_err_mask),      // output
    .intr_mask_frm_tx_cmt_out              (frm_tx_dn_mask),         // output
    .intr_mask_line_in_cmt_out             (line_in_dn_mask),        // output
    .intr_mask_line_tx_cmt_out             (line_tx_dn_mask),        // output
    .cam_ctrl_en_out                       (en_cam),                 // output
    .dma_ctrl_trig_mask_out                (dma_trig_mask),          // output
    .debug_ctrl_sram_buffer_out            (debug_sram_sel),         // output
    .debug_ctrl_debug_en_out               (debug_en),               // output
    .PADDR                                 (paddr[6:0]),             // input
    .PPROT                                 (pprot),                  // input [2:0]
    .PSEL                                  (psel),                   // input
    .PENABLE                               (penable),                // input
    .PWRITE                                (pwrite),                 // input [31:0]
    .PWDATA                                (pwdata),                 // input
    .PSTRB                                 (pstrb),                  // input [3:0]
    .PREADY                                (pready),                 // output
    .PRDATA                                (prdata),                 // output [31:0]
    .PSLVERR                               (pslverr),                // output
    .PCLK                                  (pclk),                   // input
    .PRESETn                               (presetn)                 // input
  );

  //=============================================================================================
  // SRAM Controller
  //=============================================================================================
  axi5_sram_ctrl_stub U_SRAM_CTRL (
    .aclk            (dvp_pclk),                  // input
    .aresetn         (rst_n),                     // input
    .awvalid_s       (i_axi_s_cam_cntrl_awvalid), // input
    .awready_s       (o_axi_s_cam_cntrl_awready), // output
    .awid_s          (i_axi_s_cam_cntrl_awid),    // input [11:0]
    .awaddr_s        (i_axi_s_cam_cntrl_awaddr),  // input [13:0]
    .awlen_s         (i_axi_s_cam_cntrl_awlen),   // input [7:0]
    .awsize_s        (i_axi_s_cam_cntrl_awsize),  // input [2:0]
    .awburst_s       (i_axi_s_cam_cntrl_awburst), // input [1:0]
    .awlock_s        (i_axi_s_cam_cntrl_awlock),  // input
    .awprot_s        (i_axi_s_cam_cntrl_awprot),  // input [2:0]
    .awqos_s         (i_axi_s_cam_cntrl_awqos),   // input [3:0]
    .awakeup_s       (i_axi_s_cam_cntrl_awakeup), // input
    .wvalid_s        (i_axi_s_cam_cntrl_wvalid),  // input
    .wready_s        (o_axi_s_cam_cntrl_wready),  // output
    .wdata_s         (i_axi_s_cam_cntrl_wdata),   // input [127:0]
    .wstrb_s         (i_axi_s_cam_cntrl_wstrb),   // input [15:0]
    .wlast_s         (i_axi_s_cam_cntrl_wlast),   // input
    .wpoison_s       (i_axi_s_cam_cntrl_wpoison), // input
    .bvalid_s        (o_axi_s_cam_cntrl_bvalid),  // output
    .bready_s        (i_axi_s_cam_cntrl_bready),  // input
    .bid_s           (o_axi_s_cam_cntrl_bid),     // output [11:0]
    .bresp_s         (o_axi_s_cam_cntrl_bresp),   // output
    .arvalid_s       (i_axi_s_cam_cntrl_arvalid), // input
    .arready_s       (o_axi_s_cam_cntrl_arready), // output
    .arid_s          (i_axi_s_cam_cntrl_arid),    // input [11:0]
    .araddr_s        (i_axi_s_cam_cntrl_araddr),  // input [13:0]
    .arlen_s         (i_axi_s_cam_cntrl_arlen),   // input [7:0]
    .arsize_s        (i_axi_s_cam_cntrl_arsize),  // input [2:0]
    .arburst_s       (i_axi_s_cam_cntrl_arburst), // input [1:0]
    .arlock_s        (i_axi_s_cam_cntrl_arlock),  // input
    .arprot_s        (i_axi_s_cam_cntrl_arprot),  // input [2:0]
    .arqos_s         (i_axi_s_cam_cntrl_arqos),   // input [3:0]
    .rvalid_s        (o_axi_s_cam_cntrl_rvalid),  // output
    .rready_s        (i_axi_s_cam_cntrl_rready),  // input
    .rid_s           (o_axi_s_cam_cntrl_rid),     // output [11:0]
    .rdata_s         (o_axi_s_cam_cntrl_rdata),   // output [127:0]
    .rresp_s         (o_axi_s_cam_cntrl_rresp),   // output [1:0]
    .rlast_s         (o_axi_s_cam_cntrl_rlast),   // output
    .rpoison_s       (o_axi_s_cam_cntrl_rpoison), // output [1:0]
    .clk_qreqn       (i_axi_s_clk_qreqn),         // input
    .clk_qacceptn    (o_axi_s_clk_qacceptn),      // output
    .clk_qdeny       (o_axi_s_clk_qdeny),         // output
    .clk_qactive     (o_axi_s_clk_qactive),       // output
    .pwr_qreqn       (i_axi_s_pwr_qreqn),         // input
    .pwr_qacceptn    (o_axi_s_pwr_qacceptn),      // output
    .pwr_qdeny       (o_axi_s_pwr_qdeny),         // output
    .pwr_qactive     (o_axi_s_pwr_qactive),       // output
    .ext_gt_qreqn    (i_axi_s_ext_gt_qreqn),      // input
    .ext_gt_qacceptn (o_axi_s_ext_gt_qacceptn),   // output
    .cfg_gate_resp   (i_axi_s_cfg_gate_resp),     // input
    .memaddr         (padded_memaddr),            // output [13:0]
    .memd            (memd),                      // output [127:0]
    .memq            (memq),                      // input [127:0]
    .memcen          (memcen),                    // output
    .memwen          (memwen)                     // output [15:0]
  );

endmodule

