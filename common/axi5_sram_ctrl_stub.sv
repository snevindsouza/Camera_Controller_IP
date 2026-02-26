// Stub of an SRAM mem control module with an AXI-5 interface
//
module axi5_sram_ctrl_stub (
  input  wire logic           aclk,
  input  wire logic           aresetn,
  input  wire logic           awvalid_s,
  output      logic           awready_s,
  input  wire logic [12-1:0]  awid_s,
  input  wire logic [14-1:0]  awaddr_s,
  input  wire logic [7:0]     awlen_s,
  input  wire logic [2:0]     awsize_s,
  input  wire logic [1:0]     awburst_s,
  input  wire logic           awlock_s,
  input  wire logic [2:0]     awprot_s,
  input  wire logic [3:0]     awqos_s,
  input  wire logic           wvalid_s,
  output      logic           wready_s,
  input  wire logic [128-1:0] wdata_s,
  input  wire logic [16-1:0]  wstrb_s,
  input  wire logic           wlast_s,
  input  wire logic [2-1:0]   wpoison_s,
  output      logic           bvalid_s,
  input  wire logic           bready_s,
  output      logic [12-1:0]  bid_s,
  output      logic [1:0]     bresp_s,
  input  wire logic           arvalid_s,
  output      logic           arready_s,
  input  wire logic [12-1:0]  arid_s,
  input  wire logic [14-1:0]  araddr_s,
  input  wire logic [7:0]     arlen_s,
  input  wire logic [2:0]     arsize_s,
  input  wire logic [1:0]     arburst_s,
  input  wire logic           arlock_s,
  input  wire logic [2:0]     arprot_s,
  input  wire logic [3:0]     arqos_s,
  output      logic           rvalid_s,
  input  wire logic           rready_s,
  output      logic [12-1:0]  rid_s,
  output      logic [128-1:0] rdata_s,
  output      logic [1:0]     rresp_s,
  output      logic           rlast_s,
  output      logic [2-1:0]   rpoison_s,
  input  wire logic           awakeup_s,
  input  wire logic           clk_qreqn,
  output      logic           clk_qacceptn,
  output      logic           clk_qdeny,
  output      logic           clk_qactive,
  input  wire logic           pwr_qreqn,
  output      logic           pwr_qacceptn,
  output      logic           pwr_qdeny,
  output      logic           pwr_qactive,
  input  wire logic           ext_gt_qreqn,
  output      logic           ext_gt_qacceptn,
  input  wire logic           cfg_gate_resp,
  output      logic [14-1 :0] memaddr,
  output      logic [128-1:0] memd,
  input  wire logic [128-1:0] memq,
  output      logic           memcen,
  output      logic [16-1 :0] memwen
);

  assign awready_s	     = 1'b0;   
  assign wready_s	       = 1'b0;
  assign bvalid_s	       = 1'b0;
  assign bid_s	         = 12'b0;
  assign bresp_s	       = 2'b0;
  assign arready_s	     = 1'b0;
  assign rvalid_s	       = 1'b0;
  assign rid_s	         = 12'b0;
  assign rdata_s	       = 128'b0;
  assign rresp_s	       = 2'b0;
  assign rlast_s	       = 1'b0;
  assign rpoison_s	     = 2'b0;
  assign clk_qacceptn    = 1'b0;
  assign clk_qdeny	     = 1'b0;
  assign clk_qactive     = 1'b0;
  assign pwr_qacceptn    = 1'b0;
  assign pwr_qdeny	     = 1'b0;
  assign pwr_qactive     = 1'b0;
  assign ext_gt_qacceptn = 1'b0;
  assign memaddr	       = 14'b0;
  assign memd	           = 128'b0;
  assign memcen	         = 1'b0;
  assign memwen	         = 16'b0;

endmodule

