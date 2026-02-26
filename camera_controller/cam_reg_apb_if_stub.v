// Camera Registers -  APB Interface Stub
module cam_reg_apb_if_stub
(
  // Ports to functional block
  output [1:0]  frmt_cam_out,
  output [1:0]  res_cam_out,
  output        intr_line_trn_tx_cmt_out,
  input         intr_line_trn_tx_cmt_in,
  input         intr_line_trn_tx_cmt_hw_wr_en_in,
  output        intr_line_input_in_cmt_out,
  input         intr_line_input_in_cmt_in,
  input         intr_line_input_in_cmt_hw_wr_en_in,
  output        intr_frm_trn_tx_cmt_out,
  input         intr_frm_trn_tx_cmt_in,
  input         intr_frm_trn_tx_cmt_hw_wr_en_in,
  output        intr_buffer_over_buf_ovr_out,
  input         intr_buffer_over_buf_ovr_in,
  input         intr_buffer_over_buf_ovr_hw_wr_en_in,
  output        intr_buffer_under_buf_und_out,
  input         intr_buffer_under_buf_und_in,
  input         intr_buffer_under_buf_und_hw_wr_en_in,
  output        intr_mask_buf_under_out,
  output        intr_mask_buf_over_out,
  output        intr_mask_frm_tx_cmt_out,
  output        intr_mask_line_in_cmt_out,
  output        intr_mask_line_tx_cmt_out,
  output        cam_ctrl_en_out,
  output        dma_ctrl_trig_mask_out,
  output        debug_ctrl_sram_buffer_out,
  output        debug_ctrl_debug_en_out,

  // System bus ports
  output        PREADY,
  output [31:0] PRDATA,
  output        PSLVERR,
  input  [6:0]  PADDR,
  input  [2:0]  PPROT,
  input         PSEL,
  input         PENABLE,
  input         PWRITE,
  input  [31:0] PWDATA,
  input  [3:0]  PSTRB,
  input         PCLK,
  input         PRESETn
);

                                              
  assign  frmt_cam_out                      =    2'b0;  
  assign  res_cam_out                       =    2'b0;  
  assign  intr_line_trn_tx_cmt_out          =    1'b0;  
  assign  intr_line_input_in_cmt_out        =    1'b0;  
  assign  intr_frm_trn_tx_cmt_out           =    1'b0;  
  assign  intr_buffer_over_buf_ovr_out      =    1'b0;  
  assign  intr_buffer_under_buf_und_out     =    1'b0;  
  assign  intr_mask_buf_under_out           =    1'b0;  
  assign  intr_mask_buf_over_out            =    1'b0;  
  assign  intr_mask_frm_tx_cmt_out          =    1'b0;  
  assign  intr_mask_line_in_cmt_out         =    1'b0;  
  assign  intr_mask_line_tx_cmt_out         =    1'b0;  
  assign  cam_ctrl_en_out                   =    1'b0;  
  assign  dma_ctrl_trig_mask_out            =    1'b0;  
  assign  debug_ctrl_sram_buffer_out        =    1'b0;  
  assign  debug_ctrl_debug_en_out           =    1'b0;  
  assign  PREADY                            =    1'b0;  
  assign  PRDATA                            =    32'b0;  
  assign  PSLVERR                           =    1'b0;  



endmodule
