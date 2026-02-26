//
//=================================================================================================
// Module Name: camera_data_pipe
//
// Description: 
//   This is the Camera Data Pipeline of the Camera controller IP of **Mirafra's Ramanujan SOC**.
//   It receives data from the external camera sensors, organizes/accumulates it into frame lines,
//   writes into local memeory ping/pong buffers and issues triggers to on-chip DMA for transfer of
//   buffered partial frame data to pack the external frame buffer.
//
// Key Features:
//   - Captures pixel data sent by the camera sensor
//   - Accumulates Frame line data by selected pixel format
//   - Supports YUV420 (Planar and Interleaved), RGB888 and MJPEG formats
//   - DMA triggers to offload buffered data to system frame buffer
//   - Interrupts (Buffer over/under, Line/Frame xfer comp, line input done)
//=================================================================================================
//
`timescale 1ns / 1ps

module camera_data_pipe (

  input  logic         rst_n,                   // Active Low Reset Input
  
  // Camera DVP Interface
  input  logic         dvp_vsync,               // DVP Vertical Sync (pulse)
  input  logic         dvp_href,                // DVP Horizontal Ref (level)
  input  logic         dvp_pclk,                // DVP Pixel Clock
  input  logic [7:0]   dvp_data,                // DVP Data
  
  // Interrupt Outputs
  output logic         intr_line_tx_dn,         // Full line read by DMA        
  output logic         intr_line_in_dn,         // Full line from camera
  output logic         intr_frm_tx_dn,          // Full frame read by the DMA
  output logic         intr_buf_over_err,       // Memory buffer overrun 
  output logic         intr_buf_undr_err,       // Memory buffer underrun 
  
  // DMA Trigger Interface
  output logic         dma_trig_req,            // DMA Trig Request
  output logic [1:0]   dma_trig_req_type,       // DMA Trig Request Type
  input  logic         dma_trig_ack,            // DMA Trig Acknowledge
  input  logic [1:0]   dma_trig_ack_type,       // DMA Trig Acknowledge Type
  input  logic         dma_trig_mask,           // DMA Trig Mask
  
  // Memory Interface (compatible with ARM's SIE-300)
  input  logic [8:0]   memaddr,                 // Address
  input  logic [127:0] memd,                    // Data input
  output logic [127:0] memq,                    // Data Output
  input  logic         memcen,                  // Enable - Active Low
  input  logic [15:0]  memwen,                  // Write En / Byte- Active Low
  
  // Configuration Inputs
  input  logic [1:0]   res,                     // Resolution select
  input  logic [1:0]   frmt,                    // Format select
  input  logic         en_cam,                  // Camera Interface Enable
  
  // Interrupt Mask inputs - Active high
  input  logic         line_tx_dn_mask,
  input  logic         line_in_dn_mask,
  input  logic         frm_tx_dn_mask,
  input  logic         buf_over_err_mask,
  input  logic         buf_undr_err_mask,

  // Interrupt ack inputs - Active high
  input logic          intr_line_tx_dn_ack,
  input logic          intr_frm_tx_dn_ack,
  input logic          intr_line_in_dn_ack,
  input logic          intr_buf_over_err_ack,
  input logic          intr_buf_undr_err_ack,

  // Debug Control Inputs
  input  logic         debug_sram_sel,          // Select SRAM in Debug Mode
  input  logic         debug_en,                // Enable the Debug Mode    
  input  logic         SCR1_CAM_SRAM_EMAS,      // Mem EMAS Control from SCR
  input  logic [1:0]   SCR1_CAM_SRAM_EMAW,      // Mem EMAW Control from SCR
  input  logic [2:0]   SCR1_CAM_SRAM_EMA,       // Mem EMA Control from SCR

  // Debug Fanout Signals to Debug Mux
  output logic         dbg_cam_buffer_a_select,
  output logic         dbg_cam_buffer_a_full,
  output logic         dbg_cam_buffer_b_full,
  output logic         dbg_cam_buffer_a_empty,
  output logic         dbg_cam_buffer_b_empty
);

  // ==========================================================================
  // Parameter Definitions
  // ==========================================================================

  // State machine states
  typedef enum logic [2:0] {
    IDLE         = 3'b000,
    WAIT_LINE    = 3'b001,
    CAPTURE_LINE = 3'b010,
    TRIGGER_DMA  = 3'b011,
    CHECK_FRAME  = 3'b100,
    FRAME_DONE   = 3'b101
  } main_state_t;

  typedef enum logic [1:0] {
    BUF_A_WRITE = 2'b00,
    BUF_B_WRITE = 2'b01,
    BUF_SWITCH  = 2'b10
  } buf_state_t;

  localparam RGB              = 2'b00; // RGB888
  localparam MJPEG            = 2'b01;
  localparam YUV_I            = 2'b10; // YUV420-Interleaved: Y/U/V as rcvd
  localparam YUV_P            = 2'b11; // YUV420-Planar: Y/U/V cached seperate 
  localparam MEMWID           = 'd128;
  localparam U_MEM_START_ADDR = 'd300;
  localparam V_MEM_START_ADDR = 'd400;
  localparam BYTE_COUNT_W     = 'd15;
  localparam BYTE_COUNT_Y_1_W = 'd30;
  localparam BYTE_COUNT_U_W   = 'd61;
  localparam BYTE_COUNT_Y_2_W = 'd62;
  localparam BYTE_COUNT_V_W   = 'd63;
  localparam U_START_ADD      = 'd300;
  localparam V_START_ADD      = 'd400;
  localparam DMA_DENY         = 2'b10;

  // ==========================================================================
  // Internal Signals and Registers
  // ==========================================================================

  // Configuration derived signals
  logic [11:0] line_pix;
  logic [11:0] frm_lines;
  logic [1:0]  pix_bytes;
  logic [15:0] line_bytes;
  logic [8:0]  last_addr;
  logic [11:0] line_pix_comp;
  logic [8:0]  v_mem_depth;
  logic        frmt_yuv;
  logic        frmt_yuv_i;
  logic        frmt_yuv_p;

  // State machine registers
  main_state_t main_state;
  main_state_t main_state_next;
  buf_state_t  buf_state;
  buf_state_t  buf_state_next;
  
  // Data accumulation
  logic [127:0] data_accumulator;    // Accumulating data for RGB and Y for YUV
  logic [127:0] data_accumulator_u;  // Accumulating data for U of YUV
  logic [127:0] data_accumulator_v;  // Accumulating data for V of YUV
  logic [5:0]   byte_count;
  logic [127:0] write_data;
  logic         write_enable;
  logic         write_enable_early;

  // Address generation
  logic [8:0]   write_addr;
  logic [8:0]   write_addr_y;
  logic [8:0]   write_addr_u;
  logic [8:0]   write_addr_v;
  logic [8:0]   read_addr;
  logic         addr_increment;
  logic         byte_count_write;

  // Buffer control
  logic         buffer_a_select;
  logic         buffer_a_full;
  logic         buffer_a_empty;
  logic         buffer_b_full;
  logic         buffer_b_empty;
  logic         current_buf_full;
  logic         current_buf_empty;

  // Line and frame counters
  logic [11:0]  pixel_count;
  logic [1:0]   pixel_bytes_count;
  logic [11:0]  line_in_count;
  logic [11:0]  line_tx_count;
  logic         yuv_2ndline;
  logic         yuv_1stline;
  logic         line_incr;
  logic         yuv_2ndline_d1;
  logic         yuv_2ndline_d2;

  // Synchronizers for DVP signals
  logic         dvp_vsync_sync;
  logic         dvp_vsync_edge;
  logic         dvp_href_d;
  logic         dvp_href_edge;
  logic         dvp_href_edge_d;
  logic         dvp_vsync_r;
  logic         dvp_href_r;
  logic         dvp_href_f;

  // Interrupt generation
  logic         line_tx_dn_int;
  logic         line_in_dn_int;
  logic         frm_tx_dn_int;
  logic         buf_over_err_int;
  logic         buf_undr_err_int;
  logic         intr_line_tx_dn_set;  
  logic         intr_line_in_dn_set;  
  logic         intr_frm_tx_dn_set;   
  logic         intr_buf_over_err_set;
  logic         intr_buf_undr_err_set;
  logic         ovrrn_int_prv_frame;
  
  // DMA interface
  logic         dma_req_pending;
  logic         last_dma_tx;
  logic         dma_re_request;
  
  // Dual-port SRAM instances (8KB each = 512 x 128-bit)
  logic [127:0] sram_a_q;
  logic [127:0] sram_b_q;
  logic [15:0]  sram_a_wen;
  logic [15:0]  sram_b_wen;
  logic         sram_a_cen;
  logic         sram_b_cen;
  logic [8:0]   sram_a_addr;
  logic [8:0]   sram_b_addr;
  logic [127:0] sram_a_d;
  logic [127:0] sram_b_d;

  // Routing camera data pipe internals to debug mux for observability
  assign dbg_cam_buffer_a_select = buffer_a_select; 
  assign dbg_cam_buffer_a_full   = buffer_a_full; 
  assign dbg_cam_buffer_b_full   = buffer_b_full; 
  assign dbg_cam_buffer_a_empty  = buffer_a_empty;  
  assign dbg_cam_buffer_b_empty  = buffer_b_empty; 
  
  // ==========================================================================
  // Configuration Logic
  // ==========================================================================
  assign frmt_yuv   = (frmt == YUV_I) || (frmt == YUV_P);
  assign frmt_yuv_i = (frmt == YUV_I);
  assign frmt_yuv_p = (frmt == YUV_P);
  
  //********* Determine the values for each format and resolution *************
  always_comb begin
    v_mem_depth = 9'h0;
    // Resolution configuration: Determine number of pixels per line (line_pix)
    // and number of lines per frame (frm_lines) based on resolution                                             
    case (res)                                                             
      2'd0: begin                 // HD 720p Resolution
              line_pix  = 1280;
              frm_lines = 720;
            end   
      2'd1: begin                 // Full HD Resolution
              line_pix  = 1920;
              frm_lines = 1080;
            end     
      2'd2: begin                 // VGA Resolution
              line_pix  = 640;
              frm_lines = 480;
            end              
      2'd3: begin                 // QVGA Resolution
              line_pix  = 320;
              frm_lines = 240;
            end               
    endcase                                                              
    
    // Calculate number of bytes in a pixel (pix_bytes), num bytes in a line
    // (line_bytes) & last memory address (last_addr) for each format
    //
    // v_mem_depth is used in YUV420 Planar format to determine the last_addr
    // for a given resolution. For YUV420 format, two lines are stored in local
    // buffers, instead of 1. Hence, line interrupts/DMA requests are triggered
    // after two lines, when in YUV420 format. line_pix_comp will be equivalent
    // to two lines for YUV420 format and one line for RGB/MJPEG. line_in_count
    // counts the number of incoming lines: bit [0] is used to distinguish
    // between YYYY and YUYV lines for YUV420 format.

    // ***** Calculations specific to YUV420 *****
    if (frmt_yuv) begin
      pix_bytes     = line_in_count[0] ? 2'd2 : 2'd1;  // Even YYYY, odd YUYV
      line_bytes    = line_pix * pix_bytes;            // Number of bytes/line
      line_pix_comp = 2 * line_pix;  //  2 lns (YYYY/YUYV) triggers DMA/intrpt
      if (frmt_yuv_p) begin
        // YUYV line, V/U accounts for 1/2 the pixels
        v_mem_depth = (line_pix * 8) / (2 * MEMWID);
        // YUV planar address step is non-linear for Y/U/V data
        last_addr   = V_MEM_START_ADDR + v_mem_depth;
      end else // Linear address step: byte count for 2 lines is line_pix * 3
        // YUV interleaved address step is linear for Y/U/V data
        last_addr   = (line_pix * 3 * 8) / MEMWID;

    // ***** Calculations for RGB888 / MJPEG *****
    end else begin
      pix_bytes     = 2'd3;
      line_bytes    = line_pix * pix_bytes;
      line_pix_comp = line_pix;
      last_addr     = (line_bytes * 8)/ MEMWID; // Convert line to 128-b words
    end
  end         

  // Differentiate YUV 1st vs 2nd lines using line input counter
  assign yuv_1stline = frmt_yuv && ~line_in_count[0];
  assign yuv_2ndline = frmt_yuv && line_in_count[0];

  // Delaying yuv_2ndline till the required calculation/assignments are done
  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n) begin
      yuv_2ndline_d1 <= 1'b0;
      yuv_2ndline_d2 <= 1'b0;
    end else begin
      yuv_2ndline_d1 <= yuv_2ndline;    // for Y-addr increment allignment
      yuv_2ndline_d2 <= yuv_2ndline_d1; // for data accumulator allignment
    end

  // ========================================================================
  // DVP Signal Capture
  // ========================================================================

  // Detect edges for VSYNC and HREF
  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n) begin
      dvp_vsync_r    <= 1'b0;
      dvp_vsync_sync <= 1'b0;
      dvp_href_r     <= 1'b0;
      dvp_href_d     <= 1'b0;
    end else if (~en_cam) begin   // Synchronous reset on camera disable
      dvp_vsync_r    <= 1'b0;
      dvp_vsync_sync <= 1'b0;
      dvp_href_r     <= 1'b0;
      dvp_href_d     <= 1'b0;
    end else begin
      dvp_vsync_r    <= dvp_vsync;
      dvp_vsync_sync <= dvp_vsync_r;
      dvp_href_r     <= dvp_href;    
      dvp_href_d     <= dvp_href_r;
    end
  assign dvp_vsync_edge = dvp_vsync_sync & ~dvp_vsync_r; 
  assign dvp_href_edge  = ~dvp_href_r & dvp_href;

  // ==========================================================================
  // Main Control State Machine
  // ==========================================================================

  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n)
      main_state <= IDLE;
    else
      if (~en_cam)
        main_state <= IDLE;
      else
        main_state <= main_state_next;

  always_comb begin
    main_state_next = main_state;
    
    case (main_state)
              // FSM starts on VSYNC edge detect if camera cntrlr is enabled
              IDLE: if (dvp_vsync_edge && en_cam) 
                      main_state_next = WAIT_LINE;    // Wait for start of line
      
         WAIT_LINE: if (dvp_href)                     // HREF edge detected
                      main_state_next = CAPTURE_LINE;
      
      CAPTURE_LINE: if (dvp_vsync_edge)               // Premature VSYNC 
                      main_state_next = WAIT_LINE;
                    else if (!dvp_href && (pixel_count == line_pix) && frmt_yuv)
                      main_state_next = WAIT_LINE;    // YUV420 format - 2 lines
                    else if (!dvp_href && (pixel_count == line_pix_comp))
                      main_state_next = TRIGGER_DMA;  // Trigger DMA
      
       TRIGGER_DMA: main_state_next = CHECK_FRAME;    // Check frame completion
              
       CHECK_FRAME: if (line_in_count == frm_lines)   // Is Frame completed?
                      main_state_next = FRAME_DONE;
                    else
                      main_state_next = WAIT_LINE;
      
        FRAME_DONE: main_state_next = IDLE;
      
           default: main_state_next = IDLE;
    endcase
  end
  
  // ==========================================================================
  // Data Accumulation
  // ==========================================================================
  // The data from camera sensor is accumulated to 128bits before pushing
  // into the ping or pong local memory buffers.
  // MJPEG Format: only compression mode-4 is supported and the first 2 bytes
  // of every line in this mode represents the line length, which is ignored.
  // Data accumulation starts from the 3rd byte of every line for MJPEG.
  // YUV420-Planar Format: data for Y/U/V is seperately acucmulated before 
  // transfers to the ping/pong local buffers

  always_ff @(posedge dvp_pclk or negedge rst_n) begin
    if (~rst_n) begin
      data_accumulator   <= 128'h0;
      data_accumulator_u <= 128'h0;
      data_accumulator_v <= 128'h0;
      byte_count         <= 6'h0;
      pixel_count        <= 12'h0;
      pixel_bytes_count  <= 2'h0;
    end else begin
      // Initialize variables, when camera controller disabled by config
      if (~en_cam) begin 
        data_accumulator  <= 128'h0;
        byte_count        <= 6'h0;
        pixel_count       <= 12'h0;
        pixel_bytes_count <= 2'h0;
      // Early data capture- all formats, except MJPEG (1st 2 bytes ignored)
      end else if ((main_state == WAIT_LINE) && (frmt != MJPEG)) begin
        // Capture 1st data byte, as HREF asserted with valid data on the bus
        data_accumulator <= {dvp_data, data_accumulator[127:8]};
        if (dvp_href) begin
          byte_count     <= 6'd1; // Start the count if HREF is detected
          if (yuv_1stline) begin
            pixel_bytes_count <= 2'b00; // Increments only for YUV second line
            pixel_count       <= 12'h1; // Start counting immediately
          end else if (yuv_2ndline)  begin
            pixel_bytes_count <= 2'b01;
            pixel_count       <= pixel_count; // Continue count over 1st line
          end else begin
            pixel_bytes_count <= 2'b01;       // Calculations for RGB/MJPEG
            pixel_count       <= 12'h0; 
          end
        end
      // Ignore first 2 bytes for MJPEG
      end else if ((main_state == CAPTURE_LINE) &&
                   ((frmt == MJPEG) ? dvp_href_d : 1'b1)) begin
        // Accumulate 8-bit data into 128-bit word
        if (yuv_2ndline_d1 && frmt_yuv_p) begin    // YUV-Planar conversion
          // byte_count increments till 128bit data is accumulated for U and V
          // Max byte_count of 63 (16 bytes U + 16 bytes V + 32 bytes Y) 
          byte_count <= (byte_count == 6'd63) ? 6'd0 : byte_count + 6'd1;
          // Accumulate data for Y,U and V seperately based on byte_count value
          if (~byte_count[0])                    // Accumulating Y data in YUYV
            data_accumulator <= {dvp_data, data_accumulator[127:8]};                
          else if (~byte_count[1])               // Accumulating U data in YUYV
            data_accumulator_u <= {dvp_data, data_accumulator_u[127:8]};            
          else                                   // Accumulating V data in YUYV
            data_accumulator_v <= {dvp_data, data_accumulator_v[127:8]}; 
        end else begin
          // byte_count increments till 128bit data is accumulated
          byte_count <= (byte_count == 6'd15) ? 6'd0 : byte_count + 6'd1;
          data_accumulator <= {dvp_data, data_accumulator[127:8]}; 
        end
        // Count pixels based on format                       
        if (pixel_bytes_count == (pix_bytes - 2'b1)) begin     
          pixel_bytes_count <= 2'h0;                              
          pixel_count       <= pixel_count + 12'b1;                      
        end else
          pixel_bytes_count <= pixel_bytes_count + 2'b1;    
        
      end else if (main_state == TRIGGER_DMA) begin
        pixel_count       <= 12'h0;
        byte_count        <= 6'h0;
        pixel_bytes_count <= 2'h0;
      end
    end
  end

  //************************* Write enables *********************************//
  // Except for YUV-P-2nd-line, wait for 128b accumulation before mem write
  // For YUV Planar 2nd line (YUYV), mem write once 128b accumulated on U/V
  // Since Y data is double that of U and V, there will be 2 writes to the
  // Y location for each write to U and V locations

  assign byte_count_write = (yuv_2ndline_d2 && frmt_yuv_p) ? 
                              ((byte_count == BYTE_COUNT_Y_1_W) ||
                               (byte_count == BYTE_COUNT_Y_2_W) ||
                               (byte_count == BYTE_COUNT_U_W) ||
                               (byte_count == BYTE_COUNT_V_W)) :
                              (byte_count == BYTE_COUNT_W);
  
  // Delay write_enable to allign with full 128b data accumulation
  assign write_enable_early = (main_state == CAPTURE_LINE) &&
                              byte_count_write && dvp_href_d;  
  
  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n)
      write_enable <= 1'b0;
    // Sync reset write_enable on camera ctrl disable
    else if (~en_cam)
      write_enable <= 1'b0;
    else
      write_enable <= write_enable_early;
  
  // ***** Write Data assignment *****
  // Data is aligned by design for all formats,
  // except for YUV420 Planar U/V data, which requires customized timing
  assign write_data = (yuv_2ndline_d2 && frmt_yuv_p) ?
                       ((byte_count == (BYTE_COUNT_U_W + 6'b1)) ?
                         data_accumulator_u :    // Write U data @byte #62
                          (~|byte_count ?
                            data_accumulator_v : // Write V data @byte #64
                            data_accumulator))
                       : data_accumulator;

  // ===========================================================================
  // Buffer Management
  // ===========================================================================
  // Memory Buffer selection for write; Default: BUF_A
  // When BUF A is selected for write, BUF B is available for read and viceversa
  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n)
      buf_state <= BUF_A_WRITE;
    // Sync reset on camera ctrl disable
    else if (~en_cam)
      buf_state <= BUF_A_WRITE;
    else
      buf_state <= buf_state_next;
  
  // ***** Ping-Pong Buffer switching *****
  always_comb begin
    buf_state_next = buf_state;
    case (buf_state)
      BUF_A_WRITE: if (buffer_a_full)
                     buf_state_next = BUF_B_WRITE;
      BUF_B_WRITE: if (buffer_b_full)
                     buf_state_next = BUF_SWITCH;
       BUF_SWITCH: if (!buffer_a_full || !buffer_b_full)
                     buf_state_next = buffer_a_empty ? BUF_A_WRITE : BUF_B_WRITE;
          default: buf_state_next = BUF_A_WRITE;
    endcase
  end
  
  // Buffer selection logic
  assign buffer_a_select = (buf_state == BUF_A_WRITE);
  
  // Address generation
  // YUV-P-2nd-line, address to be calculated seperately for Y/U/V
  always_ff @(posedge dvp_pclk or negedge rst_n) begin
    if (~rst_n) begin
      write_addr   <= 9'h0;
      write_addr_y <= 9'h0;          // Write address for Y data of YUV Planar
      write_addr_u <= U_START_ADD;   // Write address for U data of YUV Planar
      write_addr_v <= V_START_ADD;   // Write address for V data of YUV Planar
    // Sync reset on camera ctrl disable
    end else if (~en_cam) begin
      write_addr <= 9'h0;
      write_addr_y <= 9'h0;                                                                    
      write_addr_u <= U_START_ADD;                                     
      write_addr_v <= V_START_ADD;                                     
    end else begin
      // Write to Y/U/V locations for YUV planar second line
      if (yuv_2ndline_d1 && frmt_yuv_p && write_enable_early) begin
        if ((byte_count == BYTE_COUNT_Y_1_W) ||
            (byte_count == BYTE_COUNT_Y_2_W)) begin
          write_addr   <= write_addr_y;          // Write address for Y data
          write_addr_y <= write_addr_y + 9'b1;   // Increment Y write address
        end else if (byte_count == BYTE_COUNT_U_W) begin
          write_addr   <= write_addr_u;          // Write address for U data
          write_addr_u <= write_addr_u + 9'b1;   // Increment U write address
        end else if (byte_count == BYTE_COUNT_V_W) begin
          write_addr   <= write_addr_v;          // Write address for V data
          write_addr_v <= write_addr_v + 9'b1;   // Increment V write address
        end else begin
          write_addr   <= write_addr_y;          // Default: point to y address
        end
      // Initilaize address after yuv_2ndline for YUV Planar
      end else if (yuv_2ndline_d1 && frmt_yuv_p && write_enable) begin
        if (write_addr == (last_addr - 9'b1)) begin
          write_addr   <= 9'h0;
          write_addr_y <= 9'h0;
          write_addr_u <= U_START_ADD;
          write_addr_v <= V_START_ADD;
        end
      end else if ((frmt_yuv_p ? ~yuv_2ndline_d1 : 1'b1) && write_enable) begin
        write_addr_y <= write_addr + 9'b1; // Y data Add - continues 1st line Y
        write_addr_u <= U_START_ADD;
        write_addr_v <= V_START_ADD;
        if (write_addr == (last_addr - 9'b1))
          write_addr <= 9'h0;              // Re-initialize write addr
        else
          write_addr <= write_addr + 9'b1; // Increment write address
      end
    end
  end //always


  // Buffer full/empty flag generation
  // Buffer is flagged full on write to last address
  // Buffer is flagged empty once data is read
  always_ff @(posedge dvp_pclk or negedge rst_n) begin
    if (~rst_n) begin
      buffer_a_full  <= 1'b0;
      buffer_b_full  <= 1'b0;
      buffer_a_empty <= 1'b1;
      buffer_b_empty <= 1'b1;
    end else if (debug_en) begin                
      buffer_a_full  <= buffer_a_full;
      buffer_b_full  <= buffer_b_full;
      buffer_a_empty <= buffer_a_empty;
      buffer_b_empty <= buffer_b_empty;
    // Sync reset on camera ctrl disable or overrun error
    end else if ((~en_cam) || (dvp_vsync_edge && ovrrn_int_prv_frame)) begin
      buffer_a_full  <= 1'b0;
      buffer_b_full  <= 1'b0;
      buffer_a_empty <= 1'b1;
      buffer_b_empty <= 1'b1;
    end else begin
      // Buffer A flags
      if (buffer_a_select && write_enable) begin
        buffer_a_empty <= 1'b0;                     
        if (write_addr == (last_addr -1'b1))
          buffer_a_full <= 1'b1;                   
      end else if (!buffer_a_select && !memcen && (&memwen)) begin  
        buffer_a_full <= 1'b0;                                      
        if (memaddr == (last_addr -1'b1))
          buffer_a_empty <= 1'b1;
      end
      
      // Buffer B flags
      if (!buffer_a_select && write_enable) begin
        buffer_b_empty <= 1'b0;                      
        if (write_addr == (last_addr -1'b1))
          buffer_b_full <= 1'b1;                   
      end else if (buffer_a_select && !memcen && (&memwen)) begin
        buffer_b_full <= 1'b0;                                      
        if (memaddr == (last_addr -1'b1))
          buffer_b_empty <= 1'b1;
      end
    end
  end
  
  // ==========================================================================
  // SRAM Selection 
  // ==========================================================================
  
  // In Debug mode, either buff A or buff B is selected for both read & write
  // Else, ping or pong buffer is selected by buffer_a_select
  always_comb begin
    if (debug_en) begin
      sram_a_addr = memaddr;
      sram_b_addr = memaddr;
      sram_a_d    = memd;
      sram_b_d    = memd;
      sram_a_cen  = debug_sram_sel ? 1'b1 : memcen; // SRAM A: debug_sram_sel=0
      sram_b_cen  = debug_sram_sel ? memcen : 1'b1; // SRAM B: debug_sram_sel=1
      sram_a_wen  = debug_sram_sel ? {16{1'b1}} : memwen;
      sram_b_wen  = debug_sram_sel ? memwen : {16{1'b1}};
      memq        = debug_sram_sel ? sram_b_q : sram_a_q;
    end else begin
      sram_a_addr = buffer_a_select ? write_addr : memaddr;
      sram_b_addr = buffer_a_select ? memaddr : write_addr;
      sram_a_d    = buffer_a_select ? write_data : memd;
      sram_b_d    = buffer_a_select ? memd : write_data;
      sram_a_cen  = buffer_a_select ? !write_enable : memcen;
      sram_b_cen  = buffer_a_select ? memcen : !write_enable;
      sram_a_wen  = buffer_a_select ? {16{!write_enable}} : memwen;
      sram_b_wen  = buffer_a_select ? memwen : {16{!write_enable}};
      memq        = buffer_a_select ? sram_b_q : sram_a_q;
    end
  end

  // ==========================================================================
  // Line and Frame Counters
  // ==========================================================================

  // Increment line input counter once all pixels are recevied for a line
  // YUV-P-2nd-line: pixel_count compared to line_pix_comp since mid-count
  assign line_incr = ((main_state == CAPTURE_LINE) && 
                      (pixel_bytes_count == (pix_bytes - 2'b1))) ?
                        (yuv_2ndline ? 
                          (pixel_count == (line_pix_comp - 12'b1)) :
                          (pixel_count == (line_pix - 12'b1))) :
                        1'b0;
 
  // Calculate line input count and line transfer count
  always_ff @(posedge dvp_pclk or negedge rst_n) begin
    if (~rst_n) begin
        line_in_count <= 12'h0;
        line_tx_count <= 12'h0;
    // Sync reset on camera ctrl disable
    end else if (~en_cam) begin
        line_in_count <= 12'h0;
        line_tx_count <= 12'h0;
    end else begin
      // Line input counter
      if (main_state == IDLE)                                
          line_in_count <= 12'h0;
      else if (line_incr)
          line_in_count <= line_in_count + 12'd1;
        
      // Line transmit counter
      // On VSYNC negegde, reset tx_count if prev frame over-ran  
      if (dvp_vsync_edge && ovrrn_int_prv_frame)
        line_tx_count <=  12'd0;  
      else if (line_tx_dn_int)  // Increment counter on line transmit 
        if (frmt_yuv)
          if (line_tx_count == (frm_lines - 12'd2)) // YUV-P cntr incr by 2
            line_tx_count <=  12'd0;         
          else
            line_tx_count <= line_tx_count + 12'd2; // YUV-P: tx_int/ 2 lines
        else  
          if (line_tx_count == (frm_lines - 12'd1)) // Compare w/ 2nd last 
            line_tx_count <= 12'd0;
          else
            line_tx_count <= line_tx_count + 12'd1;
    end
  end //end always
  
  // ==========================================================================
  // DMA Trigger Interface
  // ==========================================================================
  // Execute in the TRIGGER_DMA state unless DMA is masked
  // Keep re-issuing DMA request, if denial recieved in ack of prev req
  always_ff @(posedge dvp_pclk or negedge rst_n) begin
    if (~rst_n) begin
      dma_req_pending <= 1'b0;
      dma_re_request  <= 1'b0;         
    // Sync reset on Debug enable or camera controller disable
    end else if (debug_en || ~en_cam) begin
      dma_req_pending <= 1'b0;
      dma_re_request  <= 1'b0;
    end else begin
      if (main_state == TRIGGER_DMA && !dma_req_pending && !dma_trig_mask)
        dma_req_pending <= 1'b1;
      else if (dma_trig_ack) begin  
        dma_req_pending <= 1'b0;
        if (dma_trig_ack_type[1:0] == DMA_DENY)
          dma_re_request <= 1'b1;
      end else if (dma_re_request) begin // Req again since prev req is denied
        dma_req_pending <= 1'b1;
        dma_re_request  <= 1'b0;
      end 
    end
  end
  
  // YUV planar: increment line count by 2
  assign last_dma_tx       = (line_tx_count == ((frmt_yuv) ? 
                                (frm_lines-2'd2) : (frm_lines-2'd1)));
  assign dma_trig_req      = dma_req_pending;
  // Request type - 2'b10: Block and 2'b11:Last Block for ARM's DMA350
  assign dma_trig_req_type = last_dma_tx ? 2'b11 : 2'b10;
  
  // ==========================================================================
  // Interrupt Generation
  // ==========================================================================
  
  // Line input done - 2 lines for YUV planar, 1 line for all other formats 
  assign  line_in_dn_int = ((main_state == CAPTURE_LINE) && 
                            (pixel_count == (line_pix_comp - 12'b1)) && 
                            (pixel_bytes_count == (pix_bytes - 2'b1)));
 
  // Line transfer done - when last address is read from active buffer
  // Mask this interrupt in debug mode to prevent a false trigger
  assign  line_tx_dn_int = debug_en ? 1'b0 : (!memcen && (&memwen) && 
                                              (memaddr == (last_addr - 9'b1)));   
  
  // Frame transfer done once all lines in a frame is transferred
  // For YUV, tx count is incremented by 2
  assign  frm_tx_dn_int = line_tx_dn_int && 
                          (frmt_yuv ? (line_tx_count == (frm_lines - 12'd2)) :
                                      (line_tx_count == (frm_lines - 12'd1)));  

  // Buffer overflow err - when line write starts + target buffer is NOT empty
  // Igore YUV 2nd line check since memory is already written with first line
  // Buffer empty check ensures that prior DMA transfer had completed
  assign  buf_over_err_int = ((main_state == WAIT_LINE) && dvp_href_edge &&
                             ~yuv_2ndline &&
                             ~(buffer_a_empty || buffer_b_empty));

  // Buffer underflow err - both buffers are empty + memory read is attempted
  assign  buf_undr_err_int = (~memcen && (&memwen) &&
                              buffer_a_empty && buffer_b_empty);
  
  // Apply interrupt masks
  assign intr_line_tx_dn_set   = line_tx_dn_int & ~line_tx_dn_mask;
  assign intr_line_in_dn_set   = line_in_dn_int & ~line_in_dn_mask;
  assign intr_frm_tx_dn_set    = frm_tx_dn_int & ~frm_tx_dn_mask;
  assign intr_buf_over_err_set = buf_over_err_int & ~buf_over_err_mask;
  assign intr_buf_undr_err_set = buf_undr_err_int & ~buf_undr_err_mask;
  
  // If overrun interrupt occured in prev frame, reset all pointers/counters
  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n)
      ovrrn_int_prv_frame <= 1'b0;
    else if (dvp_vsync_edge)
      ovrrn_int_prv_frame <= 1'b0;
    else if (intr_buf_over_err_set)
      ovrrn_int_prv_frame <= 1'b1;

  // Interrupt set/reset flops 
  // Assert interrupts when set pusles are detected
  // De-assert on Ack
  always_ff @(posedge dvp_pclk or negedge rst_n)
    if (~rst_n) begin
      intr_line_tx_dn   <= 1'b0;
      intr_line_in_dn   <= 1'b0;
      intr_frm_tx_dn    <= 1'b0;
      intr_buf_over_err <= 1'b0;
      intr_buf_undr_err <= 1'b0;
    // Sync reset interrupts on Debug enable or camera controller disable
    end else if (debug_en || ~en_cam) begin
      intr_line_tx_dn   <= 1'b0;
      intr_line_in_dn   <= 1'b0;
      intr_frm_tx_dn    <= 1'b0;
      intr_buf_over_err <= 1'b0;
      intr_buf_undr_err <= 1'b0;
    end else begin
      intr_line_tx_dn   <= (intr_line_tx_dn_set || intr_line_tx_dn) &&
                           ~intr_line_tx_dn_ack;
      intr_line_in_dn   <= (intr_line_in_dn_set || intr_line_in_dn) &&
                           ~intr_line_in_dn_ack;
      intr_frm_tx_dn    <= (intr_frm_tx_dn_set || intr_frm_tx_dn) &&
                           ~intr_frm_tx_dn_ack;      
      intr_buf_over_err <= (intr_buf_over_err_set || intr_buf_over_err) &&
                           ~intr_buf_over_err_ack;      
      intr_buf_undr_err <= (intr_buf_undr_err_set || intr_buf_undr_err) &&
                           ~intr_buf_undr_err_ack;
    end    

  // ==========================================================================
  // SRAM Instances 
  // ==========================================================================
  
  // SRAM Buffer A (8KB = 512 x 128-bit)
  sram_wrapper sram_buffer_a (
      .mem_clk                 (dvp_pclk          ),
      .mem_addr                (sram_a_addr       ),
      .mem_d                   (sram_a_d          ),
      .mem_q                   (sram_a_q          ),
      .mem_cen                 (sram_a_cen        ),
      .mem_wen                 (sram_a_wen        ),
      .SCR1_CAM_DISP_SRAM_EMAS (SCR1_CAM_SRAM_EMAS),    
      .SCR1_CAM_DISP_SRAM_EMAW (SCR1_CAM_SRAM_EMAW),    
      .SCR1_CAM_DISP_SRAM_EMA  (SCR1_CAM_SRAM_EMA )      
  );
  
  // SRAM Buffer B (8KB = 512 x 128-bit)
  sram_wrapper sram_buffer_b (
      .mem_clk                 (dvp_pclk          ),
      .mem_addr                (sram_b_addr       ),
      .mem_d                   (sram_b_d          ),
      .mem_q                   (sram_b_q          ),
      .mem_cen                 (sram_b_cen        ),
      .mem_wen                 (sram_b_wen        ),
      .SCR1_CAM_DISP_SRAM_EMAS (SCR1_CAM_SRAM_EMAS),        
      .SCR1_CAM_DISP_SRAM_EMAW (SCR1_CAM_SRAM_EMAW),        
      .SCR1_CAM_DISP_SRAM_EMA  (SCR1_CAM_SRAM_EMA )          
  );

endmodule

