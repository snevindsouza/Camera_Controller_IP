// Wrapper around stub of 128b wide 8KB SRAM
// 
 `timescale 1 ns / 1 ps

module sram_wrapper (
  input  logic         mem_clk, 
  input  logic [8:0]   mem_addr,
  input  logic [127:0] mem_d ,
  input  logic [15:0]  mem_wen,
  input  logic         mem_cen,
  input  logic         SCR1_CAM_DISP_SRAM_EMAS, 
  input  logic [1:0]   SCR1_CAM_DISP_SRAM_EMAW, 
  input  logic [2:0]   SCR1_CAM_DISP_SRAM_EMA,  
  output logic [127:0] mem_q                  
);

sram_8KB_stub U_sram_8KB (
  .q     (mem_q                  ),
  .clk   (mem_clk                ),
  .cen   (mem_cen                ),
  .gwen  (&mem_wen               ),
  .a     (mem_addr               ),
  .d     (mem_d                  ),
  .wen   ({{8{mem_wen[15]}},
           {8{mem_wen[14]}},
           {8{mem_wen[13]}},
           {8{mem_wen[12]}},
           {8{mem_wen[11]}},
           {8{mem_wen[10]}},
           {8{mem_wen[9]}},
           {8{mem_wen[8]}},
           {8{mem_wen[7]}},
           {8{mem_wen[6]}},
           {8{mem_wen[5]}},
           {8{mem_wen[4]}},
           {8{mem_wen[3]}},
           {8{mem_wen[2]}},
           {8{mem_wen[1]}},
           {8{mem_wen[0]}}}      ),
  .stov  (1'b0                   ),
  .ema   (SCR1_CAM_DISP_SRAM_EMA ),
  .emaw  (SCR1_CAM_DISP_SRAM_EMAW),
  .emas  (SCR1_CAM_DISP_SRAM_EMAS),
  .ret1n (1'b1                   )
);

endmodule
