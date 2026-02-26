// Stub of a 128b wide 8KB SRAM
//

module sram_8KB_stub (q, clk, cen, gwen, a, d, wen, stov, ema, emaw, emas, ret1n);

  input          clk;
  input          cen;
  input          gwen;
  input  [8:0]   a;
  input  [127:0] d;
  input  [127:0] wen;
  input          stov;
  input  [2:0]   ema;
  input  [1:0]   emaw;
  input          emas;
  input          ret1n;
  output [127:0] q;

  assign q = 128'b0;
 
endmodule
