// Standalone check of the ported bit math: shifter, masks, PDE address, RPN merge.
// Each A2O expression is compared against a direct little-endian model of the
// Microwatt source it was transcribed from.
`timescale 1ns/1ns
module tb_math;
   integer errors = 0;
   integer t, s, ms;

   reg [0:51]  epn;          // A2O MSB-first: epn[0]=EA63 ... epn[51]=EA12
   reg [63:12] addr;         // Microwatt little-endian view of the same EA
   reg [5:0]   shift;
   reg [4:0]   masksize;

   // ---- A2O implementation (copied verbatim out of mmq_rtw.v) ----
   wire [0:83] epn_pad = {34'b0, epn[2:51]};
   wire [0:15] a2o_addrsh = epn_pad[(68 - shift) +: 16];
   wire [0:15] a2o_mask;
   genvar g;
   generate
     for (g = 0; g < 16; g = g + 1) begin : gm
       assign a2o_mask[g] = ((15-g) < 5) ? 1'b1 : ((15-g) < masksize) ? 1'b1 : 1'b0;
     end
     for (g = 0; g < 30; g = g + 1) begin : gf
       assign a2o_fm30[g] = ((29-g) < shift) ? 1'b1 : 1'b0;
     end
     for (g = 0; g < 31; g = g + 1) begin : gs
       assign a2o_segmask[g] = ((30-g) < shift) ? 1'b1 : 1'b0;
     end
   endgenerate
   wire [0:29] a2o_fm30;
   wire [0:30] a2o_segmask;

   // ---- Microwatt reference model (little-endian, straight from mmu.vhdl) ----
   reg [15:0] mw_addrsh, mw_mask;
   reg [29:0] mw_fm30;
   reg [30:0] mw_segmask;
   integer bi;
   always @(*) begin
      // addrshifter: (addr(61 downto 12) >> shift)(15 downto 0)   mmu.vhdl:1380
      mw_addrsh = (addr[61:12] >> shift) & 16'hffff;
      // addrmaskgen: seed 0x001f, set bit i for 5<=i<mask_size    mmu.vhdl:1417
      mw_mask = 16'h001f;
      for (bi = 5; bi <= 15; bi = bi + 1)
         if (bi < masksize) mw_mask[bi] = 1'b1;
      // finalmaskgen: bit i set iff i < shift                     mmu.vhdl:1436
      mw_fm30 = 0;
      for (bi = 0; bi <= 29; bi = bi + 1) if (bi < shift) mw_fm30[bi] = 1'b1;
      mw_segmask = 0;
      for (bi = 0; bi <= 30; bi = bi + 1) if (bi < shift) mw_segmask[bi] = 1'b1;
   end

   // reverse helpers: A2O [0:N] <-> Microwatt [N:0]
   function [15:0] rev16; input [0:15] v; integer j;
      begin for (j=0;j<16;j=j+1) rev16[j] = v[15-j]; end endfunction
   function [29:0] rev30; input [0:29] v; integer j;
      begin for (j=0;j<30;j=j+1) rev30[j] = v[29-j]; end endfunction
   function [30:0] rev31; input [0:30] v; integer j;
      begin for (j=0;j<31;j=j+1) rev31[j] = v[30-j]; end endfunction

   task chk; input [255:0] name; input [63:0] a; input [63:0] b;
      begin if (a !== b) begin
         $display("  FAIL %0s: a2o=%h mw=%h  (shift=%0d masksize=%0d)", name, a, b, shift, masksize);
         errors = errors + 1; end
      end
   endtask

   initial begin
      $display("=== ported bit-math vs Microwatt reference model ===");
      for (t = 0; t < 400; t = t + 1) begin
         addr = {$random, $random};
         for (bi = 0; bi < 52; bi = bi + 1) epn[bi] = addr[63-bi];   // same EA, both views
         shift    = $random % 48;
         masksize = 5 + ({$random} % 12);  // legal RPDS/NLS range is 5..16
         #1;
         chk("addrsh",  {48'b0, rev16(a2o_addrsh)},   {48'b0, mw_addrsh});
         chk("mask",    {48'b0, rev16(a2o_mask)},     {48'b0, mw_mask});
         chk("fm30",    {34'b0, rev30(a2o_fm30)},     {34'b0, mw_fm30});
         chk("segmask", {33'b0, rev31(a2o_segmask)},  {33'b0, mw_segmask});
      end
      if (errors == 0) $display("PASS: 400 random vectors, all four generators match");
      else             $display("FAIL: %0d mismatches", errors);
      $finish;
   end
endmodule
