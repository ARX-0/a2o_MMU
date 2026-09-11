// End-to-end walk test for mmq_rtw: behavioural L2 + a real radix tree in memory.
// Tree layout follows microwatt/reference/mmu_test/mmu.c:125-138 (RTS=8, RPDS=9).
`timescale 1ns/1ns
`include "tri_a2o.vh"
`include "mmu_a2o.vh"

module tb_walk;
   reg clk = 0;  always #5 clk = ~clk;
   wire vdd_w = 1'b1;
   wire gnd_w = 1'b0;
   // nclk[0] = clock, nclk[1] = sreset (tri_rlmlatch_p.v:77). Without a real
   // reset pulse every latch starts X, so this must be driven.
   reg sreset = 1'b1;
   wire [0:`NCLK_WIDTH-1] nclk_w;
   assign nclk_w = {clk, sreset, {(`NCLK_WIDTH-2){1'b0}}};
   integer errors = 0, checks = 0;

   // ---------------- behavioural memory holding the radix tree ----------------
   // sparse: assoc array of 8-byte doublewords keyed by real address
   reg [63:0] mem [0:(1<<20)-1];       // indexed by RA>>3, low 8MB of RA space
   function [63:0] rd; input [41:0] ra; begin rd = mem[ra[22:3]]; end endfunction

   // 64-bit so that [55:12] / [55:8] part-selects are in range (an out-of-range
   // select yields X in Icarus, which silently poisons the whole tree)
   localparam [63:0] PTCR_BASE  = 64'h0000_0000_0010_0000;   // partition table
   localparam [63:0] PROC_BASE  = 64'h0000_0000_0020_0000;   // process table
   localparam [63:0] L1_BASE    = 64'h0000_0000_0030_0000;   // root directory
   localparam [63:0] L2_BASE    = 64'h0000_0000_0040_0000;
   localparam [63:0] L3_BASE    = 64'h0000_0000_0050_0000;
   localparam [63:0] L4_BASE    = 64'h0000_0000_0060_0000;
   localparam [63:0] TARGET_RA  = 64'h0000_0000_0080_0000;

   // radix field builders, Microwatt little-endian bit numbering
   function [63:0] mk_pde;  // directory: V=1 L=0 NLB NLS
      input [55:8] nlb; input [4:0] nls;
      begin mk_pde = 64'b0; mk_pde[63]=1'b1; mk_pde[62]=1'b0;
            mk_pde[55:8]=nlb; mk_pde[4:0]=nls; end
   endfunction
   function [63:0] mk_pte;  // leaf: V=1 L=1 RPN R C X W R PRIV
      input [55:12] rpn; input rref; input c; input x; input w; input r; input priv;
      begin mk_pte = 64'b0; mk_pte[63]=1'b1; mk_pte[62]=1'b1; mk_pte[55:12]=rpn;
            mk_pte[8]=rref; mk_pte[7]=c; mk_pte[3]=priv;
            mk_pte[2]=r; mk_pte[1]=w; mk_pte[0]=x; end
   endfunction
   function [63:0] mk_prte; // PRTE0: RTS split + base + RPDS
      input [55:12] base; input [5:0] rts; input [4:0] rpds;
      begin mk_prte = 64'b0; mk_prte[62:61]=rts[4:3]; mk_prte[7:5]=rts[2:0];
            mk_prte[55:12]=base; mk_prte[4:0]=rpds; end
   endfunction

   // ---------------- DUT wiring ----------------
   reg         rxe = 1'b1;
   reg         lrat_hit = 1'b1;
   reg         req_valid = 0;
   reg  [0:`TLB_TAG_WIDTH-1] req_tag;
   reg  [`TLB_WORD_WIDTH:`TLB_WAY_WIDTH-1] req_way = 0;
   reg  [0:`MM_THREADS-1] ex5_flush = 0;
   reg         inv_inprog = 0;
   reg  [0:63] ptcr_r;
   wire        lsu_req_valid;
   wire [0:`THDID_WIDTH-1] lsu_thdid;
   wire [0:1]  lsu_ttype;
   wire [64-`REAL_ADDR_WIDTH:63] lsu_addr;
   reg         lsu_taken = 0;
   wire        pte_valid;
   wire [0:`TLB_TAG_WIDTH-1] pte_tag;
   wire [0:`PTE_WIDTH-1] pte_out;
   reg         pte_taken = 0;
   reg  [0:4]  reld_tag = 0;
   reg  [0:127] reld_data = 0;
   reg         reld_vld = 0;
   reg  [58:59] reld_qw = 0;
   reg         reld_crit = 0;
   wire [0:`MM_THREADS-1] f_ptfault, f_badtree, f_segerr, f_perm, f_rc, f_lrat, f_mchk;
   wire [0:`THDID_WIDTH-1] quiesce;

   mmq_rtw dut(
      .vdd(vdd_w), .gnd(gnd_w), .nclk(nclk_w),
      .tc_ccflush_dc(1'b0), .tc_scan_dis_dc_b(1'b1), .tc_scan_diag_dc(1'b0),
      .tc_lbist_en_dc(1'b0), .lcb_d_mode_dc(1'b0), .lcb_clkoff_dc_b(1'b1),
      .lcb_act_dis_dc(1'b0), .lcb_mpw1_dc_b(5'b11111), .lcb_mpw2_dc_b(1'b1),
      .lcb_delay_lclkr_dc(5'b0), .pc_sg_2(1'b0), .pc_func_sl_thold_2(1'b0),
      .pc_func_slp_sl_thold_2(1'b0), .ac_func_scan_in(2'b0), .ac_func_scan_out(),
      .xu_mm_ccr2_notlb_b(1'b1), .mmucr2_act_override(1'b1), .tlb_delayed_act(5'b11111),
      .mmucr1_rxe(rxe),
      .tlb_rtw_req_valid(req_valid), .tlb_rtw_req_tag(req_tag), .tlb_rtw_req_way(req_way),
      .tlb_ctl_tag2_flush({`MM_THREADS{1'b0}}), .tlb_ctl_tag3_flush({`MM_THREADS{1'b0}}),
      .tlb_ctl_tag4_flush({`MM_THREADS{1'b0}}), .tlb_tag2({`TLB_TAG_WIDTH{1'b0}}),
      .tlb_tag5_except({`MM_THREADS{1'b0}}), .xu_ex5_flush(ex5_flush),
      .inv_seq_inprogress(inv_inprog), .inv_lpid(8'b0), .inv_pid({`PID_WIDTH{1'b0}}),
      .inv_gs(1'b0), .inv_as(1'b0), .inv_all(1'b1),
      .ptcr(ptcr_r), .ptcr_wr(1'b0), .pid_wr(1'b0),
      .rtw_lsu_req_valid(lsu_req_valid), .rtw_lsu_thdid(lsu_thdid), .rtw_lsu_ttype(lsu_ttype),
      .rtw_lsu_wimge(), .rtw_lsu_u(), .rtw_lsu_addr(lsu_addr), .rtw_lsu_req_taken(lsu_taken),
      .rtw_lrat_req_valid(), .rtw_lrat_addr(), .rtw_lrat_lpid(), .rtw_lrat_hit(lrat_hit),
      .rtw_quiesce(quiesce),
      .ptereload_req_valid(pte_valid), .ptereload_req_tag(pte_tag),
      .ptereload_req_pte(pte_out), .ptereload_req_taken(pte_taken),
      .an_ac_reld_core_tag(reld_tag), .an_ac_reld_data(reld_data),
      .an_ac_reld_data_vld(reld_vld), .an_ac_reld_ecc_err(1'b0),
      .an_ac_reld_ecc_err_ue(1'b0), .an_ac_reld_qw(reld_qw),
      .an_ac_reld_ditc(1'b0), .an_ac_reld_crit_qw(reld_crit),
      .rtw_pt_fault(f_ptfault), .rtw_badtree(f_badtree), .rtw_segerror(f_segerr),
      .rtw_perm_err(f_perm), .rtw_rc_err(f_rc), .rtw_lrat_miss(f_lrat), .rtw_mchk(f_mchk),
      .rtw_dbg_seq_idle(), .rtw_dbg_ctx0_seq_q(), .rtw_dbg_ctx1_seq_q(),
      .rtw_dbg_ctx_valid_q(), .rtw_dbg_ctx_killed_q(), .rtw_dbg_ctx0_shift_q(),
      .rtw_dbg_ctx1_shift_q(), .rtw_dbg_ptb_valid_q(), .rtw_dbg_pt0_valid_q(),
      .rtw_dbg_pt3_valid_q()
   );

   // ---------------- behavioural L2: grant, then return the line ----------------
   integer nloads = 0;
   reg [41:0] last_addr;
   reg [2:0]  dly = 0;
   reg [41:0] pend_addr;
   reg [0:1]  pend_ttype;
   reg        pend = 0;
   reg        l2_stall = 0;

   // Single always block: driving reld_vld from two blocks on the same edge is a
   // race and Icarus resolves it inconsistently (that is what made load 2 vanish).
   always @(posedge clk) begin
      lsu_taken <= 1'b0;
      reld_vld  <= 1'b0;
      reld_crit <= 1'b0;
      if (lsu_req_valid && !lsu_taken && !pend && !l2_stall) begin
         lsu_taken  <= 1'b1;
         last_addr  <= lsu_addr;
         pend_addr  <= lsu_addr;
         pend_ttype <= lsu_ttype;
         pend       <= 1'b1;
         dly        <= 0;
         nloads     <= nloads + 1;
         $display("    [L2] load %0d addr=%h data=%h", nloads, lsu_addr, rd(lsu_addr));
      end
      else if (pend) begin
         dly <= dly + 1;
         if (dly == 2) begin
            reld_vld  <= 1'b1;
            reld_crit <= 1'b1;
            reld_tag  <= (pend_ttype == 2'b11) ? 5'b01101 : 5'b01100;
            reld_qw   <= pend_addr[5:4];
            if (pend_addr[3] == 1'b0) reld_data <= {rd(pend_addr), 64'b0};
            else                      reld_data <= {64'b0, rd(pend_addr)};
            pend <= 1'b0;
         end
      end
   end

   // ---------------- helpers ----------------
   task build_tag; input [0:51] epn; input gs; input pr;
      integer b;
      begin
         req_tag = {`TLB_TAG_WIDTH{1'b0}};
         for (b=0;b<52;b=b+1) req_tag[`tagpos_epn+b] = epn[b];
         req_tag[`tagpos_thdid+0] = 1'b1;              // thread 0 -> context 0
         req_tag[`tagpos_type_derat] = 1'b1;
         req_tag[`tagpos_nonspec] = 1'b1;
         req_tag[`tagpos_gs] = gs;
         req_tag[`tagpos_pr] = pr;
      end
   endtask

   task run_walk; input [0:51] epn; input [255:0] name;
      integer timeout;
      begin
         build_tag(epn, 1'b0, 1'b0);
         @(negedge clk); req_valid = 1'b1;
         @(negedge clk); req_valid = 1'b0;
         timeout = 0; pte_taken = 1'b0;
         while (!pte_valid && timeout < 2000) begin @(negedge clk); timeout = timeout + 1; end
         if (!pte_valid) begin
            $display("  FAIL %0s: no ptereload within 2000 cycles (EMQ LEAK)", name);
            errors = errors + 1;
         end
         checks = checks + 1;
      end
   endtask
   task finish_walk; begin @(negedge clk); pte_taken = 1'b1; @(negedge clk); pte_taken = 1'b0; end endtask

   // ---------------- the tests ----------------
   reg [0:51] ea_epn;
   integer i;
   initial begin
      for (i = 0; i < (1<<20); i = i + 1) mem[i] = 64'b0;

      // RTS=17 -> 48-bit space; shift after the segment check is 17+19-9 = 27, so
      // the levels sit at shift 27/18/9/0 -- a genuine 4-level walk to a 4K leaf.
      // Index bits: L1=EA[47:39] L2=EA[38:30] L3=EA[29:21] L4=EA[20:12].
      // (An RTS of 8 would give only a 39-bit space and a 3-level tree.)
      // PTCR[55:12] = partition table base.  A2O reads ptcr[22:51] == RA[41:12].
      ptcr_r = {22'b0, PTCR_BASE[41:12], 12'b0};
      // PATE1: bits 55:12 = process table base, bits 4:0 = PRTS
      mem[(PTCR_BASE + 8) >> 3]  = 64'b0;
      mem[(PTCR_BASE + 8) >> 3][55:12] = PROC_BASE[55:12];
      mem[(PTCR_BASE + 8) >> 3][4:0]   = 5'd9;      // PRTS
      // PRTE0 for PID 0 (quadrant 0, effpid=0 -> entry 0): RTS=8, base=L1, RPDS=9
      mem[PROC_BASE >> 3] = mk_prte(L1_BASE[55:12], 6'd17, 5'd9);
      // directory levels
      mem[(L1_BASE + 8*1) >> 3] = mk_pde(L2_BASE[55:8], 5'd9);
      mem[(L2_BASE + 8*2) >> 3] = mk_pde(L3_BASE[55:8], 5'd9);
      mem[(L3_BASE + 8*3) >> 3] = mk_pde(L4_BASE[55:8], 5'd9);
      // leaf, 4K, RWX, R=1 C=1, user-accessible
      mem[(L4_BASE + 8*4) >> 3] = mk_pte(TARGET_RA[55:12], 1'b1,1'b1, 1'b1,1'b1,1'b1, 1'b0);

      // EA = l1<<39 | l2<<30 | l3<<21 | l4<<12, quadrant 0 (EA63:62 = 00)
      // epn[0]=EA63 ... epn[51]=EA12, so EA bit b -> epn[63-b]
      ea_epn = 52'b0;
      ea_epn[63-39] = 1'b1;                       // L1 index = 1  (EA[47:39])
      ea_epn[63-31] = 1'b1;                       // L2 index = 2  (EA[38:30])
      ea_epn[63-22] = 1'b1; ea_epn[63-21] = 1'b1; // L3 index = 3  (EA[29:21])
      ea_epn[63-14] = 1'b1;                       // L4 index = 4  (EA[20:12])

      $display("=== mmq_rtw end-to-end walk ===");
      sreset = 1'b1; repeat (4) @(negedge clk);
      sreset = 1'b0; repeat (2) @(negedge clk);

      // ---- test 1: full 4-level walk, cold caches ----
      nloads = 0;
      run_walk(ea_epn, "4-level cold walk");
      if (pte_valid) begin
         $display("  loads issued = %0d (expect 6: PATE1 + PRTE0 + 4 levels)", nloads);
         if (nloads != 6) begin $display("  FAIL: wrong load count"); errors=errors+1; end
         if (pte_out[`ptepos_valid] !== 1'b1) begin
            $display("  FAIL: install V=0, expected a successful leaf"); errors=errors+1;
         end
         if (pte_out[`ptepos_size +: 4] !== 4'b0001) begin
            $display("  FAIL: size=%b expected 4KB 0001", pte_out[`ptepos_size +: 4]); errors=errors+1;
         end
         // RA[41:12] lives at ptepos_rpn+10 .. +39
         if (pte_out[(`ptepos_rpn+10) +: 30] !== TARGET_RA[41:12]) begin
            $display("  FAIL: RPN=%h expected %h", pte_out[(`ptepos_rpn+10) +: 30], TARGET_RA[41:12]);
            errors=errors+1;
         end else $display("  RPN correct: %h", pte_out[(`ptepos_rpn+10) +: 30]);
         // usxwr = UX SX UW SW UR SR, all set for a user RWX page
         if (pte_out[`ptepos_usxwr +: 6] !== 6'b111111) begin
            $display("  FAIL: usxwr=%b expected 111111", pte_out[`ptepos_usxwr +: 6]); errors=errors+1;
         end else $display("  usxwr correct: %b", pte_out[`ptepos_usxwr +: 6]);
      end
      finish_walk; #50;

      // ---- test 2: warm caches -- roots are cached, only 4 loads now ----
      nloads = 0;
      run_walk(ea_epn, "4-level warm walk");
      if (pte_valid) begin
         $display("  loads issued = %0d (expect 4: roots cached)", nloads);
         if (nloads != 4) begin $display("  FAIL: root caching not effective"); errors=errors+1; end
      end
      finish_walk; #50;

      // ---- test 3: V=0 leaf -> page fault, still returns a reload ----
      mem[(L4_BASE + 8*4) >> 3] = 64'b0;          // V=0
      run_walk(ea_epn, "V=0 page fault");
      if (pte_valid) begin
         if (pte_out[`ptepos_valid] !== 1'b0) begin
            $display("  FAIL: fault installed with V=1"); errors=errors+1; end
         if (f_ptfault[0] !== 1'b1) begin
            $display("  FAIL: pt_fault not raised (got %b)", f_ptfault); errors=errors+1;
         end else $display("  pt_fault raised correctly, V=0 install");
      end
      finish_walk; #50;

      // ---- test 4: R=0 -> rc_error (P0-2: checked, never written back) ----
      mem[(L4_BASE + 8*4) >> 3] = mk_pte(TARGET_RA[55:12], 1'b0,1'b1, 1'b1,1'b1,1'b1, 1'b0);
      run_walk(ea_epn, "R=0 rc_error");
      if (pte_valid) begin
         if (f_rc[0] !== 1'b1) begin
            $display("  FAIL: rc_err not raised (got %b)", f_rc); errors=errors+1;
         end else $display("  rc_err raised correctly (R=0, hardware never sets it)");
      end
      finish_walk; #50;

      // ---- test 5: bad NLS -> badtree ----
      mem[(L4_BASE + 8*4) >> 3] = mk_pte(TARGET_RA[55:12], 1'b1,1'b1, 1'b1,1'b1,1'b1, 1'b0);
      mem[(L3_BASE + 8*3) >> 3] = mk_pde(L4_BASE[55:8], 5'd2);   // NLS=2, below the legal 5
      run_walk(ea_epn, "NLS=2 badtree");
      if (pte_valid) begin
         if (f_badtree[0] !== 1'b1) begin
            $display("  FAIL: badtree not raised (got %b)", f_badtree); errors=errors+1;
         end else $display("  badtree raised correctly");
      end
      finish_walk; #50;
      mem[(L3_BASE + 8*3) >> 3] = mk_pde(L4_BASE[55:8], 5'd9);

      // ---- test 6: quadrant 1 (EA63=0,EA62=1) -> segerror ----
      begin : t6
         reg [0:51] q1; q1 = ea_epn; q1[1] = 1'b1;      // epn[1] = EA62
         run_walk(q1, "quadrant 1 segerror");
         if (pte_valid) begin
            if (f_segerr[0] !== 1'b1) begin
               $display("  FAIL: segerror not raised (got %b)", f_segerr); errors=errors+1;
            end else $display("  segerror raised correctly for quadrant 1");
         end
         finish_walk; #50;
      end

      // ---- test 7 (P0-1/P0-3): flush mid-walk still returns a reload ----
      begin : t7
         integer to;
         build_tag(ea_epn, 1'b0, 1'b0);
         @(negedge clk); req_valid = 1'b1;
         @(negedge clk); req_valid = 1'b0;
         repeat (12) @(negedge clk);          // let the walk get going
         $display("    [dbg] before flush: seq0=%b valid0=%b killed0=%b resv0=%b pte_valid=%b",
                  dut.rtw_dbg_ctx0_seq_q, dut.ctx_valid_q[0], dut.ctx_killed_q[0],
                  dut.ctx_resv_q[0], pte_valid);
         ex5_flush = {`MM_THREADS{1'b1}};     // completion-unit flush
         @(negedge clk);
         $display("    [dbg] at flush: kill_now0=%b killed0=%b", dut.ctx_kill_now[0], dut.ctx_killed_q[0]);
         ex5_flush = 0;
         @(negedge clk);
         $display("    [dbg] after flush: seq0=%b killed0=%b", dut.rtw_dbg_ctx0_seq_q, dut.ctx_killed_q[0]);
         to = 0;
         while (!pte_valid && to < 2000) begin @(negedge clk); to = to + 1; end
         if (!pte_valid) begin
            $display("  FAIL flush mid-walk: no ptereload -> EMQ ENTRY LEAKED"); errors=errors+1;
         end else if (pte_out[`ptepos_valid] !== 1'b0) begin
            $display("  FAIL flush mid-walk: installed V=1 after flush"); errors=errors+1;
         end else
            $display("  flush mid-walk: reload returned with V=0, EMQ freed (P0-1/P0-3 OK)");
         checks = checks + 1;
         finish_walk; #50;
      end

      // ---- test 8 (P0-4): invalidate mid-walk clears the reservation ----
      begin : t8
         integer to;
         build_tag(ea_epn, 1'b0, 1'b0);
         @(negedge clk); req_valid = 1'b1;
         @(negedge clk); req_valid = 1'b0;
         repeat (12) @(negedge clk);
         inv_inprog = 1'b1;                   // tlbivax targeting anything
         @(negedge clk); inv_inprog = 1'b0;
         to = 0;
         while (!pte_valid && to < 2000) begin @(negedge clk); to = to + 1; end
         if (!pte_valid) begin
            $display("  FAIL invalidate mid-walk: no ptereload -> EMQ LEAK"); errors=errors+1;
         end else if (pte_out[`ptepos_valid] !== 1'b0) begin
            $display("  FAIL invalidate mid-walk: installed a stale translation"); errors=errors+1;
         end else
            $display("  invalidate mid-walk: walk discarded, reload returned (P0-4 OK)");
         checks = checks + 1;
         finish_walk; #50;
      end


      // ---- test 9: radix disabled -- the walker must be completely inert ----
      begin : t9
         integer to;
         rxe = 1'b0;
         nloads = 0;
         build_tag(ea_epn, 1'b0, 1'b0);
         @(negedge clk); req_valid = 1'b1;
         @(negedge clk); req_valid = 1'b0;
         to = 0;
         while (!pte_valid && to < 200) begin @(negedge clk); to = to + 1; end
         if (pte_valid || nloads != 0) begin
            $display("  FAIL radix disabled: walker active (pte_valid=%b loads=%0d)", pte_valid, nloads);
            errors = errors + 1;
         end else
            $display("  radix disabled: no loads, no reload -- Book-E path unaffected");
         checks = checks + 1;
         rxe = 1'b1;
         #50;
      end

      // ---- test 10 (P2-11): guest-mode walk refused when the LRAT says no ----
      begin : t10
         integer to;
         lrat_hit = 1'b0;
         build_tag(ea_epn, 1'b1, 1'b0);        // gs = 1
         @(negedge clk); req_valid = 1'b1;
         @(negedge clk); req_valid = 1'b0;
         to = 0;
         while (!pte_valid && to < 2000) begin @(negedge clk); to = to + 1; end
         if (!pte_valid) begin
            $display("  FAIL guest-mode refusal: no ptereload -> EMQ LEAK"); errors = errors + 1;
         end else if (f_lrat[0] !== 1'b1) begin
            $display("  FAIL guest-mode refusal: lrat_miss not raised (got %b)", f_lrat);
            errors = errors + 1;
         end else if (pte_out[`ptepos_valid] !== 1'b0) begin
            $display("  FAIL guest-mode refusal: installed an unvalidated translation");
            errors = errors + 1;
         end else
            $display("  guest-mode walk refused with lrat_miss, nothing installed (P2-11 OK)");
         checks = checks + 1;
         finish_walk;
         lrat_hit = 1'b1;
         #50;
      end

      // ---- test 11 (P1-7): watchdog -- L2 never answers ----
      begin : t11
         integer to;
         build_tag(ea_epn, 1'b0, 1'b0);
         @(negedge clk); req_valid = 1'b1;
         @(negedge clk); req_valid = 1'b0;
         l2_stall = 1'b1;                      // swallow the request, never reply
         to = 0;
         while (!pte_valid && to < 20000) begin @(negedge clk); to = to + 1; end
         if (!pte_valid) begin
            $display("  FAIL watchdog: still hung after %0d cycles -> thread would never quiesce", to);
            errors = errors + 1;
         end else if (f_mchk[0] !== 1'b1) begin
            $display("  FAIL watchdog: fired but no machine check (got %b)", f_mchk);
            errors = errors + 1;
         end else
            $display("  watchdog fired after %0d cycles, mchk raised, EMQ freed (P1-7 OK)", to);
         checks = checks + 1;
         finish_walk;
         l2_stall = 1'b0;
         #50;
      end

      $display("");
      if (errors == 0) $display("PASS: %0d walk scenarios, no failures", checks);
      else             $display("FAIL: %0d errors across %0d scenarios", errors, checks);
      $finish;
   end
endmodule
