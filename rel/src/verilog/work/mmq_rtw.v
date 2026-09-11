// © IBM Corp. 2020
// Licensed under the Apache License, Version 2.0 (the "License"), as modified by
// the terms below; you may not use the files in this repository except in
// compliance with the License as modified.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Modified Terms:
//
//    1) For the purpose of the patent license granted to you in Section 3 of the
//    License, the "Work" hereby includes implementations of the work of authorship
//    in physical form.
//
//    2) Notwithstanding any terms to the contrary in the License, any licenses
//    necessary for implementation of the Work that are available from OpenPOWER
//    via the Power ISA End User License Agreement (EULA) are explicitly excluded
//    hereunder, and may be obtained from OpenPOWER under the terms and conditions
//    of the EULA.
//
// Unless required by applicable law or agreed to in writing, the reference design
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
// for the specific language governing permissions and limitations under the License.
//
// Additional rights, including the ability to physically implement a softcore that
// is compliant with the required sections of the Power ISA Specification, are
// available at no cost under the terms of the OpenPOWER Power ISA EULA, which can be
// obtained (along with the Power ISA) here: https://openpowerfoundation.org.

//********************************************************************
//* TITLE: Memory Management Unit Radix Tree Hardware Table Walker
//*
//* Multi-level (Power ISA 3.1C) radix page-table walker, ported from
//* Microwatt's mmu.vhdl into A2O's out-of-order MMU environment.
//*
//* Sits beside mmq_htw (the Book-E E.PT single-level walker), which is
//* left untouched.  Selected by MMUCR1[RXE]; mmq.v muxes the LSU request
//* and ptereload interfaces between the two.
//*
//* See PLAN.md section 4 (design) and section 5 (out-of-order constraints).
//* The five P0 constraints from PLAN.md section 5 are implemented here and
//* marked with "P0-n" comments:
//*   P0-1  per-context killed bit, tested at every level boundary
//*   P0-2  R/C are checked, never written back
//*   P0-3  ptereload/derat_rel returned on EVERY termination path
//*   P0-4  reservation cleared on any matching invalidate, EPN-independent
//*   P0-5  one load in flight per context, re-arbitrated per level
//*********************************************************************

`timescale 1 ns / 1 ns

`include "tri_a2o.vh"
`include "mmu_a2o.vh"

`define        RTW_SEQ_WIDTH     4
`define        RTW_NUM_CTX       2
`define        RTW_WD_WIDTH      12
`define        RTW_RETRY_WIDTH   2

module mmq_rtw(
   inout                                  vdd,
   inout                                  gnd,
   (* pin_data ="PIN_FUNCTION=/G_CLK/" *)
   input [0:`NCLK_WIDTH-1]                nclk,

   input                                  tc_ccflush_dc,
   input                                  tc_scan_dis_dc_b,
   input                                  tc_scan_diag_dc,
   input                                  tc_lbist_en_dc,
   input                                  lcb_d_mode_dc,
   input                                  lcb_clkoff_dc_b,
   input                                  lcb_act_dis_dc,
   input [0:4]                            lcb_mpw1_dc_b,
   input                                  lcb_mpw2_dc_b,
   input [0:4]                            lcb_delay_lclkr_dc,
   input                                  pc_sg_2,
   input                                  pc_func_sl_thold_2,
   input                                  pc_func_slp_sl_thold_2,
(* pin_data="PIN_FUNCTION=/SCAN_IN/" *)
   input [0:1]                            ac_func_scan_in,
(* pin_data="PIN_FUNCTION=/SCAN_OUT/" *)
   output [0:1]                           ac_func_scan_out,

   input                                  xu_mm_ccr2_notlb_b,
   input                                  mmucr2_act_override,
   input [24:28]                          tlb_delayed_act,

   // radix enable, MMUCR1[RXE].  When 0 this module is inert.
   input                                  mmucr1_rxe,

   // ---- request handoff from mmq_tlb_cmp (mirrors tlb_htw_req_*) ----
   input                                  tlb_rtw_req_valid,
   input [0:`TLB_TAG_WIDTH-1]              tlb_rtw_req_tag,
   input [`TLB_WORD_WIDTH:`TLB_WAY_WIDTH-1] tlb_rtw_req_way,

   // ---- invalidate / flush / exception context ----
   input [0:`MM_THREADS-1]                tlb_ctl_tag2_flush,
   input [0:`MM_THREADS-1]                tlb_ctl_tag3_flush,
   input [0:`MM_THREADS-1]                tlb_ctl_tag4_flush,
   // NOTE: tlb_ctl_tag{2,3,4}_flush, tlb_tag2 and tlb_tag5_except are carried for
   //       interface parity with mmq_htw so mmq.v can mux the two walkers with one
   //       set of wires. The radix walker deliberately does NOT use the tag-pipe
   //       flush signals -- they are hard-wired to zero for erat/ptereload types
   //       (mmq_tlb_ctl.v:2332-2342), which is exactly the P0-1 hazard. It uses
   //       xu_ex5_flush below instead.
   input [0:`TLB_TAG_WIDTH-1]             tlb_tag2,
   input [0:`MM_THREADS-1]                tlb_tag5_except,
   // P0-1: completion-unit flush.  The tag-pipe flush signals above are
   //       hard-wired to zero for erat/ptereload types (mmq_tlb_ctl.v:2332-2342)
   //       so they can never abort a walk.  This is the real kill signal.
   input [0:`MM_THREADS-1]                xu_ex5_flush,
   // P0-4: any invalidate in progress that matches this context's
   //       (lpid,pid,gs,as) -- EPN deliberately NOT compared, because a walk
   //       touches 4-5 addresses and we cannot know which level was hit.
   input                                  inv_seq_inprogress,
   input [0:`LPID_WIDTH-1]                inv_lpid,
   input [0:`PID_WIDTH-1]                 inv_pid,
   input                                  inv_gs,
   input                                  inv_as,
   input                                  inv_all,

   // ---- radix root pointers (PLAN.md 4.5 item 5) ----
   input [0:63]                           ptcr,
   input                                  ptcr_wr,          // mtspr PTCR: drop all cached roots
   input                                  pid_wr,           // mtspr PID:  drop cached quadrant-0 root

   // ---- LSU request port (mirrors htw_lsu_*) ----
   output reg                             rtw_lsu_req_valid,
   output [0:`THDID_WIDTH-1]              rtw_lsu_thdid,
   output [0:1]                           rtw_lsu_ttype,
   output [0:4]                           rtw_lsu_wimge,
   output [0:3]                           rtw_lsu_u,
   output [64-`REAL_ADDR_WIDTH:63]        rtw_lsu_addr,
   input                                  rtw_lsu_req_taken,

   // ---- P2-11: per-level LRAT check, guest mode only ----
   output                                 rtw_lrat_req_valid,
   output [64-`REAL_ADDR_WIDTH:63]        rtw_lrat_addr,
   output [0:`LPID_WIDTH-1]               rtw_lrat_lpid,
   input                                  rtw_lrat_hit,

   // ---- quiesce ----
   output [0:`THDID_WIDTH-1]              rtw_quiesce,

   // ---- ptereload back into mmq_tlb_ctl ----
   output                                 ptereload_req_valid,
   output [0:`TLB_TAG_WIDTH-1]            ptereload_req_tag,
   output [0:`PTE_WIDTH-1]                ptereload_req_pte,
   input                                  ptereload_req_taken,

   // ---- L2 reload bus ----
   input [0:4]                            an_ac_reld_core_tag,
   input [0:127]                          an_ac_reld_data,
   input                                  an_ac_reld_data_vld,
   input                                  an_ac_reld_ecc_err,
   input                                  an_ac_reld_ecc_err_ue,
   input [58:59]                          an_ac_reld_qw,
   input                                  an_ac_reld_ditc,
   input                                  an_ac_reld_crit_qw,

   // ---- fault / error reporting ----
   output [0:`MM_THREADS-1]               rtw_pt_fault,     // PDE/PTE V=0
   output [0:`MM_THREADS-1]               rtw_badtree,      // malformed radix tree
   output [0:`MM_THREADS-1]               rtw_segerror,     // quadrant / EA range
   output [0:`MM_THREADS-1]               rtw_perm_err,
   output [0:`MM_THREADS-1]               rtw_rc_err,
   output [0:`MM_THREADS-1]               rtw_lrat_miss,
   output [0:`MM_THREADS-1]               rtw_mchk,         // watchdog / UE escalation / RA overflow

   // ---- debug ----
   output                                 rtw_dbg_seq_idle,
   output [0:`RTW_SEQ_WIDTH-1]            rtw_dbg_ctx0_seq_q,
   output [0:`RTW_SEQ_WIDTH-1]            rtw_dbg_ctx1_seq_q,
   output [0:1]                           rtw_dbg_ctx_valid_q,
   output [0:1]                           rtw_dbg_ctx_killed_q,
   output [0:5]                           rtw_dbg_ctx0_shift_q,
   output [0:5]                           rtw_dbg_ctx1_shift_q,
   output                                 rtw_dbg_ptb_valid_q,
   output                                 rtw_dbg_pt0_valid_q,
   output                                 rtw_dbg_pt3_valid_q
);

      //---------------------------------------------------------------------
      // Parameters
      //---------------------------------------------------------------------
      parameter [0:4]                        Core_Tag0_Value = 5'b01100;
      parameter [0:4]                        Core_Tag1_Value = 5'b01101;

      // A2O TLB page-size codes.  Note the encoding is log4(size/1KB), so it can
      // express only power-of-4 sizes.  2MB is NOT representable.  Furthermore
      // mmq_tlb_cmp.v:3486 builds the way size field as {1'b0, pte[ptepos_size+0:+2]},
      // so only codes 0000-0111 survive the ptereload path -- 1GB is unreachable too.
      // Radix leaf sizes are therefore DEMOTED to the largest representable size
      // that is a sub-page of the real leaf.  Demotion is always architecturally
      // safe: a smaller page maps a subset of the same translation with the same
      // permissions, it just costs extra TLB misses.  See Rtw_Demote below.
      parameter [0:3]                        TLB_PgSize_16MB = 4'b0111;
      parameter [0:3]                        TLB_PgSize_1MB  = 4'b0101;
      parameter [0:3]                        TLB_PgSize_64KB = 4'b0011;
      parameter [0:3]                        TLB_PgSize_4KB  = 4'b0001;

      // Radix shift values (log2(pagesize) - 12) for the sizes radix produces
      parameter [0:5]                        RadixShift_4KB  = 6'd0;
      parameter [0:5]                        RadixShift_64KB = 6'd4;
      parameter [0:5]                        RadixShift_2MB  = 6'd9;
      parameter [0:5]                        RadixShift_1GB  = 6'd18;
      // ...and the shift of the size we actually install for each
      parameter [0:5]                        InstShift_1MB   = 6'd8;
      parameter [0:5]                        InstShift_16MB  = 6'd12;

      // Sequencer states.  Idle is all-zeros so that an AND-mask kill returns to
      // Idle, matching the tlb_seq_abort idiom at mmq_tlb_ctl.v:1376-1378.
      // Single-bit transitions along the nominal path, as in mmq_tlb_ctl.v:343-375.
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_Idle     = 4'b0000;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_PartRd   = 4'b0001;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_PartWait = 4'b0011;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_ProcRd   = 4'b0010;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_ProcWait = 4'b0110;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_SegChk   = 4'b0111;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_Lookup   = 4'b0101;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_ReadWait = 4'b0100;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_Reload   = 4'b1100;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_Fault    = 4'b1101;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_Killed   = 4'b1111;
      parameter [0:`RTW_SEQ_WIDTH-1]         RtwSeq_Timeout  = 4'b1110;

      // Fault codes carried to the Fault terminal state
      parameter [0:2]                        Flt_None     = 3'b000;
      parameter [0:2]                        Flt_Invalid  = 3'b001;   // V=0
      parameter [0:2]                        Flt_BadTree  = 3'b010;
      parameter [0:2]                        Flt_SegError = 3'b011;
      parameter [0:2]                        Flt_PermErr  = 3'b100;
      parameter [0:2]                        Flt_RcErr    = 3'b101;
      parameter [0:2]                        Flt_LratMiss = 3'b110;
      parameter [0:2]                        Flt_Mchk     = 3'b111;

      // 0=tlbivax_op, 1=tlbi_complete, 2=mmu read core_tag=01100, 3=core_tag=01101
      parameter [0:1]                        LsuTtype_Rd0 = 2'b10;
      parameter [0:1]                        LsuTtype_Rd1 = 2'b11;

      // Which table a context is currently fetching (selects the address mux)
      parameter [0:1]                        Fetch_Part = 2'b00;
      parameter [0:1]                        Fetch_Proc = 2'b01;
      parameter [0:1]                        Fetch_Pde  = 2'b10;

      //---------------------------------------------------------------------
      // Scan chain offsets
      //---------------------------------------------------------------------
      // chain 0: per-context walk state
      // per-context field offsets within one CTX_STRIDE slice
      parameter                              cf_valid    = 0;
      parameter                              cf_killed   = cf_valid    + 1;
      parameter                              cf_pending  = cf_killed   + 1;
      parameter                              cf_resv     = cf_pending  + 1;
      parameter                              cf_dataval  = cf_resv     + 1;
      parameter                              cf_seq      = cf_dataval  + 1;
      parameter                              cf_shift    = cf_seq      + `RTW_SEQ_WIDTH;
      parameter                              cf_masksize = cf_shift    + 6;
      parameter                              cf_fault    = cf_masksize + 5;
      parameter                              cf_fetch    = cf_fault    + 3;
      parameter                              cf_cloff    = cf_fetch    + 2;
      parameter                              cf_qwbeat   = cf_cloff    + 3;
      parameter                              cf_err      = cf_qwbeat   + 4;
      parameter                              cf_retry    = cf_err      + 3;
      parameter                              cf_wd       = cf_retry    + `RTW_RETRY_WIDTH;
      parameter                              cf_pgbase   = cf_wd       + `RTW_WD_WIDTH;
      parameter                              cf_data     = cf_pgbase   + `REAL_ADDR_WIDTH;
      parameter                              cf_tag      = cf_data     + 64;
      parameter                              cf_way      = cf_tag      + `TLB_TAG_WIDTH;
      parameter                              CTX_STRIDE  = cf_way      + (`TLB_WAY_WIDTH-`TLB_WORD_WIDTH);

      parameter                              ctx_base_offset = 0;
      parameter                              spare_a_offset  = ctx_base_offset + (`RTW_NUM_CTX * CTX_STRIDE);
      parameter                              scan_right_0    = spare_a_offset + 16 - 1;

      // chain 1: shared root cache, lsu staging, reload pipe
      parameter                              prtbl_offset            = 0;
      parameter                              pgtbl0_offset           = prtbl_offset + 64;
      parameter                              pgtbl3_offset           = pgtbl0_offset + 64;
      parameter                              ptb_valid_offset        = pgtbl3_offset + 64;
      parameter                              pt0_valid_offset        = ptb_valid_offset + 1;
      parameter                              pt3_valid_offset        = pt0_valid_offset + 1;
      parameter                              rtw_lsu_ttype_offset    = pt3_valid_offset + 1;
      parameter                              rtw_lsu_thdid_offset    = rtw_lsu_ttype_offset + 2;
      parameter                              rtw_lsu_wimge_offset    = rtw_lsu_thdid_offset + `THDID_WIDTH;
      parameter                              rtw_lsu_u_offset        = rtw_lsu_wimge_offset + 5;
      parameter                              rtw_lsu_addr_offset     = rtw_lsu_u_offset + 4;
      parameter                              rtw_arb_ptr_offset      = rtw_lsu_addr_offset + `REAL_ADDR_WIDTH;
      parameter                              rtw_arb_armed_offset    = rtw_arb_ptr_offset + 1;
      parameter                              reload_ptr_offset       = rtw_arb_armed_offset + 1;
      parameter                              reld_core_tag_tm1_offset = reload_ptr_offset + 1;
      parameter                              reld_qw_tm1_offset      = reld_core_tag_tm1_offset + 5;
      parameter                              reld_crit_qw_tm1_offset = reld_qw_tm1_offset + 2;
      parameter                              reld_ditc_tm1_offset    = reld_crit_qw_tm1_offset + 1;
      parameter                              reld_data_vld_tm1_offset = reld_ditc_tm1_offset + 1;
      parameter                              reld_core_tag_t_offset  = reld_data_vld_tm1_offset + 1;
      parameter                              reld_qw_t_offset        = reld_core_tag_t_offset + 5;
      parameter                              reld_crit_qw_t_offset   = reld_qw_t_offset + 2;
      parameter                              reld_ditc_t_offset      = reld_crit_qw_t_offset + 1;
      parameter                              reld_data_vld_t_offset  = reld_ditc_t_offset + 1;
      parameter                              reld_core_tag_tp1_offset = reld_data_vld_t_offset + 1;
      parameter                              reld_qw_tp1_offset      = reld_core_tag_tp1_offset + 5;
      parameter                              reld_crit_qw_tp1_offset = reld_qw_tp1_offset + 2;
      parameter                              reld_ditc_tp1_offset    = reld_crit_qw_tp1_offset + 1;
      parameter                              reld_data_vld_tp1_offset = reld_ditc_tp1_offset + 1;
      parameter                              reld_core_tag_tp2_offset = reld_data_vld_tp1_offset + 1;
      parameter                              reld_qw_tp2_offset      = reld_core_tag_tp2_offset + 5;
      parameter                              reld_crit_qw_tp2_offset = reld_qw_tp2_offset + 2;
      parameter                              reld_ditc_tp2_offset    = reld_crit_qw_tp2_offset + 1;
      parameter                              reld_data_vld_tp2_offset = reld_ditc_tp2_offset + 1;
      parameter                              reld_ecc_err_tp2_offset = reld_data_vld_tp2_offset + 1;
      parameter                              reld_ecc_err_ue_tp2_offset = reld_ecc_err_tp2_offset + 1;
      parameter                              reld_data_tp1_offset    = reld_ecc_err_ue_tp2_offset + 1;
      parameter                              reld_data_tp2_offset    = reld_data_tp1_offset + 128;
      parameter                              spare_b_offset          = reld_data_tp2_offset + 128;
      parameter                              scan_right_1            = spare_b_offset + 16 - 1;

      //---------------------------------------------------------------------
      // Declarations
      //---------------------------------------------------------------------
      // NOTE ON STYLE: mmq_htw.v unrolls its four request slots by hand.  This
      // module keeps its two walk contexts in arrays driven from a generate loop
      // instead.  A radix context carries ~10x the state of an E.PT slot, and
      // hand-unrolling it twice is how transcription bugs get in.  Everything
      // else -- tri_rlmreg_p scan latches, explicit sensitivity lists, full
      // default-assignment prologues, `[0:N]` MSB-first vectors -- follows the
      // house style exactly (PLAN.md section 4.3).

      wire                                   ctx_valid_d    [0:`RTW_NUM_CTX-1];
      wire                                   ctx_valid_q    [0:`RTW_NUM_CTX-1];
      wire                                   ctx_killed_d   [0:`RTW_NUM_CTX-1];
      wire                                   ctx_killed_q   [0:`RTW_NUM_CTX-1];
      wire                                   ctx_pending_d  [0:`RTW_NUM_CTX-1];
      wire                                   ctx_pending_q  [0:`RTW_NUM_CTX-1];
      wire                                   ctx_resv_d     [0:`RTW_NUM_CTX-1];
      wire                                   ctx_resv_q     [0:`RTW_NUM_CTX-1];
      wire                                   ctx_dataval_d  [0:`RTW_NUM_CTX-1];
      wire                                   ctx_dataval_q  [0:`RTW_NUM_CTX-1];
      reg  [0:`RTW_SEQ_WIDTH-1]              ctx_seq_d      [0:`RTW_NUM_CTX-1];
      wire [0:`RTW_SEQ_WIDTH-1]              ctx_seq_din    [0:`RTW_NUM_CTX-1];
      wire [0:`RTW_SEQ_WIDTH-1]              ctx_seq_q      [0:`RTW_NUM_CTX-1];
      reg  [0:5]                             ctx_shift_d    [0:`RTW_NUM_CTX-1];
      wire [0:5]                             ctx_shift_q    [0:`RTW_NUM_CTX-1];
      reg  [0:4]                             ctx_masksize_d [0:`RTW_NUM_CTX-1];
      wire [0:4]                             ctx_masksize_q [0:`RTW_NUM_CTX-1];
      reg  [0:2]                             ctx_fault_d    [0:`RTW_NUM_CTX-1];
      wire [0:2]                             ctx_fault_q    [0:`RTW_NUM_CTX-1];
      reg  [0:1]                             ctx_fetch_d    [0:`RTW_NUM_CTX-1];
      wire [0:1]                             ctx_fetch_q    [0:`RTW_NUM_CTX-1];
      reg  [64-`REAL_ADDR_WIDTH:63]          ctx_pgbase_d   [0:`RTW_NUM_CTX-1];
      wire [64-`REAL_ADDR_WIDTH:63]          ctx_pgbase_q   [0:`RTW_NUM_CTX-1];
      wire [0:`TLB_TAG_WIDTH-1]              ctx_tag_d      [0:`RTW_NUM_CTX-1];
      wire [0:`TLB_TAG_WIDTH-1]              ctx_tag_q      [0:`RTW_NUM_CTX-1];
      wire [`TLB_WORD_WIDTH:`TLB_WAY_WIDTH-1] ctx_way_d     [0:`RTW_NUM_CTX-1];
      wire [`TLB_WORD_WIDTH:`TLB_WAY_WIDTH-1] ctx_way_q     [0:`RTW_NUM_CTX-1];
      wire [0:63]                            ctx_data_d     [0:`RTW_NUM_CTX-1];
      wire [0:63]                            ctx_data_q     [0:`RTW_NUM_CTX-1];
      wire [58:60]                           ctx_cloff_d    [0:`RTW_NUM_CTX-1];
      wire [58:60]                           ctx_cloff_q    [0:`RTW_NUM_CTX-1];
      wire [0:3]                             ctx_qwbeat_d   [0:`RTW_NUM_CTX-1];
      wire [0:3]                             ctx_qwbeat_q   [0:`RTW_NUM_CTX-1];
      wire [0:2]                             ctx_err_d      [0:`RTW_NUM_CTX-1];
      wire [0:2]                             ctx_err_q      [0:`RTW_NUM_CTX-1];
      wire [0:`RTW_RETRY_WIDTH-1]            ctx_retry_d    [0:`RTW_NUM_CTX-1];
      wire [0:`RTW_RETRY_WIDTH-1]            ctx_retry_q    [0:`RTW_NUM_CTX-1];
      wire [0:`RTW_WD_WIDTH-1]               ctx_wd_d       [0:`RTW_NUM_CTX-1];
      wire [0:`RTW_WD_WIDTH-1]               ctx_wd_q       [0:`RTW_NUM_CTX-1];

      // per-context combinational
      reg                                    ctx_seq_load_req  [0:`RTW_NUM_CTX-1];
      reg                                    ctx_seq_reload    [0:`RTW_NUM_CTX-1];
      reg                                    ctx_seq_done      [0:`RTW_NUM_CTX-1];
      reg                                    ctx_seq_retry     [0:`RTW_NUM_CTX-1];
      wire                                   ctx_load_taken    [0:`RTW_NUM_CTX-1];
      wire                                   ctx_reload_taken  [0:`RTW_NUM_CTX-1];
      wire [64-`REAL_ADDR_WIDTH:63]          ctx_req_addr      [0:`RTW_NUM_CTX-1];
      wire [0:`PTE_WIDTH-1]                  ctx_pte_out       [0:`RTW_NUM_CTX-1];
      wire                                   ctx_kill_now      [0:`RTW_NUM_CTX-1];
      wire                                   ctx_inv_match     [0:`RTW_NUM_CTX-1];
      wire                                   ctx_reld_for_me_tp2 [0:`RTW_NUM_CTX-1];
      wire                                   ctx_wd_expired    [0:`RTW_NUM_CTX-1];
      wire                                   ctx_wd_run        [0:`RTW_NUM_CTX-1];
      wire [0:`THDID_WIDTH-1]                ctx_thdid         [0:`RTW_NUM_CTX-1];
      reg                                    ctx_wr_prtbl      [0:`RTW_NUM_CTX-1];
      reg                                    ctx_wr_pgtbl      [0:`RTW_NUM_CTX-1];
      wire                                   ctx_quad3         [0:`RTW_NUM_CTX-1];
      wire                                   ctx_fault_val     [0:`RTW_NUM_CTX-1];

      // shared root cache
      wire [0:63]                            prtbl_d,  prtbl_q;
      wire [0:63]                            pgtbl0_d, pgtbl0_q;
      wire [0:63]                            pgtbl3_d, pgtbl3_q;
      wire                                   ptb_valid_d, ptb_valid_q;
      wire                                   pt0_valid_d, pt0_valid_q;
      wire                                   pt3_valid_d, pt3_valid_q;

      // lsu staging
      wire [0:1]                             rtw_lsu_ttype_d, rtw_lsu_ttype_q;
      wire [0:`THDID_WIDTH-1]                rtw_lsu_thdid_d, rtw_lsu_thdid_q;
      wire [0:4]                             rtw_lsu_wimge_d, rtw_lsu_wimge_q;
      wire [0:3]                             rtw_lsu_u_d, rtw_lsu_u_q;
      wire [64-`REAL_ADDR_WIDTH:63]          rtw_lsu_addr_d, rtw_lsu_addr_q;
      wire                                   rtw_arb_ptr_d, rtw_arb_ptr_q;
      wire                                   rtw_arb_armed_d, rtw_arb_armed_q;
      wire                                   reload_ptr_d, reload_ptr_q;
      wire                                   rtw_arb_sel;
      reg                                    rtw_arb_gnt;
      wire                                   reload_sel;
      reg                                    reload_gnt;

      // reload pipe (identical staging to mmq_htw.v:1355-1400)
      wire [0:4]   reld_core_tag_tm1_d, reld_core_tag_tm1_q;
      wire [58:59] reld_qw_tm1_d, reld_qw_tm1_q;
      wire         reld_crit_qw_tm1_d, reld_crit_qw_tm1_q;
      wire         reld_ditc_tm1_d, reld_ditc_tm1_q;
      wire         reld_data_vld_tm1_d, reld_data_vld_tm1_q;
      wire [0:4]   reld_core_tag_t_d, reld_core_tag_t_q;
      wire [58:59] reld_qw_t_d, reld_qw_t_q;
      wire         reld_crit_qw_t_d, reld_crit_qw_t_q;
      wire         reld_ditc_t_d, reld_ditc_t_q;
      wire         reld_data_vld_t_d, reld_data_vld_t_q;
      wire [0:4]   reld_core_tag_tp1_d, reld_core_tag_tp1_q;
      wire [58:59] reld_qw_tp1_d, reld_qw_tp1_q;
      wire         reld_crit_qw_tp1_d, reld_crit_qw_tp1_q;
      wire         reld_ditc_tp1_d, reld_ditc_tp1_q;
      wire         reld_data_vld_tp1_d, reld_data_vld_tp1_q;
      wire [0:127] reld_data_tp1_d, reld_data_tp1_q;
      wire [0:4]   reld_core_tag_tp2_d, reld_core_tag_tp2_q;
      wire [58:59] reld_qw_tp2_d, reld_qw_tp2_q;
      wire         reld_crit_qw_tp2_d, reld_crit_qw_tp2_q;
      wire         reld_ditc_tp2_d, reld_ditc_tp2_q;
      wire         reld_data_vld_tp2_d, reld_data_vld_tp2_q;
      wire         reld_ecc_err_tp2_d, reld_ecc_err_tp2_q;
      wire         reld_ecc_err_ue_tp2_d, reld_ecc_err_ue_tp2_q;
      wire [0:127] reld_data_tp2_d, reld_data_tp2_q;

      wire [0:15]  spare_a_q;
      wire [0:15]  spare_b_q;
      wire [0:scan_right_0] siv_0, sov_0;
      wire [0:scan_right_1] siv_1, sov_1;

      wire         pc_sg_0, pc_sg_1;
      wire         pc_func_sl_thold_0, pc_func_sl_thold_1;
      wire         pc_func_sl_thold_0_b;
      wire         pc_func_slp_sl_thold_0, pc_func_slp_sl_thold_1;
      wire         pc_func_slp_sl_thold_0_b;
      wire         pc_func_sl_force, pc_func_slp_sl_force;
      wire         rtw_act, reld_act;
      wire [0:`THDID_WIDTH-1] rtw_quiesce_b;
      wire [0:31]  unused_dc;

      genvar       i;
      genvar       k;

      //---------------------------------------------------------------------
      // Shared: cached radix roots (PLAN.md 4.5 item 5)
      //---------------------------------------------------------------------
      // Microwatt caches the partition-table entry (r.prtbl) and the two quadrant
      // roots (r.pgtbl0/r.pgtbl3) with valid bits, and drops them on mtspr
      // PTCR/PID (mmu.vhdl:1544-1560).  Doing the same here matters far more in
      // A2O than it does in Microwatt: PLAN.md section 5 P0-5 shows the MMU holds
      // exactly one LSU credit token, so every table read we can skip is a whole
      // L2 round trip removed from the critical path.

      assign prtbl_d  = (ptcr_wr) ? 64'b0 :
                        (ctx_wr_prtbl[0]) ? ctx_data_q[0] :
                        (ctx_wr_prtbl[1]) ? ctx_data_q[1] : prtbl_q;
      assign pgtbl0_d = (ptcr_wr | pid_wr) ? 64'b0 :
                        (ctx_wr_pgtbl[0] & ~ctx_quad3[0]) ? ctx_data_q[0] :
                        (ctx_wr_pgtbl[1] & ~ctx_quad3[1]) ? ctx_data_q[1] : pgtbl0_q;
      assign pgtbl3_d = (ptcr_wr) ? 64'b0 :
                        (ctx_wr_pgtbl[0] &  ctx_quad3[0]) ? ctx_data_q[0] :
                        (ctx_wr_pgtbl[1] &  ctx_quad3[1]) ? ctx_data_q[1] : pgtbl3_q;

      assign ptb_valid_d = (ptcr_wr) ? 1'b0 :
                           (ctx_wr_prtbl[0] | ctx_wr_prtbl[1]) ? 1'b1 : ptb_valid_q;
      assign pt0_valid_d = (ptcr_wr | pid_wr) ? 1'b0 :
                           ((ctx_wr_pgtbl[0] & ~ctx_quad3[0]) |
                            (ctx_wr_pgtbl[1] & ~ctx_quad3[1])) ? 1'b1 : pt0_valid_q;
      assign pt3_valid_d = (ptcr_wr) ? 1'b0 :
                           ((ctx_wr_pgtbl[0] &  ctx_quad3[0]) |
                            (ctx_wr_pgtbl[1] &  ctx_quad3[1])) ? 1'b1 : pt3_valid_q;

      //---------------------------------------------------------------------
      // Shared: L2 reload staging (mmq_htw.v:1355-1400, unchanged)
      //---------------------------------------------------------------------
      assign reld_core_tag_tm1_d   = an_ac_reld_core_tag;
      assign reld_qw_tm1_d         = an_ac_reld_qw;
      assign reld_crit_qw_tm1_d    = an_ac_reld_crit_qw;
      assign reld_ditc_tm1_d       = an_ac_reld_ditc;
      assign reld_data_vld_tm1_d   = an_ac_reld_data_vld;
      assign reld_core_tag_t_d     = reld_core_tag_tm1_q;
      assign reld_qw_t_d           = reld_qw_tm1_q;
      assign reld_crit_qw_t_d      = reld_crit_qw_tm1_q;
      assign reld_ditc_t_d         = reld_ditc_tm1_q;
      assign reld_data_vld_t_d     = reld_data_vld_tm1_q;
      assign reld_core_tag_tp1_d   = reld_core_tag_t_q;
      assign reld_qw_tp1_d         = reld_qw_t_q;
      assign reld_crit_qw_tp1_d    = reld_crit_qw_t_q;
      assign reld_ditc_tp1_d       = reld_ditc_t_q;
      assign reld_data_vld_tp1_d   = reld_data_vld_t_q;
      assign reld_data_tp1_d       = an_ac_reld_data;
      assign reld_core_tag_tp2_d   = reld_core_tag_tp1_q;
      assign reld_qw_tp2_d         = reld_qw_tp1_q;
      assign reld_crit_qw_tp2_d    = reld_crit_qw_tp1_q;
      assign reld_ditc_tp2_d       = reld_ditc_tp1_q;
      assign reld_data_vld_tp2_d   = reld_data_vld_tp1_q;
      assign reld_data_tp2_d       = reld_data_tp1_q;
      assign reld_ecc_err_tp2_d    = an_ac_reld_ecc_err;
      assign reld_ecc_err_ue_tp2_d = an_ac_reld_ecc_err_ue;

      //---------------------------------------------------------------------
      // Shared: LSU request arbiter (P0-5)
      //---------------------------------------------------------------------
      // One credit token is shared with tlbivax/tlbsync in mmq_inval, so exactly
      // one MMU request may be outstanding to the LSU at a time.  A radix walk
      // re-enters this arbiter once per level; with two contexts the arbiter
      // alternates so neither thread can be starved by the other's walk (P2-10 --
      // context 0 is bound to thread 0 and context 1 to thread 1, so the
      // per-thread reservation is structural rather than policy).
      //
      // Two-phase, mirroring HtwSeq_Stg1/Stg2 (mmq_htw.v:572-601): phase 1 loads
      // the address staging latches and arms; phase 2 asserts req_valid until
      // taken.  The address must be latched before req_valid rises.

      // Flattened out of the unpacked arrays on purpose: a variable index into an
      // unpacked array read inside always@(*) does not yield a dependable
      // sensitivity list, which is the same class of bug PLAN.md 4.3 rule 4 warns
      // about for hand-written Verilog-1995 lists.
      wire ld_req0, ld_req1, ld_req_any, arb_pref;
      assign ld_req0    = ctx_seq_load_req[0];
      assign ld_req1    = ctx_seq_load_req[1];
      assign ld_req_any = ld_req0 | ld_req1;
      assign arb_pref   = (rtw_arb_ptr_q == 1'b0) ? ld_req0 : ld_req1;

      always @(*)
      begin: Rtw_Arbiter
         rtw_arb_gnt       = 1'b0;
         rtw_lsu_req_valid = 1'b0;
         if (rtw_arb_armed_q == 1'b1)
         begin
            // phase 2: address is staged, drive the request
            rtw_lsu_req_valid = mmucr1_rxe;
            rtw_arb_gnt       = rtw_lsu_req_taken;
         end
      end

      // phase 1 selection: preferred context first, then the other one
      assign rtw_arb_sel = (rtw_arb_armed_q == 1'b1) ? rtw_arb_ptr_q :
                           (arb_pref == 1'b1)        ? rtw_arb_ptr_q :
                                                       (~rtw_arb_ptr_q);

      assign rtw_arb_armed_d = (rtw_arb_armed_q == 1'b1) ? (~rtw_lsu_req_taken) :
                               ld_req_any;
      assign rtw_arb_ptr_d   = (rtw_arb_armed_q == 1'b1 & rtw_lsu_req_taken == 1'b1) ? (~rtw_arb_ptr_q) :
                               (rtw_arb_armed_q == 1'b0) ? rtw_arb_sel : rtw_arb_ptr_q;

      assign ctx_load_taken[0] = rtw_arb_gnt & (rtw_arb_ptr_q == 1'b0);
      assign ctx_load_taken[1] = rtw_arb_gnt & (rtw_arb_ptr_q == 1'b1);

      // context N always uses core tag N, so the returning data identifies itself
      wire [0:`THDID_WIDTH-1]         arb_thdid;
      wire [64-`REAL_ADDR_WIDTH:63]   arb_addr;
      wire                            arb_load;
      assign arb_thdid = (rtw_arb_sel == 1'b1) ? ctx_thdid[1]    : ctx_thdid[0];
      assign arb_addr  = (rtw_arb_sel == 1'b1) ? ctx_req_addr[1] : ctx_req_addr[0];
      assign arb_load  = (rtw_arb_armed_q == 1'b0) & ld_req_any;

      assign rtw_lsu_ttype_d = (arb_load) ? ((rtw_arb_sel == 1'b1) ? LsuTtype_Rd1 : LsuTtype_Rd0) :
                                            rtw_lsu_ttype_q;
      assign rtw_lsu_thdid_d = (arb_load) ? arb_thdid : rtw_lsu_thdid_q;
      assign rtw_lsu_addr_d  = (arb_load) ? arb_addr  : rtw_lsu_addr_q;
      // Page-table walk accesses are cacheable, coherent, guarded; not write-through,
      // not cache-inhibited.  WIMGE = 0_0_1_1_0.  Unlike the E.PT walker there is no
      // indirect TLB entry to inherit WIMGE from -- the radix tree has no such entry.
      assign rtw_lsu_wimge_d = (arb_load) ? 5'b00110 : rtw_lsu_wimge_q;
      assign rtw_lsu_u_d     = (arb_load) ? 4'b0000  : rtw_lsu_u_q;

      assign rtw_lsu_ttype = rtw_lsu_ttype_q;
      assign rtw_lsu_thdid = rtw_lsu_thdid_q;
      assign rtw_lsu_wimge = rtw_lsu_wimge_q;
      assign rtw_lsu_u     = rtw_lsu_u_q;
      assign rtw_lsu_addr  = rtw_lsu_addr_q;

      //---------------------------------------------------------------------
      // Shared: LRAT check for guest-mode walks (P2-11)
      //---------------------------------------------------------------------
      // Every level address after the root is read out of guest-writable memory.
      // A2O never had to check a walk address because its single address came
      // from a hypervisor-installed indirect entry.  Here it must be translated
      // and validated before the load is issued, or a corrupted PDE becomes an
      // arbitrary real-address access.
      assign rtw_lrat_req_valid = (ctx_seq_load_req[0] & ctx_tag_q[0][`tagpos_gs]) |
                                  (ctx_seq_load_req[1] & ctx_tag_q[1][`tagpos_gs]);
      assign rtw_lrat_addr      = arb_addr;
      assign rtw_lrat_lpid      = (rtw_arb_sel == 1'b1) ?
                                     ctx_tag_q[1][`tagpos_lpid:`tagpos_lpid+`LPID_WIDTH-1] :
                                     ctx_tag_q[0][`tagpos_lpid:`tagpos_lpid+`LPID_WIDTH-1];

      //---------------------------------------------------------------------
      // Shared: ptereload output (P0-3)
      //---------------------------------------------------------------------
      // EVERY terminating context drives this, not just successful ones.  The LSU
      // ERAT-miss-queue entry is freed only by a returning reload (lq_derat.v:4503-4512),
      // so a silently dropped walk leaks an EMQ entry and hangs the thread forever.
      // Reload, Fault, Killed and Timeout all funnel through here.
      wire rl_req0, rl_req1, rl_pref, rl_sel_req;
      assign rl_req0    = ctx_seq_reload[0];
      assign rl_req1    = ctx_seq_reload[1];
      assign rl_pref    = (reload_ptr_q == 1'b0) ? rl_req0 : rl_req1;
      assign reload_sel = (rl_pref == 1'b1) ? reload_ptr_q : (~reload_ptr_q);
      assign rl_sel_req = (reload_sel == 1'b1) ? rl_req1 : rl_req0;

      always @(*)
      begin: Rtw_ReloadArb
         reload_gnt = rl_sel_req & ptereload_req_taken;
      end

      assign reload_ptr_d = (reload_gnt == 1'b1) ? (~reload_sel) : reload_ptr_q;

      assign ctx_reload_taken[0] = reload_gnt & (reload_sel == 1'b0);
      assign ctx_reload_taken[1] = reload_gnt & (reload_sel == 1'b1);

      // P0-4 last line of defence: an invalidate that landed during the walk has
      // cleared the reservation, so the entry is presented with V=0 and the TLB
      // write is discarded downstream by the wq==2'b10 gate (mmq_tlb_ctl.v:2980).
      // The reload is still returned, which is what frees the EMQ entry.
      assign ptereload_req_valid = mmucr1_rxe & rl_sel_req;
      assign ptereload_req_tag   = (reload_sel == 1'b1) ? ctx_tag_q[1]   : ctx_tag_q[0];
      assign ptereload_req_pte   = (reload_sel == 1'b1) ? ctx_pte_out[1] : ctx_pte_out[0];

      //---------------------------------------------------------------------
      // Shared: quiesce (mirrors mmq_htw.v:555-560)
      //---------------------------------------------------------------------
      assign rtw_quiesce_b = ({`THDID_WIDTH{ctx_valid_q[0]}} & ctx_thdid[0]) |
                             ({`THDID_WIDTH{ctx_valid_q[1]}} & ctx_thdid[1]);
      assign rtw_quiesce   = (~rtw_quiesce_b);

      //---------------------------------------------------------------------
      // Shared: fault reporting, per thread
      //---------------------------------------------------------------------
      assign ctx_fault_val[0] = ctx_seq_reload[0] & (ctx_seq_q[0] == RtwSeq_Fault  |
                                                     ctx_seq_q[0] == RtwSeq_Timeout);
      assign ctx_fault_val[1] = ctx_seq_reload[1] & (ctx_seq_q[1] == RtwSeq_Fault  |
                                                     ctx_seq_q[1] == RtwSeq_Timeout);

      assign rtw_pt_fault  = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_Invalid )}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_Invalid )}} & ctx_thdid[1][0:`MM_THREADS-1]);
      assign rtw_badtree   = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_BadTree )}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_BadTree )}} & ctx_thdid[1][0:`MM_THREADS-1]);
      assign rtw_segerror  = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_SegError)}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_SegError)}} & ctx_thdid[1][0:`MM_THREADS-1]);
      assign rtw_perm_err  = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_PermErr )}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_PermErr )}} & ctx_thdid[1][0:`MM_THREADS-1]);
      assign rtw_rc_err    = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_RcErr   )}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_RcErr   )}} & ctx_thdid[1][0:`MM_THREADS-1]);
      assign rtw_lrat_miss = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_LratMiss)}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_LratMiss)}} & ctx_thdid[1][0:`MM_THREADS-1]);
      assign rtw_mchk      = ({`MM_THREADS{ctx_fault_val[0] & (ctx_fault_q[0] == Flt_Mchk    )}} & ctx_thdid[0][0:`MM_THREADS-1]) |
                             ({`MM_THREADS{ctx_fault_val[1] & (ctx_fault_q[1] == Flt_Mchk    )}} & ctx_thdid[1][0:`MM_THREADS-1]);

      assign rtw_act  = (ctx_valid_q[0] | ctx_valid_q[1] | tlb_rtw_req_valid | mmucr2_act_override) & xu_mm_ccr2_notlb_b;
      assign reld_act = (ctx_valid_q[0] | ctx_valid_q[1] | mmucr2_act_override) & xu_mm_ccr2_notlb_b;

      assign rtw_dbg_seq_idle     = (ctx_seq_q[0] == RtwSeq_Idle) & (ctx_seq_q[1] == RtwSeq_Idle);
      assign rtw_dbg_ctx0_seq_q   = ctx_seq_q[0];
      assign rtw_dbg_ctx1_seq_q   = ctx_seq_q[1];
      assign rtw_dbg_ctx_valid_q  = {ctx_valid_q[0],  ctx_valid_q[1]};
      assign rtw_dbg_ctx_killed_q = {ctx_killed_q[0], ctx_killed_q[1]};
      assign rtw_dbg_ctx0_shift_q = ctx_shift_q[0];
      assign rtw_dbg_ctx1_shift_q = ctx_shift_q[1];
      assign rtw_dbg_ptb_valid_q  = ptb_valid_q;
      assign rtw_dbg_pt0_valid_q  = pt0_valid_q;
      assign rtw_dbg_pt3_valid_q  = pt3_valid_q;

      //---------------------------------------------------------------------
      // Per-context walk datapath and sequencer
      //---------------------------------------------------------------------
      // BIT ORDER.  Microwatt's mmu.vhdl is little-endian (`downto`); A2O is
      // MSB-first (`[0:N]`).  Radix tables are big-endian in memory and the L2
      // returns them MSB-first, so no byte swap is needed -- the translation is
      // purely index inversion:  a2o_index = 63 - microwatt_index.
      // Every field below is annotated with its Microwatt bit number.

      generate
      for (i = 0; i < `RTW_NUM_CTX; i = i + 1)
      begin : gen_ctx

         wire [0:`EPN_WIDTH-1]           epn;
         wire [0:83]                     epn_pad;
         wire [0:15]                     addrsh;
         wire [0:15]                     mask;
         wire [0:30]                     segmask;
         wire [0:29]                     fm30;
         wire [0:3]                      fm4;
         wire [0:11]                     effpid;
         wire [0:`PID_WIDTH-1]           ctx_pid;
         wire [0:5]                      root_rts;
         wire [0:4]                      root_rpds;
         wire [0:4]                      pde_nls;
         wire [0:5]                      seg_newshift;
         wire                            seg_nonzero;
         wire [64-`REAL_ADDR_WIDTH:63]   parttbl_addr;
         wire [64-`REAL_ADDR_WIDTH:63]   prtable_addr;
         wire [64-`REAL_ADDR_WIDTH:63]   pgtable_addr;
         wire [64-`REAL_ADDR_WIDTH:63]   nlb_base;
         wire [0:5]                      inst_shift;
         wire [0:3]                      inst_size;
         wire [0:29]                     rpn_out;
         wire [0:5]                      usxwr;
         wire [0:4]                      wimge_out;
         wire                            pde_v, pde_leaf, pde_priv, pde_ci;
         wire                            pde_x, pde_w, pde_r, pde_rref, pde_c;
         wire                            rc_ok;
         wire                            accept;
         wire                            ra_overflow;
         reg                             seq_valid_set, seq_valid_clr;
         reg                             seq_load_root_part, seq_load_root_pg;
         reg                             seq_install_valid;

         //------------------------------------------------------------------
         // request context
         //------------------------------------------------------------------
         assign epn     = ctx_tag_q[i][`tagpos_epn:`tagpos_epn+`EPN_WIDTH-1];
         assign ctx_pid = ctx_tag_q[i][`tagpos_pid:`tagpos_pid+`PID_WIDTH-1];
         assign ctx_thdid[i] = ctx_tag_q[i][`tagpos_thdid:`tagpos_thdid+`THDID_WIDTH-1];
         // quadrant 3 == EA[0] set == EPN[0].  Microwatt mmu.vhdl:1818-1821 forces
         // PID=0 for quadrant 3, so kernel addresses always use process-table entry 0.
         assign ctx_quad3[i] = epn[0];
         assign effpid  = (ctx_quad3[i]) ? 12'b0 : ctx_pid[`PID_WIDTH-12:`PID_WIDTH-1];

         //------------------------------------------------------------------
         // barrel shifter -- Microwatt addrshifter, mmu.vhdl:1380-1414
         //------------------------------------------------------------------
         // addrsh_mw(a) = addr(12+shift+a).  Converting: addrsh[k] = EPN[36-shift+k].
         // The shifter input is addr(61:12) == epn[2:51], so epn[0:1] (EA63:62)
         // must NOT be visible: Microwatt shifts in zeros from above bit 61.
         // Padding with 34 zeros both excludes them and keeps the part-select base
         // non-negative for any 6-bit shift, which a 3-stage case-mux would
         // otherwise have to special-case.
         assign epn_pad = {34'b0, epn[2:`EPN_WIDTH-1]};
         assign addrsh  = epn_pad[(68 - ctx_shift_q[i]) +: 16];

         //------------------------------------------------------------------
         // index mask -- Microwatt addrmaskgen, mmu.vhdl:1417-1432
         //------------------------------------------------------------------
         // mask_mw(b) = 1 for b<5 (seeded 0x001f), else b < mask_size.
         // mask[k] == mask_mw(15-k).
         for (k = 0; k < 16; k = k + 1)
         begin : gen_mask
            assign mask[k] = ((15 - k) < 5) ? 1'b1 :
                             ((15 - k) < ctx_masksize_q[i]) ? 1'b1 : 1'b0;
         end

         //------------------------------------------------------------------
         // final mask -- Microwatt finalmaskgen, mmu.vhdl:1436-1448
         //------------------------------------------------------------------
         // fm30 merges EA bits into the RPN for pages > 4kB. fm30[r] == fm_mw(29-r).
         for (k = 0; k < 30; k = k + 1)
         begin : gen_fm30
            assign fm30[k] = ((29 - k) < inst_shift) ? 1'b1 : 1'b0;
         end
         // fm4 selects how many PID bits may override the process-table base
         // (Microwatt mmu.vhdl:1823-1826). fm4[t] == fm_mw(3-t), shift == PRTS here.
         for (k = 0; k < 4; k = k + 1)
         begin : gen_fm4
            assign fm4[k] = ((3 - k) < ctx_shift_q[i]) ? 1'b1 : 1'b0;
         end
         // segmask checks that EA bits above 31+RTS are zero (mmu.vhdl:1671).
         // It covers EPN[2:32] == addr(61:31); segmask[s] == fm_mw(30-s).
         for (k = 0; k < 31; k = k + 1)
         begin : gen_segmask
            assign segmask[k] = ((30 - k) < ctx_shift_q[i]) ? 1'b1 : 1'b0;
         end

         //------------------------------------------------------------------
         // table field extraction (a2o_index = 63 - microwatt_index)
         //------------------------------------------------------------------
         // RTS = '0' & data(62:61) & data(7:5)                    mmu.vhdl:1655-1657
         assign root_rts  = {1'b0, ctx_data_q[i][1], ctx_data_q[i][2],
                                   ctx_data_q[i][56], ctx_data_q[i][57], ctx_data_q[i][58]};
         // RPDS / PRTS / NLS = data(4:0)                          mmu.vhdl:1668, 1720
         assign root_rpds = ctx_data_q[i][59:63];
         assign pde_nls   = ctx_data_q[i][59:63];
         // NLB = data(55:8) << 8                                  mmu.vhdl:1670
         assign nlb_base  = {ctx_data_q[i][22:55], 8'b0};

         assign pde_v    = ctx_data_q[i][0];    // mw 63  V
         assign pde_leaf = ctx_data_q[i][1];    // mw 62  L
         assign pde_rref = ctx_data_q[i][55];   // mw  8  R (reference)
         assign pde_c    = ctx_data_q[i][56];   // mw  7  C (change)
         assign pde_ci   = ctx_data_q[i][58];   // mw  5  I (cache inhibited)
         assign pde_priv = ctx_data_q[i][60];   // mw  3  PRIV
         assign pde_r    = ctx_data_q[i][61];   // mw  2  read
         assign pde_w    = ctx_data_q[i][62];   // mw  1  write
         assign pde_x    = ctx_data_q[i][63];   // mw  0  execute

         //------------------------------------------------------------------
         // segment check -- Microwatt SEGMENT_CHECK, mmu.vhdl:1667-1685
         //------------------------------------------------------------------
         assign seg_newshift = ctx_shift_q[i] + 6'd19 - {1'b0, root_rpds};
         assign seg_nonzero  = |(epn[2:32] & (~segmask));

         //------------------------------------------------------------------
         // addresses
         //------------------------------------------------------------------
         // partition table entry 0, doubleword 1 (PATE1) -- mmu.vhdl:1846.
         // LPID and PATS are ignored, exactly as upstream: the partition table
         // is vestigial in Microwatt and A2O's LRAT covers partition scope.
         assign parttbl_addr = {ptcr[22:51], 12'h008};
         // process table entry, 16-byte entries, PID indexed -- mmu.vhdl:1823-1826
         assign prtable_addr = {ctx_data_q[i][22:47],
                                (ctx_data_q[i][48:51] & (~fm4)) | (effpid[0:3] & fm4),
                                effpid[4:11], 4'b0};
         // next PDE: index OR'd into the base, 8-byte entries -- mmu.vhdl:1828-1830
         assign pgtable_addr = {ctx_pgbase_q[i][22:44],
                                (ctx_pgbase_q[i][45:60] & (~mask)) | (addrsh & mask),
                                3'b0};

         assign ctx_req_addr[i] = (ctx_fetch_q[i] == Fetch_Part) ? parttbl_addr :
                                  (ctx_fetch_q[i] == Fetch_Proc) ? prtable_addr :
                                  pgtable_addr;

         // A2O real addresses are 42 bits (`REAL_ADDR_WIDTH). Microwatt forms 56-bit
         // PDE addresses, so a tree built for a wider machine can point outside what
         // this core can address. That is a machine check, not a silent truncation.
         assign ra_overflow = (ctx_fetch_q[i] == Fetch_Pde) ? |(ctx_data_q[i][8:21]) : 1'b0;

         //------------------------------------------------------------------
         // leaf install: size demotion
         //------------------------------------------------------------------
         // A2O's 4-bit size code is log4(size/1KB) so it cannot express 2MB, and
         // mmq_tlb_cmp.v:3486 builds the way size as {1'b0, pte[ptepos_size+0:+2]},
         // which drops everything above 16MB. Radix leaves are therefore installed
         // at the largest representable sub-page size. Always safe: a smaller page
         // maps a subset of the same translation with identical permissions.
         assign inst_shift = (ctx_shift_q[i] >= 6'd12) ? InstShift_16MB :
                             (ctx_shift_q[i] >= 6'd8)  ? InstShift_1MB  :
                             (ctx_shift_q[i] >= 6'd4)  ? RadixShift_64KB :
                                                         RadixShift_4KB;
         assign inst_size  = (ctx_shift_q[i] >= 6'd12) ? TLB_PgSize_16MB :
                             (ctx_shift_q[i] >= 6'd8)  ? TLB_PgSize_1MB  :
                             (ctx_shift_q[i] >= 6'd4)  ? TLB_PgSize_64KB :
                                                         TLB_PgSize_4KB;

         // RPN = (pde & ~finalmask) | (ea & finalmask)            mmu.vhdl:1831-1833
         // RA[41:12] lives at data[22:51] and at epn[22:51].
         assign rpn_out = (ctx_data_q[i][22:51] & (~fm30)) | (epn[22:51] & fm30);

         //------------------------------------------------------------------
         // permissions -- Microwatt check_perm_c, mmu.vhdl:351-371
         //------------------------------------------------------------------
         // Execute is denied on cache-inhibited pages (mmu.vhdl:367). There is no
         // IAMR/AMR in either core, so no KUEP/KUAP (mmu.vhdl:365, dcache.vhdl:1103).
         assign usxwr[0] = pde_x & (~pde_ci) & (~pde_priv);   // UX
         assign usxwr[1] = pde_x & (~pde_ci);                 // SX
         assign usxwr[2] = pde_w & (~pde_priv);               // UW
         assign usxwr[3] = pde_w;                             // SW
         assign usxwr[4] = pde_r & (~pde_priv);               // UR
         assign usxwr[5] = pde_r;                             // SR

         // P0-2: R and C are CHECKED, never written back. There is no store path
         // from the MMU (lq_imq.v:107) and no pending-memory-write mechanism, so a
         // hardware R/C update would be an architecturally visible write on behalf
         // of a non-committed instruction. Software must set them, as in Microwatt.
         // Conservative: the A2O tag carries no load/store bit, so C is required
         // unconditionally rather than only for stores (Microwatt mmu.vhdl:1700
         // can test r.store because loadstore1 tells it).
         assign rc_ok = pde_rref & pde_c;

         assign wimge_out = {1'b0, pde_ci, 1'b1, pde_ci, 1'b0};   // W I M G E

         // A2O ptepos format; mmq_tlb_cmp.v:3576-3581 folds r/c into usxwr, and
         // RA[41:12] sits at ptepos_rpn+10 .. +39 (see mmq_tlb_ctl.v:3229).
         assign ctx_pte_out[i] = {10'b0, rpn_out,                  // ptepos_rpn   0:39
                                  wimge_out,                       // ptepos_wimge 40:44
                                  1'b1,                            // ptepos_r     45
                                  4'b0,                            // ptepos_ubits 46:49
                                  1'b0,                            // ptepos_sw0   50
                                  1'b1,                            // ptepos_c     51
                                  inst_size,                       // ptepos_size  52:55
                                  usxwr,                           // ptepos_usxwr 56:61
                                  1'b0,                            // ptepos_sw1   62
                                  seq_install_valid};              // ptepos_valid 63

         //------------------------------------------------------------------
         // P0-1 / P0-4 / P1-6: kill conditions, re-tested at every level
         //------------------------------------------------------------------
         assign ctx_kill_now[i] = (`MM_THREADS > i) ? xu_ex5_flush[i] : 1'b0;
         // EPN is deliberately NOT compared: a radix walk reads 4-5 different
         // addresses and an invalidate may target a directory level we already
         // consumed. Matches the tlbilx T=0 semantics at mmq_htw.v:963.
         assign ctx_inv_match[i] = inv_seq_inprogress &
                                   (inv_all |
                                    ((inv_lpid == ctx_tag_q[i][`tagpos_lpid:`tagpos_lpid+`LPID_WIDTH-1]) &
                                     (inv_gs   == ctx_tag_q[i][`tagpos_gs]) &
                                     (inv_as   == ctx_tag_q[i][`tagpos_as]) &
                                     ((inv_pid == ctx_pid) | (inv_pid == {`PID_WIDTH{1'b0}}))));

         assign ctx_wd_expired[i] = (ctx_wd_q[i] == {`RTW_WD_WIDTH{1'b1}});

         // context N is statically bound to thread N (P2-10): the per-thread
         // resource reservation is structural, so neither thread can starve the
         // other of walk capacity the way they can with mmq_htw's shared slots.
         assign accept = tlb_rtw_req_valid & mmucr1_rxe & (~ctx_valid_q[i]) &
                         tlb_rtw_req_tag[`tagpos_thdid + i];

         // P1-8: a corrected-ECC replay is retried a bounded number of times and
         // then escalated. mmq_htw.v has no counter, so a line that keeps failing
         // loops forever (clear reservation -> discard -> restart -> same error).
         wire retry_exhausted;
         assign retry_exhausted = (ctx_retry_q[i] == {`RTW_RETRY_WIDTH{1'b1}});

         // incoming-request view of the root cache (the tag is not latched yet)
         wire        req_quad3;
         wire        req_root_cached;
         wire [0:63] req_root;
         wire [0:5]  req_root_rts;
         wire [0:5]  prtbl_prts;
         assign req_quad3       = tlb_rtw_req_tag[`tagpos_epn];
         assign req_root_cached = (req_quad3) ? pt3_valid_q : pt0_valid_q;
         assign req_root        = (req_quad3) ? pgtbl3_q : pgtbl0_q;
         assign req_root_rts    = {1'b0, req_root[1], req_root[2],
                                         req_root[56], req_root[57], req_root[58]};
         assign prtbl_prts      = {1'b0, prtbl_q[59:63]};

         //------------------------------------------------------------------
         // Sequencer
         //------------------------------------------------------------------
         // SENSITIVITY LIST: PLAN.md 4.3 rule 4 requires an explicit Verilog-1995
         // list because a missing entry is a silent sim/synth mismatch. Inside a
         // generate loop over arrayed context state a hand-written list is exactly
         // the thing that goes stale, so `always @(*)` is used instead -- it
         // enforces the rule's intent (complete sensitivity) by construction. The
         // full default-assignment prologue below is kept verbatim from the house
         // style; a missing default here still infers a latch.
         always @(*)
         begin : Rtw_Sequencer
            ctx_seq_d[i]        = ctx_seq_q[i];
            ctx_shift_d[i]      = ctx_shift_q[i];
            ctx_masksize_d[i]   = ctx_masksize_q[i];
            ctx_fault_d[i]      = ctx_fault_q[i];
            ctx_fetch_d[i]      = ctx_fetch_q[i];
            ctx_pgbase_d[i]     = ctx_pgbase_q[i];
            ctx_seq_load_req[i] = 1'b0;
            ctx_seq_reload[i]   = 1'b0;
            ctx_seq_done[i]     = 1'b0;
            ctx_seq_retry[i]    = 1'b0;
            ctx_wr_prtbl[i]     = 1'b0;
            ctx_wr_pgtbl[i]     = 1'b0;
            seq_valid_set       = 1'b0;
            seq_valid_clr       = 1'b0;
            seq_load_root_part  = 1'b0;
            seq_load_root_pg    = 1'b0;
            seq_install_valid   = 1'b0;

            case (ctx_seq_q[i])

               RtwSeq_Idle :
                  if (accept == 1'b1)
                  begin
                     seq_valid_set = 1'b1;
                     if (ptb_valid_q == 1'b0)
                     begin
                        // no cached partition-table entry: full walk from PTCR
                        ctx_fetch_d[i] = Fetch_Part;
                        ctx_seq_d[i]   = RtwSeq_PartRd;
                     end
                     else if (req_root_cached == 1'b0)
                     begin
                        // partition entry cached, process-table root is not
                        seq_load_root_part = 1'b1;          // ctx_data <= prtbl_q
                        ctx_shift_d[i]     = prtbl_prts;    // PRTS indexes the PID
                        ctx_fetch_d[i]     = Fetch_Proc;
                        ctx_seq_d[i]       = RtwSeq_ProcRd;
                     end
                     else
                     begin
                        // both roots cached: straight into the tree
                        seq_load_root_pg = 1'b1;            // ctx_data <= pgtblN_q
                        ctx_shift_d[i]   = req_root_rts;
                        ctx_seq_d[i]     = RtwSeq_SegChk;
                     end
                  end

               // ---- partition table: PATE1 at (PTCR & ~0xFFF) + 8 -------------
               RtwSeq_PartRd :
                  if (ctx_kill_now[i] | ctx_killed_q[i] | (~ctx_resv_q[i]))
                     ctx_seq_d[i] = RtwSeq_Killed;
                  else if (ctx_wd_expired[i] == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Timeout;
                  end
                  else
                  begin
                     ctx_seq_load_req[i] = 1'b1;
                     if (ctx_load_taken[i] == 1'b1)
                        ctx_seq_d[i] = RtwSeq_PartWait;
                  end

               RtwSeq_PartWait :
                  if (ctx_wd_expired[i] == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Timeout;
                  end
                  else if (ctx_dataval_q[i] == 1'b1)
                  begin
                     // P0-1 / P0-4: also test the kill and reservation on the way OUT
                     // of a wait state. Gating only the request states would let a
                     // flush or invalidate arriving during the final level complete
                     // the walk and install a translation that must not be installed.
                     // Safe here: dataval means the load already returned, so this
                     // never abandons an in-flight reload.
                     if (ctx_kill_now[i] | ctx_killed_q[i] | (~ctx_resv_q[i]))
                        ctx_seq_d[i] = RtwSeq_Killed;
                     else
                     if (ctx_err_q[i][1] == 1'b1)          // uncorrectable
                     begin
                        ctx_fault_d[i] = Flt_Mchk;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if ((ctx_err_q[i] == 3'b100) & (~retry_exhausted))
                     begin                                 // corrected, replay line
                        ctx_seq_retry[i] = 1'b1;
                        ctx_seq_d[i]     = RtwSeq_PartRd;
                     end
                     else if (ctx_err_q[i] == 3'b100)
                     begin
                        ctx_fault_d[i] = Flt_Mchk;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else
                     begin
                        ctx_wr_prtbl[i] = 1'b1;            // cache PATE1
                        ctx_shift_d[i]  = {1'b0, ctx_data_q[i][59:63]};   // PRTS
                        ctx_fetch_d[i]  = Fetch_Proc;
                        ctx_seq_d[i]    = RtwSeq_ProcRd;
                     end
                  end

               // ---- process table: PRTE0, PID indexed -----------------------
               RtwSeq_ProcRd :
                  if (ctx_kill_now[i] | ctx_killed_q[i] | (~ctx_resv_q[i]))
                     ctx_seq_d[i] = RtwSeq_Killed;
                  else if (ctx_wd_expired[i] == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Timeout;
                  end
                  else
                  begin
                     ctx_seq_load_req[i] = 1'b1;
                     if (ctx_load_taken[i] == 1'b1)
                        ctx_seq_d[i] = RtwSeq_ProcWait;
                  end

               RtwSeq_ProcWait :
                  if (ctx_wd_expired[i] == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Timeout;
                  end
                  else if (ctx_dataval_q[i] == 1'b1)
                  begin
                     // P0-1 / P0-4: also test the kill and reservation on the way OUT
                     // of a wait state. Gating only the request states would let a
                     // flush or invalidate arriving during the final level complete
                     // the walk and install a translation that must not be installed.
                     // Safe here: dataval means the load already returned, so this
                     // never abandons an in-flight reload.
                     if (ctx_kill_now[i] | ctx_killed_q[i] | (~ctx_resv_q[i]))
                        ctx_seq_d[i] = RtwSeq_Killed;
                     else
                     if (ctx_err_q[i][1] == 1'b1)
                     begin
                        ctx_fault_d[i] = Flt_Mchk;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if ((ctx_err_q[i] == 3'b100) & (~retry_exhausted))
                     begin
                        ctx_seq_retry[i] = 1'b1;
                        ctx_seq_d[i]     = RtwSeq_ProcRd;
                     end
                     else if (ctx_err_q[i] == 3'b100)
                     begin
                        ctx_fault_d[i] = Flt_Mchk;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else
                     begin
                        ctx_wr_pgtbl[i] = 1'b1;            // cache the quadrant root
                        ctx_shift_d[i]  = root_rts;        // RTS
                        ctx_seq_d[i]    = RtwSeq_SegChk;
                     end
                  end

               // ---- segment check -- mmu.vhdl:1667-1685 ---------------------
               RtwSeq_SegChk :
                  begin
                     ctx_masksize_d[i] = root_rpds;
                     ctx_pgbase_d[i]   = nlb_base;
                     ctx_shift_d[i]    = seg_newshift;
                     ctx_fetch_d[i]    = Fetch_Pde;
                     if (root_rpds == 5'b00000)
                     begin
                        // RPDS = 0 disables radix walks for this process
                        ctx_fault_d[i] = Flt_Invalid;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if ((epn[0] != epn[1]) | (seg_nonzero == 1'b1))
                     begin
                        // quadrants 1 and 2 are not mapped; EA above 31+RTS must be 0
                        ctx_fault_d[i] = Flt_SegError;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if ((root_rpds < 5'd5) | (root_rpds > 5'd16) |
                              ({1'b0, root_rpds} > (ctx_shift_q[i] + 6'd19)))
                     begin
                        ctx_fault_d[i] = Flt_BadTree;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else
                        ctx_seq_d[i] = RtwSeq_Lookup;
                  end

               // ---- one tree level -- mmu.vhdl:1687-1747 --------------------
               RtwSeq_Lookup :
                  // P0-1 / P0-4 / P1-6: every level boundary re-tests the kill and
                  // reservation state. nonspec is sampled once at handoff and never
                  // re-evaluated by the hardware, so the walker does it here.
                  if (ctx_kill_now[i] | ctx_killed_q[i] | (~ctx_resv_q[i]))
                     ctx_seq_d[i] = RtwSeq_Killed;
                  else if (ctx_wd_expired[i] == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Timeout;
                  end
                  else if (ra_overflow == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Fault;
                  end
                  // P2-11: in guest mode the address came out of guest-writable
                  // memory and must be LRAT-translated before it leaves the core.
                  else if ((ctx_tag_q[i][`tagpos_gs] == 1'b1) & (rtw_lrat_hit == 1'b0))
                  begin
                     ctx_fault_d[i] = Flt_LratMiss;
                     ctx_seq_d[i]   = RtwSeq_Fault;
                  end
                  else
                  begin
                     ctx_seq_load_req[i] = 1'b1;
                     if (ctx_load_taken[i] == 1'b1)
                        ctx_seq_d[i] = RtwSeq_ReadWait;
                  end

               RtwSeq_ReadWait :
                  if (ctx_wd_expired[i] == 1'b1)
                  begin
                     ctx_fault_d[i] = Flt_Mchk;
                     ctx_seq_d[i]   = RtwSeq_Timeout;
                  end
                  else if (ctx_dataval_q[i] == 1'b1)
                  begin
                     // P0-1 / P0-4: also test the kill and reservation on the way OUT
                     // of a wait state. Gating only the request states would let a
                     // flush or invalidate arriving during the final level complete
                     // the walk and install a translation that must not be installed.
                     // Safe here: dataval means the load already returned, so this
                     // never abandons an in-flight reload.
                     if (ctx_kill_now[i] | ctx_killed_q[i] | (~ctx_resv_q[i]))
                        ctx_seq_d[i] = RtwSeq_Killed;
                     else
                     if (ctx_err_q[i][1] == 1'b1)
                     begin
                        // P1-8: a persistent UE would livelock. Escalate instead.
                        ctx_fault_d[i] = Flt_Mchk;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if ((ctx_err_q[i] == 3'b100) & (~retry_exhausted))
                     begin
                        ctx_seq_retry[i] = 1'b1;
                        ctx_seq_d[i]     = RtwSeq_Lookup;
                     end
                     else if (ctx_err_q[i] == 3'b100)
                     begin
                        ctx_fault_d[i] = Flt_Mchk;
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if (pde_v == 1'b0)
                     begin
                        ctx_fault_d[i] = Flt_Invalid;      // non-present, -> DSI
                        ctx_seq_d[i]   = RtwSeq_Fault;
                     end
                     else if (pde_leaf == 1'b1)
                     begin
                        if (rc_ok == 1'b0)
                        begin
                           ctx_fault_d[i] = Flt_RcErr;
                           ctx_seq_d[i]   = RtwSeq_Fault;
                        end
                        else
                           ctx_seq_d[i] = RtwSeq_Reload;
                     end
                     else
                     begin
                        // descend. Termination is shift exhaustion, guarded by
                        // NLS > shift -- there is no explicit level counter, exactly
                        // as in Microwatt (mmu.vhdl:1719-1736).
                        if ((pde_nls < 5'd5) | (pde_nls > 5'd16) |
                            ({1'b0, pde_nls} > ctx_shift_q[i]))
                        begin
                           ctx_fault_d[i] = Flt_BadTree;
                           ctx_seq_d[i]   = RtwSeq_Fault;
                        end
                        else
                        begin
                           ctx_shift_d[i]    = ctx_shift_q[i] - {1'b0, pde_nls};
                           ctx_masksize_d[i] = pde_nls;
                           ctx_pgbase_d[i]   = nlb_base;
                           ctx_seq_d[i]      = RtwSeq_Lookup;
                        end
                     end
                  end

               // ---- terminal states ----------------------------------------
               // P0-3: all four drive ptereload. A silently dropped walk leaks the
               // LSU ERAT-miss-queue entry and hangs the thread forever.
               RtwSeq_Reload :
                  begin
                     ctx_seq_reload[i] = 1'b1;
                     seq_install_valid = 1'b1;
                     if (ctx_reload_taken[i] == 1'b1)
                     begin
                        seq_valid_clr   = 1'b1;
                        ctx_seq_done[i] = 1'b1;
                        ctx_seq_d[i]    = RtwSeq_Idle;
                     end
                  end

               RtwSeq_Fault :
                  begin
                     ctx_seq_reload[i] = 1'b1;
                     seq_install_valid = 1'b0;      // V=0 -> no TLB write downstream
                     if (ctx_reload_taken[i] == 1'b1)
                     begin
                        seq_valid_clr   = 1'b1;
                        ctx_seq_done[i] = 1'b1;
                        ctx_seq_d[i]    = RtwSeq_Idle;
                     end
                  end

               RtwSeq_Killed :
                  begin
                     ctx_seq_reload[i] = 1'b1;
                     seq_install_valid = 1'b0;
                     if (ctx_reload_taken[i] == 1'b1)
                     begin
                        seq_valid_clr   = 1'b1;
                        ctx_seq_done[i] = 1'b1;
                        ctx_seq_d[i]    = RtwSeq_Idle;
                     end
                  end

               RtwSeq_Timeout :
                  begin
                     ctx_seq_reload[i] = 1'b1;
                     seq_install_valid = 1'b0;
                     if (ctx_reload_taken[i] == 1'b1)
                     begin
                        seq_valid_clr   = 1'b1;
                        ctx_seq_done[i] = 1'b1;
                        ctx_seq_d[i]    = RtwSeq_Idle;
                     end
                  end

               default :
                  ctx_seq_d[i] = RtwSeq_Idle;

            endcase
         end

         // radix disabled -> the whole context is forced to Idle. Idle is all-zeros
         // so this is the same AND-mask idiom as tlb_seq_abort (mmq_tlb_ctl.v:1378).
         assign ctx_seq_din[i] = ctx_seq_d[i] & {`RTW_SEQ_WIDTH{mmucr1_rxe}};

         //------------------------------------------------------------------
         // Per-context state
         //------------------------------------------------------------------
         wire [0:4] my_core_tag;
         assign my_core_tag = (i == 0) ? Core_Tag0_Value : Core_Tag1_Value;

         assign ctx_valid_d[i]  = (seq_valid_set) ? 1'b1 :
                                  (seq_valid_clr) ? 1'b0 : ctx_valid_q[i];

         // P0-1: a flush arriving mid-walk is remembered here and acted on at the
         // next level boundary. It is never acted on mid-load: the L2 reload is
         // already in flight and must be drained.
         assign ctx_killed_d[i] = (seq_valid_set) ? 1'b0 :
                                  (ctx_valid_q[i] & ctx_kill_now[i]) ? 1'b1 : ctx_killed_q[i];

         // P0-4: reservation. Cleared by any matching invalidate or by an
         // uncorrectable error; checked again before every level's load and used
         // downstream by the wq==2'b10 TLB-write gate (mmq_tlb_ctl.v:2980).
         assign ctx_resv_d[i]   = (seq_valid_set) ? 1'b1 :
                                  (ctx_valid_q[i] & ctx_inv_match[i]) ? 1'b0 :
                                  (ctx_valid_q[i] & ctx_err_q[i][1]) ? 1'b0 : ctx_resv_q[i];

         assign ctx_pending_d[i] = (ctx_load_taken[i]) ? 1'b1 :
                                   (ctx_dataval_q[i] | ctx_seq_done[i] | ctx_seq_retry[i]) ? 1'b0 :
                                   ctx_pending_q[i];

         assign ctx_tag_d[i] = (accept) ? tlb_rtw_req_tag : ctx_tag_q[i];
         assign ctx_way_d[i] = (accept) ? tlb_rtw_req_way : ctx_way_q[i];

         // P1-7: watchdog. Nothing else in mmq_* has a timeout; if a walk stalls the
         // slot never frees, quiesce never asserts and the thread hangs forever.
         // It measures TIME SINCE LAST PROGRESS, not time since the load was issued:
         // a request that is never granted by the LSU arbiter stalls just as hard as
         // one whose data never returns, and an earlier version that counted only
         // while a load was outstanding missed the former entirely.
         // Terminal states are excluded -- they are waiting on ptereload_req_taken,
         // and tripping there would only swap one wait for an identical one.
         assign ctx_wd_run[i] = ctx_valid_q[i] &
                                (ctx_seq_q[i] != RtwSeq_Reload) & (ctx_seq_q[i] != RtwSeq_Fault) &
                                (ctx_seq_q[i] != RtwSeq_Killed) & (ctx_seq_q[i] != RtwSeq_Timeout);
         assign ctx_wd_d[i] = (ctx_wd_run[i] == 1'b0) ? {`RTW_WD_WIDTH{1'b0}} :
                              (ctx_seq_q[i] != ctx_seq_din[i]) ? {`RTW_WD_WIDTH{1'b0}} :
                              (ctx_wd_expired[i] == 1'b1) ? ctx_wd_q[i] :
                              (ctx_wd_q[i] + 1'b1);

         // P1-8: retry counter, so a persistent ECC error escalates instead of
         // looping forever (mmq_htw.v has no counter at all).
         assign ctx_retry_d[i] = (seq_valid_set) ? {`RTW_RETRY_WIDTH{1'b0}} :
                                 (ctx_seq_retry[i]) ? (ctx_retry_q[i] + 1'b1) : ctx_retry_q[i];

         //------------------------------------------------------------------
         // reload capture (mmq_htw.v:1162-1205 pattern, one context per core tag)
         //------------------------------------------------------------------
         assign ctx_cloff_d[i] = (ctx_load_taken[i]) ? ctx_req_addr[i][58:60] : ctx_cloff_q[i];

         assign ctx_reld_for_me_tp2[i] = reld_data_vld_tp2_q & (~reld_ditc_tp2_q) &
                                         reld_crit_qw_tp2_q &
                                         (reld_qw_tp2_q == ctx_cloff_q[i][58:59]) &
                                         (reld_core_tag_tp2_q == my_core_tag);

         assign ctx_data_d[i] = (seq_load_root_part) ? prtbl_q :
                                (seq_load_root_pg)   ? req_root :
                                (ctx_reld_for_me_tp2[i] & (ctx_cloff_q[i][60] == 1'b0)) ? reld_data_tp2_q[0:63] :
                                (ctx_reld_for_me_tp2[i] & (ctx_cloff_q[i][60] == 1'b1)) ? reld_data_tp2_q[64:127] :
                                ctx_data_q[i];

         assign ctx_dataval_d[i] = (ctx_load_taken[i] | ctx_seq_retry[i] | ctx_seq_done[i]) ? 1'b0 :
                                   (ctx_reld_for_me_tp2[i]) ? 1'b1 : ctx_dataval_q[i];

         // 4 quadword beats; the whole line is resent if any beat had an ECC error
         for (k = 0; k < 4; k = k + 1)
         begin : gen_qwbeat
            localparam [1:0] KQW = k;
            assign ctx_qwbeat_d[i][k] = (ctx_load_taken[i] | ctx_seq_retry[i]) ? 1'b0 :
                                        (ctx_pending_q[i] & reld_data_vld_tp2_q & (~reld_ditc_tp2_q) &
                                         (reld_core_tag_tp2_q == my_core_tag) &
                                         (reld_qw_tp2_q == KQW)) ? 1'b1 :
                                        ctx_qwbeat_q[i][k];
         end

         // bit0 = ECC corrected, bit1 = uncorrectable, bit2 = retry taken
         assign ctx_err_d[i][0] = (ctx_load_taken[i]) ? 1'b0 :
                                  (ctx_pending_q[i] & reld_data_vld_tp2_q & (~reld_ditc_tp2_q) &
                                   (reld_core_tag_tp2_q == my_core_tag) & reld_ecc_err_tp2_q) ? 1'b1 :
                                  ctx_err_q[i][0];
         assign ctx_err_d[i][1] = (ctx_load_taken[i]) ? 1'b0 :
                                  (ctx_pending_q[i] & reld_data_vld_tp2_q & (~reld_ditc_tp2_q) &
                                   (reld_core_tag_tp2_q == my_core_tag) & reld_ecc_err_ue_tp2_q) ? 1'b1 :
                                  ctx_err_q[i][1];
         assign ctx_err_d[i][2] = (ctx_load_taken[i]) ? 1'b0 :
                                  (ctx_seq_retry[i]) ? 1'b1 : ctx_err_q[i][2];

      end
      endgenerate

      //---------------------------------------------------------------------
      // Latches
      //---------------------------------------------------------------------
      // Scan latches, not always@(posedge) -- PLAN.md 4.3 rule 2, modelled on
      // mmq_tlb_ctl.v:4165-4181. Per-context latches are instantiated from a
      // generate loop with the scan offset computed as ctx_base_offset + i*CTX_STRIDE.

      generate
      for (i = 0; i < `RTW_NUM_CTX; i = i + 1)
      begin : gen_ctx_lat
         localparam CB = ctx_base_offset + (i * CTX_STRIDE);

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) ctx_valid_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_valid]), .scout(sov_0[CB + cf_valid]),
         .din(ctx_valid_d[i]), .dout(ctx_valid_q[i])
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) ctx_killed_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_killed]), .scout(sov_0[CB + cf_killed]),
         .din(ctx_killed_d[i]), .dout(ctx_killed_q[i])
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) ctx_pending_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_pending]), .scout(sov_0[CB + cf_pending]),
         .din(ctx_pending_d[i]), .dout(ctx_pending_q[i])
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) ctx_resv_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_resv]), .scout(sov_0[CB + cf_resv]),
         .din(ctx_resv_d[i]), .dout(ctx_resv_q[i])
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) ctx_dataval_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_dataval]), .scout(sov_0[CB + cf_dataval]),
         .din(ctx_dataval_d[i]), .dout(ctx_dataval_q[i])
      );

      tri_rlmreg_p #(.WIDTH(`RTW_SEQ_WIDTH), .INIT(0), .NEEDS_SRESET(1)) ctx_seq_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_seq:CB + cf_seq + `RTW_SEQ_WIDTH - 1]), .scout(sov_0[CB + cf_seq:CB + cf_seq + `RTW_SEQ_WIDTH - 1]),
         .din(ctx_seq_din[i]), .dout(ctx_seq_q[i])
      );

      tri_rlmreg_p #(.WIDTH(6), .INIT(0), .NEEDS_SRESET(1)) ctx_shift_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_shift:CB + cf_shift + 6 - 1]), .scout(sov_0[CB + cf_shift:CB + cf_shift + 6 - 1]),
         .din(ctx_shift_d[i]), .dout(ctx_shift_q[i])
      );

      tri_rlmreg_p #(.WIDTH(5), .INIT(0), .NEEDS_SRESET(1)) ctx_masksize_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_masksize:CB + cf_masksize + 5 - 1]), .scout(sov_0[CB + cf_masksize:CB + cf_masksize + 5 - 1]),
         .din(ctx_masksize_d[i]), .dout(ctx_masksize_q[i])
      );

      tri_rlmreg_p #(.WIDTH(3), .INIT(0), .NEEDS_SRESET(1)) ctx_fault_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_fault:CB + cf_fault + 3 - 1]), .scout(sov_0[CB + cf_fault:CB + cf_fault + 3 - 1]),
         .din(ctx_fault_d[i]), .dout(ctx_fault_q[i])
      );

      tri_rlmreg_p #(.WIDTH(2), .INIT(0), .NEEDS_SRESET(1)) ctx_fetch_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_fetch:CB + cf_fetch + 2 - 1]), .scout(sov_0[CB + cf_fetch:CB + cf_fetch + 2 - 1]),
         .din(ctx_fetch_d[i]), .dout(ctx_fetch_q[i])
      );

      tri_rlmreg_p #(.WIDTH(3), .INIT(0), .NEEDS_SRESET(1)) ctx_cloff_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_cloff:CB + cf_cloff + 3 - 1]), .scout(sov_0[CB + cf_cloff:CB + cf_cloff + 3 - 1]),
         .din(ctx_cloff_d[i]), .dout(ctx_cloff_q[i])
      );

      tri_rlmreg_p #(.WIDTH(4), .INIT(0), .NEEDS_SRESET(1)) ctx_qwbeat_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_qwbeat:CB + cf_qwbeat + 4 - 1]), .scout(sov_0[CB + cf_qwbeat:CB + cf_qwbeat + 4 - 1]),
         .din(ctx_qwbeat_d[i]), .dout(ctx_qwbeat_q[i])
      );

      tri_rlmreg_p #(.WIDTH(3), .INIT(0), .NEEDS_SRESET(1)) ctx_err_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_err:CB + cf_err + 3 - 1]), .scout(sov_0[CB + cf_err:CB + cf_err + 3 - 1]),
         .din(ctx_err_d[i]), .dout(ctx_err_q[i])
      );

      tri_rlmreg_p #(.WIDTH(`RTW_RETRY_WIDTH), .INIT(0), .NEEDS_SRESET(1)) ctx_retry_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_retry:CB + cf_retry + `RTW_RETRY_WIDTH - 1]), .scout(sov_0[CB + cf_retry:CB + cf_retry + `RTW_RETRY_WIDTH - 1]),
         .din(ctx_retry_d[i]), .dout(ctx_retry_q[i])
      );

      tri_rlmreg_p #(.WIDTH(`RTW_WD_WIDTH), .INIT(0), .NEEDS_SRESET(1)) ctx_wd_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_wd:CB + cf_wd + `RTW_WD_WIDTH - 1]), .scout(sov_0[CB + cf_wd:CB + cf_wd + `RTW_WD_WIDTH - 1]),
         .din(ctx_wd_d[i]), .dout(ctx_wd_q[i])
      );

      tri_rlmreg_p #(.WIDTH(`REAL_ADDR_WIDTH), .INIT(0), .NEEDS_SRESET(1)) ctx_pgbase_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_pgbase:CB + cf_pgbase + `REAL_ADDR_WIDTH - 1]), .scout(sov_0[CB + cf_pgbase:CB + cf_pgbase + `REAL_ADDR_WIDTH - 1]),
         .din(ctx_pgbase_d[i]), .dout(ctx_pgbase_q[i])
      );

      tri_rlmreg_p #(.WIDTH(64), .INIT(0), .NEEDS_SRESET(1)) ctx_data_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_data:CB + cf_data + 64 - 1]), .scout(sov_0[CB + cf_data:CB + cf_data + 64 - 1]),
         .din(ctx_data_d[i]), .dout(ctx_data_q[i])
      );

      tri_rlmreg_p #(.WIDTH(`TLB_TAG_WIDTH), .INIT(0), .NEEDS_SRESET(1)) ctx_tag_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_tag:CB + cf_tag + `TLB_TAG_WIDTH - 1]), .scout(sov_0[CB + cf_tag:CB + cf_tag + `TLB_TAG_WIDTH - 1]),
         .din(ctx_tag_d[i]), .dout(ctx_tag_q[i])
      );

      tri_rlmreg_p #(.WIDTH((`TLB_WAY_WIDTH-`TLB_WORD_WIDTH)), .INIT(0), .NEEDS_SRESET(1)) ctx_way_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[CB + cf_way:CB + cf_way + (`TLB_WAY_WIDTH-`TLB_WORD_WIDTH) - 1]), .scout(sov_0[CB + cf_way:CB + cf_way + (`TLB_WAY_WIDTH-`TLB_WORD_WIDTH) - 1]),
         .din(ctx_way_d[i]), .dout(ctx_way_q[i])
      );

      end
      endgenerate


      tri_rlmreg_p #(.WIDTH(16), .INIT(0), .NEEDS_SRESET(1)) spare_a_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(1'b0),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_0[spare_a_offset:spare_a_offset + 16 - 1]), .scout(sov_0[spare_a_offset:spare_a_offset + 16 - 1]),
         .din(spare_a_q), .dout(spare_a_q)
      );

      //---------------------------------------------------------------------
      // Chain 1: shared root cache, LSU staging, reload pipe
      //---------------------------------------------------------------------

      tri_rlmreg_p #(.WIDTH(64), .INIT(0), .NEEDS_SRESET(1)) prtbl_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[prtbl_offset:prtbl_offset + 64 - 1]), .scout(sov_1[prtbl_offset:prtbl_offset + 64 - 1]),
         .din(prtbl_d), .dout(prtbl_q)
      );

      tri_rlmreg_p #(.WIDTH(64), .INIT(0), .NEEDS_SRESET(1)) pgtbl0_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[pgtbl0_offset:pgtbl0_offset + 64 - 1]), .scout(sov_1[pgtbl0_offset:pgtbl0_offset + 64 - 1]),
         .din(pgtbl0_d), .dout(pgtbl0_q)
      );

      tri_rlmreg_p #(.WIDTH(64), .INIT(0), .NEEDS_SRESET(1)) pgtbl3_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[pgtbl3_offset:pgtbl3_offset + 64 - 1]), .scout(sov_1[pgtbl3_offset:pgtbl3_offset + 64 - 1]),
         .din(pgtbl3_d), .dout(pgtbl3_q)
      );

      tri_rlmreg_p #(.WIDTH(2), .INIT(0), .NEEDS_SRESET(1)) rtw_lsu_ttype_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_lsu_ttype_offset:rtw_lsu_ttype_offset + 2 - 1]), .scout(sov_1[rtw_lsu_ttype_offset:rtw_lsu_ttype_offset + 2 - 1]),
         .din(rtw_lsu_ttype_d), .dout(rtw_lsu_ttype_q)
      );

      tri_rlmreg_p #(.WIDTH(`THDID_WIDTH), .INIT(0), .NEEDS_SRESET(1)) rtw_lsu_thdid_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_lsu_thdid_offset:rtw_lsu_thdid_offset + `THDID_WIDTH - 1]), .scout(sov_1[rtw_lsu_thdid_offset:rtw_lsu_thdid_offset + `THDID_WIDTH - 1]),
         .din(rtw_lsu_thdid_d), .dout(rtw_lsu_thdid_q)
      );

      tri_rlmreg_p #(.WIDTH(5), .INIT(0), .NEEDS_SRESET(1)) rtw_lsu_wimge_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_lsu_wimge_offset:rtw_lsu_wimge_offset + 5 - 1]), .scout(sov_1[rtw_lsu_wimge_offset:rtw_lsu_wimge_offset + 5 - 1]),
         .din(rtw_lsu_wimge_d), .dout(rtw_lsu_wimge_q)
      );

      tri_rlmreg_p #(.WIDTH(4), .INIT(0), .NEEDS_SRESET(1)) rtw_lsu_u_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_lsu_u_offset:rtw_lsu_u_offset + 4 - 1]), .scout(sov_1[rtw_lsu_u_offset:rtw_lsu_u_offset + 4 - 1]),
         .din(rtw_lsu_u_d), .dout(rtw_lsu_u_q)
      );

      tri_rlmreg_p #(.WIDTH(`REAL_ADDR_WIDTH), .INIT(0), .NEEDS_SRESET(1)) rtw_lsu_addr_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_lsu_addr_offset:rtw_lsu_addr_offset + `REAL_ADDR_WIDTH - 1]), .scout(sov_1[rtw_lsu_addr_offset:rtw_lsu_addr_offset + `REAL_ADDR_WIDTH - 1]),
         .din(rtw_lsu_addr_d), .dout(rtw_lsu_addr_q)
      );

      tri_rlmreg_p #(.WIDTH(5), .INIT(0), .NEEDS_SRESET(1)) reld_core_tag_tm1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_core_tag_tm1_offset:reld_core_tag_tm1_offset + 5 - 1]), .scout(sov_1[reld_core_tag_tm1_offset:reld_core_tag_tm1_offset + 5 - 1]),
         .din(reld_core_tag_tm1_d), .dout(reld_core_tag_tm1_q)
      );

      tri_rlmreg_p #(.WIDTH(2), .INIT(0), .NEEDS_SRESET(1)) reld_qw_tm1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_qw_tm1_offset:reld_qw_tm1_offset + 2 - 1]), .scout(sov_1[reld_qw_tm1_offset:reld_qw_tm1_offset + 2 - 1]),
         .din(reld_qw_tm1_d), .dout(reld_qw_tm1_q)
      );

      tri_rlmreg_p #(.WIDTH(5), .INIT(0), .NEEDS_SRESET(1)) reld_core_tag_t_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_core_tag_t_offset:reld_core_tag_t_offset + 5 - 1]), .scout(sov_1[reld_core_tag_t_offset:reld_core_tag_t_offset + 5 - 1]),
         .din(reld_core_tag_t_d), .dout(reld_core_tag_t_q)
      );

      tri_rlmreg_p #(.WIDTH(2), .INIT(0), .NEEDS_SRESET(1)) reld_qw_t_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_qw_t_offset:reld_qw_t_offset + 2 - 1]), .scout(sov_1[reld_qw_t_offset:reld_qw_t_offset + 2 - 1]),
         .din(reld_qw_t_d), .dout(reld_qw_t_q)
      );

      tri_rlmreg_p #(.WIDTH(5), .INIT(0), .NEEDS_SRESET(1)) reld_core_tag_tp1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_core_tag_tp1_offset:reld_core_tag_tp1_offset + 5 - 1]), .scout(sov_1[reld_core_tag_tp1_offset:reld_core_tag_tp1_offset + 5 - 1]),
         .din(reld_core_tag_tp1_d), .dout(reld_core_tag_tp1_q)
      );

      tri_rlmreg_p #(.WIDTH(2), .INIT(0), .NEEDS_SRESET(1)) reld_qw_tp1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_qw_tp1_offset:reld_qw_tp1_offset + 2 - 1]), .scout(sov_1[reld_qw_tp1_offset:reld_qw_tp1_offset + 2 - 1]),
         .din(reld_qw_tp1_d), .dout(reld_qw_tp1_q)
      );

      tri_rlmreg_p #(.WIDTH(5), .INIT(0), .NEEDS_SRESET(1)) reld_core_tag_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_core_tag_tp2_offset:reld_core_tag_tp2_offset + 5 - 1]), .scout(sov_1[reld_core_tag_tp2_offset:reld_core_tag_tp2_offset + 5 - 1]),
         .din(reld_core_tag_tp2_d), .dout(reld_core_tag_tp2_q)
      );

      tri_rlmreg_p #(.WIDTH(2), .INIT(0), .NEEDS_SRESET(1)) reld_qw_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_qw_tp2_offset:reld_qw_tp2_offset + 2 - 1]), .scout(sov_1[reld_qw_tp2_offset:reld_qw_tp2_offset + 2 - 1]),
         .din(reld_qw_tp2_d), .dout(reld_qw_tp2_q)
      );

      tri_rlmreg_p #(.WIDTH(128), .INIT(0), .NEEDS_SRESET(1)) reld_data_tp1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_data_tp1_offset:reld_data_tp1_offset + 128 - 1]), .scout(sov_1[reld_data_tp1_offset:reld_data_tp1_offset + 128 - 1]),
         .din(reld_data_tp1_d), .dout(reld_data_tp1_q)
      );

      tri_rlmreg_p #(.WIDTH(128), .INIT(0), .NEEDS_SRESET(1)) reld_data_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_data_tp2_offset:reld_data_tp2_offset + 128 - 1]), .scout(sov_1[reld_data_tp2_offset:reld_data_tp2_offset + 128 - 1]),
         .din(reld_data_tp2_d), .dout(reld_data_tp2_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) ptb_valid_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[ptb_valid_offset]), .scout(sov_1[ptb_valid_offset]),
         .din(ptb_valid_d), .dout(ptb_valid_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) pt0_valid_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[pt0_valid_offset]), .scout(sov_1[pt0_valid_offset]),
         .din(pt0_valid_d), .dout(pt0_valid_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) pt3_valid_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[pt3_valid_offset]), .scout(sov_1[pt3_valid_offset]),
         .din(pt3_valid_d), .dout(pt3_valid_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) rtw_arb_ptr_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_arb_ptr_offset]), .scout(sov_1[rtw_arb_ptr_offset]),
         .din(rtw_arb_ptr_d), .dout(rtw_arb_ptr_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) rtw_arb_armed_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[rtw_arb_armed_offset]), .scout(sov_1[rtw_arb_armed_offset]),
         .din(rtw_arb_armed_d), .dout(rtw_arb_armed_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reload_ptr_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(rtw_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reload_ptr_offset]), .scout(sov_1[reload_ptr_offset]),
         .din(reload_ptr_d), .dout(reload_ptr_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_crit_qw_tm1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_crit_qw_tm1_offset]), .scout(sov_1[reld_crit_qw_tm1_offset]),
         .din(reld_crit_qw_tm1_d), .dout(reld_crit_qw_tm1_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_ditc_tm1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_ditc_tm1_offset]), .scout(sov_1[reld_ditc_tm1_offset]),
         .din(reld_ditc_tm1_d), .dout(reld_ditc_tm1_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_data_vld_tm1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_data_vld_tm1_offset]), .scout(sov_1[reld_data_vld_tm1_offset]),
         .din(reld_data_vld_tm1_d), .dout(reld_data_vld_tm1_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_crit_qw_t_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_crit_qw_t_offset]), .scout(sov_1[reld_crit_qw_t_offset]),
         .din(reld_crit_qw_t_d), .dout(reld_crit_qw_t_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_ditc_t_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_ditc_t_offset]), .scout(sov_1[reld_ditc_t_offset]),
         .din(reld_ditc_t_d), .dout(reld_ditc_t_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_data_vld_t_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_data_vld_t_offset]), .scout(sov_1[reld_data_vld_t_offset]),
         .din(reld_data_vld_t_d), .dout(reld_data_vld_t_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_crit_qw_tp1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_crit_qw_tp1_offset]), .scout(sov_1[reld_crit_qw_tp1_offset]),
         .din(reld_crit_qw_tp1_d), .dout(reld_crit_qw_tp1_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_ditc_tp1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_ditc_tp1_offset]), .scout(sov_1[reld_ditc_tp1_offset]),
         .din(reld_ditc_tp1_d), .dout(reld_ditc_tp1_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_data_vld_tp1_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_data_vld_tp1_offset]), .scout(sov_1[reld_data_vld_tp1_offset]),
         .din(reld_data_vld_tp1_d), .dout(reld_data_vld_tp1_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_crit_qw_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_crit_qw_tp2_offset]), .scout(sov_1[reld_crit_qw_tp2_offset]),
         .din(reld_crit_qw_tp2_d), .dout(reld_crit_qw_tp2_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_ditc_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_ditc_tp2_offset]), .scout(sov_1[reld_ditc_tp2_offset]),
         .din(reld_ditc_tp2_d), .dout(reld_ditc_tp2_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_data_vld_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_data_vld_tp2_offset]), .scout(sov_1[reld_data_vld_tp2_offset]),
         .din(reld_data_vld_tp2_d), .dout(reld_data_vld_tp2_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_ecc_err_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_ecc_err_tp2_offset]), .scout(sov_1[reld_ecc_err_tp2_offset]),
         .din(reld_ecc_err_tp2_d), .dout(reld_ecc_err_tp2_q)
      );

      tri_rlmlatch_p #(.INIT(0), .NEEDS_SRESET(1)) reld_ecc_err_ue_tp2_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(reld_act),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[reld_ecc_err_ue_tp2_offset]), .scout(sov_1[reld_ecc_err_ue_tp2_offset]),
         .din(reld_ecc_err_ue_tp2_d), .dout(reld_ecc_err_ue_tp2_q)
      );

      tri_rlmreg_p #(.WIDTH(16), .INIT(0), .NEEDS_SRESET(1)) spare_b_latch(
         .vd(vdd), .gd(gnd), .nclk(nclk), .act(1'b0),
         .thold_b(pc_func_slp_sl_thold_0_b), .sg(pc_sg_0), .force_t(pc_func_slp_sl_force),
         .delay_lclkr(lcb_delay_lclkr_dc[0]), .mpw1_b(lcb_mpw1_dc_b[0]), .mpw2_b(lcb_mpw2_dc_b),
         .d_mode(lcb_d_mode_dc),
         .scin(siv_1[spare_b_offset:spare_b_offset + 16 - 1]), .scout(sov_1[spare_b_offset:spare_b_offset + 16 - 1]),
         .din(spare_b_q), .dout(spare_b_q)
      );

      //------------------------------------------------
      // thold/sg latches
      //------------------------------------------------

      tri_plat #(.WIDTH(3)) perv_2to1_reg(
         .vd(vdd), .gd(gnd), .nclk(nclk), .flush(tc_ccflush_dc),
         .din( {pc_func_sl_thold_2, pc_func_slp_sl_thold_2, pc_sg_2} ),
         .q(   {pc_func_sl_thold_1, pc_func_slp_sl_thold_1, pc_sg_1} )
      );

      tri_plat #(.WIDTH(3)) perv_1to0_reg(
         .vd(vdd), .gd(gnd), .nclk(nclk), .flush(tc_ccflush_dc),
         .din( {pc_func_sl_thold_1, pc_func_slp_sl_thold_1, pc_sg_1} ),
         .q(   {pc_func_sl_thold_0, pc_func_slp_sl_thold_0, pc_sg_0} )
      );

      tri_lcbor  perv_lcbor_func_sl(
         .clkoff_b(lcb_clkoff_dc_b), .thold(pc_func_sl_thold_0), .sg(pc_sg_0),
         .act_dis(lcb_act_dis_dc), .force_t(pc_func_sl_force), .thold_b(pc_func_sl_thold_0_b)
      );

      tri_lcbor  perv_lcbor_func_slp_sl(
         .clkoff_b(lcb_clkoff_dc_b), .thold(pc_func_slp_sl_thold_0), .sg(pc_sg_0),
         .act_dis(lcb_act_dis_dc), .force_t(pc_func_slp_sl_force), .thold_b(pc_func_slp_sl_thold_0_b)
      );

      //---------------------------------------------------------------------
      // Scan
      //---------------------------------------------------------------------
      assign siv_0[0:scan_right_0] = {sov_0[1:scan_right_0], ac_func_scan_in[0]};
      assign ac_func_scan_out[0]   = sov_0[0];
      assign siv_1[0:scan_right_1] = {sov_1[1:scan_right_1], ac_func_scan_in[1]};
      assign ac_func_scan_out[1]   = sov_1[0];

      assign unused_dc = {tc_scan_dis_dc_b, tc_scan_diag_dc, tc_lbist_en_dc,
                          lcb_delay_lclkr_dc[1:4], lcb_mpw1_dc_b[1:4],
                          tlb_delayed_act, tlb_ctl_tag2_flush[0], tlb_ctl_tag3_flush[0],
                          tlb_ctl_tag4_flush[0], tlb_tag2[0], tlb_tag5_except[0],
                          ctx_way_q[0][`TLB_WORD_WIDTH],
                          ctx_way_q[1][`TLB_WORD_WIDTH], ctx_qwbeat_q[0][0],
                          ctx_qwbeat_q[1][0], ctx_err_q[0][0], ctx_err_q[1][0],
                          spare_a_q[0], spare_b_q[0],
                          pc_func_sl_thold_0_b, pc_func_sl_force, 1'b0};

endmodule