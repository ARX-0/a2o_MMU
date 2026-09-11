# Radix Page-Table Walk for the IBM A2O MMU

Documentation set for `mmq_rtw.v` — a Power ISA 3.1C radix multi-level page-table walker
added to the OpenPOWER A2O core, ported from Microwatt's `mmu.vhdl`.

---

## Executive summary

**The problem.** The A2O core implements Power ISA 2.07 using Book III-E. Its own release
notes name *radix translation* as the first thing required for ISA 3.0c/3.1 compliance. Its
MMU is a Book-E embedded MMU whose hardware tablewalker performs a **single-level** walk:
software installs an indirect TLB entry whose RPN field is the base of a flat page-table
array, and the walker issues exactly one 8-byte load. There is no root-pointer register, no
level counter, and no multi-level descent anywhere in the design.

**What was built.** A new module, `mmq_rtw.v`, implementing the full radix tree walk —
`PTCR` → partition table → process table → up to four levels of page-directory descent →
leaf PTE — transcribed from Microwatt's `mmu.vhdl`, and adapted to A2O's out-of-order,
two-threaded environment. It sits beside the existing Book-E walker (`mmq_htw.v`), which is
left untouched and fully functional; a boot-configuration bit selects between them.

**Scale of the change.**

| File | Lines added | What changed |
|---|---:|---|
| `work/mmq_rtw.v` | 1978 | **New** — the radix walker |
| `work/mmq.v` | 210 | Instantiation, walker mux, exception merge |
| `work/mmq_spr.v` | 99 | PTCR (SPR 464), invalidate strobes, radix enable bit |
| `work/mmq_tlb_cmp.v` | 26 | Walker handoff, TLB-miss suppression |
| `work/mmu_a2o.vh` | 23 | Radix PDE/PTE field definitions |
| `work/xu_spr_cspr.v` | 19 | PTCR decode and privilege qualification |
| `sim/` | 524 | Two testbenches and a run script |

**Verification status.** 400 random vectors comparing the ported bit manipulation against a
direct model of the Microwatt source, plus 11 end-to-end walk scenarios against a
behavioural L2 and a real radix tree in memory. All pass. The `mmq` hierarchy lints with the
same error count as pristine upstream A2O.

### Three findings worth reporting

1. **A2O cannot represent a 2 MB page.** Its TLB size field encodes log₄(size/1 KB), so only
   power-of-four sizes exist, and the page-table-reload datapath keeps only three of those
   bits, capping the reachable size at 16 MB. Radix produces 4 K/64 K/2 M/1 G. Leaves are
   therefore installed at the largest representable sub-page size, which is always
   architecturally safe. See [03-datapath](03-datapath.md#36-leaf-size-demotion).

2. **Porting into an out-of-order core is the hard part, not the algorithm.** Microwatt
   dispatches TLB invalidations through the *same* state machine as a walk, so a walk and an
   invalidate are mutually exclusive by construction. A2O has neither that serialisation nor
   any mechanism to abort a walk — its flush window is about five cycles, while a radix walk
   is roughly a hundred times longer. Five distinct hazard classes had to be addressed before
   the walker was safe. See [05-ooo-safety](05-ooo-safety.md).

3. **The Microwatt and A2O SPR spaces collide in nine places** — A2O's Book-E debug, timer
   and MMU-control register blocks occupy exactly the numbers ISA 3.x assigns to its
   hypervisor registers, HEIR and PIR. None of the collisions block this port: exactly one
   new SPR number is claimed, PTCR = 464, and it is free.
   See [04-integration](04-integration.md#44-spr-work).

---

## Contents

| # | Document | Covers |
|---|---|---|
| 01 | [Background and problem statement](01-background.md) | Radix translation, what A2O had, what Microwatt has, structural comparison |
| 02 | [The state machine](02-fsm.md) | 12 states, encodings, transitions, flow of events for a walk |
| 03 | [Datapath and bit manipulation](03-datapath.md) | Bit-order convention, shifter, masks, address formation, size demotion |
| 04 | [Integration into the core](04-integration.md) | Module wiring, enable bit, SPRs, exception mapping |
| 05 | [Out-of-order safety](05-ooo-safety.md) | Why an OoO core changes the design; the five P0 hazards |
| 06 | [Verification](06-verification.md) | Testbenches, scenarios, bugs found, how to reproduce |

Suggested reading order for a first pass: **00 → 01 → 02 → 05**. Documents 03, 04 and 06 are
reference depth.

---

## Status and limitations

Stated explicitly, because the boundaries matter as much as the achievements.

### Working

- Radix walk in **hypervisor state**, 1–4 levels, cold and warm (cached roots).
- All five architected fault classes detected and reported: non-present PTE, malformed tree,
  segment/quadrant error, reference/change violation, and machine check.
- Book-E path **unaffected** — with the radix enable clear, the walker is inert and A2O
  behaves exactly as before.
- Out-of-order safety: flush mid-walk, invalidate mid-walk, watchdog timeout, and bounded
  ECC retry are all implemented and tested.

### Deliberately refused

- **Guest-mode (`MSR[GS]=1`) radix walks fault rather than proceed.** In guest mode every
  page-table address after the root is read out of guest-writable memory and must be
  validated through the LRAT before leaving the core. A2O's LRAT (`mmq_tlb_lrat.v`) is a
  *pipelined* lookup driven from the TLB tag pipeline, not a standalone request port, so a
  per-level check needs a second compare port rather than a wire. Until that exists the
  walker's LRAT-hit input is tied low, so guest walks raise `lrat_miss`. This fails closed,
  not open — the alternative would be issuing memory requests to addresses supplied by a
  guest without validation.

### Not yet done

- **Hardware reference/change bit updates.** The walker checks R and C but never writes them
  back; software must set them. This is not an oversight — a hardware R/C store would be an
  architecturally visible memory write issued on behalf of an instruction that has not
  committed, and A2O has no mechanism to defer it. Microwatt behaves the same way. See
  [05-ooo-safety](05-ooo-safety.md#p0-2--rc-writeback).
- **`mmq_inval.v` re-verification.** The invalidate sequencer contains six deliberate
  "service the tablewalker or deadlock" detours. A multi-level walker exercises them four to
  five times harder. This has not been re-examined.
- **Invalidate matching is conservative.** Any TLB invalidate snoop currently discards every
  in-flight walk. Correct, but coarser than necessary.
- **No whole-core or FPGA simulation.** All testing is at module level. A full `mmq`
  simulation requires Xilinx `RAMB16` primitives that are not present in the source tree.
- **No page-walk cache.** Microwatt caches intermediate directory entries; this port does
  not. Cold-walk latency is four to six serial L2 round trips.

---

## Source layout

```
rel/src/verilog/work/mmq_rtw.v     the radix walker
rel/src/verilog/work/mmq*.v        the rest of the A2O MMU
rel/src/verilog/sim/               testbenches + run_rtw_tests.sh
rel/doc/radix-mmu/                 this documentation set
```

The reference implementation is Microwatt's `mmu.vhdl`, which is not vendored here:
<https://github.com/antonblanchard/microwatt/blob/master/mmu.vhdl>.

To reproduce the test results:

```bash
rel/src/verilog/sim/run_rtw_tests.sh   # lint + both benches, ~5 seconds
```

To see every line this work changed, relative to pristine upstream A2O:

```bash
git diff --stat master..a2o_MMU    # per-file counts
git diff master..a2o_MMU           # the whole change set
git log --oneline master..a2o_MMU  # the six commits, in dependency order
```

Because this branch is cut directly from upstream `master` and touches nothing else, that
diff **is** the port -- there is no unrelated content to filter out.
