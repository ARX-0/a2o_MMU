# Radix Page-Table Walk for the IBM A2O MMU

Documentation set for `mmq_rtw.v` — a Power ISA 3.1C radix multi-level page-table walker
added to the OpenPOWER A2O core, ported from Microwatt's `mmu.vhdl`.

This page assumes no prior familiarity with A2O, Book-E, or radix translation. Every
abbreviation is expanded where it first appears, and collected in the
[glossary](#glossary) at the end.

---

## Executive summary

**The problem.** The A2O core is an out-of-order POWER processor released by IBM through the
OpenPOWER Foundation. It implements Power ISA 2.07 using Book III-E — the *embedded* variant
of the Power ISA's supervisor book, which specifies a different address-translation mechanism
from the server variant used by ISA 3.0 onwards. A2O's own release notes name *radix
translation* as the first thing required for ISA 3.0c/3.1 compliance.

Its MMU (Memory Management Unit — the hardware that turns the addresses a program uses into
the addresses memory actually has) is a Book-E embedded MMU whose hardware tablewalker
performs a **single-level** walk: software installs an indirect entry in the TLB (Translation
Lookaside Buffer, the on-chip cache of recently used translations) whose RPN (Real Page
Number) field is the base of a flat page-table array, and the walker issues exactly one
8-byte load. There is no root-pointer register, no level counter, and no multi-level descent
anywhere in the design.

**What was built.** A new module, `mmq_rtw.v`, implementing the full radix tree walk —
`PTCR` (Partition Table Control Register, a Special Purpose Register holding the root of the
whole structure) → partition table → process table → up to four levels of page-directory
descent → leaf PTE (Page Table Entry, the record that finally supplies a real address and its
access permissions) — transcribed from Microwatt's `mmu.vhdl`, and adapted to A2O's
out-of-order, two-threaded environment. It sits beside the existing Book-E walker
(`mmq_htw.v`), which is left untouched and fully functional; a boot-configuration bit selects
between them.

**Scale of the change.**

| File | Lines added | What changed |
|---|---:|---|
| `work/mmq_rtw.v` | 1979 | **New** — the radix walker |
| `work/mmq.v` | 210 | Instantiation, walker mux, exception merge |
| `work/mmq_spr.v` | 101 | PTCR (SPR 464), invalidate strobes, radix enable bit |
| `work/mmq_tlb_cmp.v` | 27 | Walker handoff, TLB-miss suppression |
| `work/mmu_a2o.vh` | 23 | Radix PDE/PTE field definitions |
| `work/xu_spr_cspr.v` | 19 | PTCR decode and privilege qualification |
| `sim/` | 524 | Two testbenches and a run script |

**Verification status.** 400 random vectors comparing the ported bit manipulation against a
direct model of the Microwatt source, plus 11 end-to-end walk scenarios against a
behavioural L2 and a real radix tree in memory. All pass. The `mmq` hierarchy lints with the
same error count as pristine upstream A2O.

### Three structural findings

1. **A2O cannot represent a 2 MB page.** Its TLB size field encodes log₄(size/1 KB), so only
   sizes that are an integer power of four times 1 KB have an encoding at all. 2 MB is 2¹¹ ×
   1 KB, and log₄ of that is 5.5 — there is no encoding for it. The page-table-reload datapath
   then keeps only three of those bits, capping the size reachable through that path at 16 MB.
   Radix produces 4 K/64 K/2 M/1 G leaves. Leaves are therefore installed at the largest
   representable sub-page size. The argument that this is safe is structural — a smaller page
   maps a subset of the same translation under the same permission bits — but it rests on
   reasoning rather than an ISA citation, and **the demotion path is not covered by any
   testbench**. See [03-datapath](03-datapath.md#36-leaf-size-demotion).

2. **Porting into an out-of-order core is the hard part, not the algorithm.** Microwatt
   dispatches TLB invalidations through the *same* state machine as a walk, so a walk and an
   invalidate are mutually exclusive by construction. A2O has neither that serialisation nor
   any mechanism to abort a walk — its flush window is about five cycles, while a radix walk
   is roughly a hundred times longer. Five distinct hazard classes had to be addressed before
   the walker was safe. See [05-ooo-safety](05-ooo-safety.md).

3. **The Microwatt and A2O SPR spaces collide in nine places** — A2O's Book-E debug (304–319),
   timer (336–343) and MMU-control (1012–1023) register blocks occupy exactly the numbers ISA
   3.x assigns to its hypervisor registers, HEIR and PIR. None of the collisions block this
   port: exactly one new SPR number is claimed, PTCR = 464, and it is free.
   See [04-integration](04-integration.md#44-spr-work) for the register-by-register table.

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

What works, what was refused on purpose, and what is simply not done yet. These are three
different things and are kept under three different headings.

### Working

- Radix walk in **hypervisor state**, 1–4 levels, cold and warm (cached roots).
- All five architected fault classes detected and reported: non-present PTE, malformed tree,
  segment/quadrant error, reference/change violation, and machine check.
- Book-E path **unaffected** — with the radix enable clear, the walker is inert and A2O
  behaves exactly as before.
- Out-of-order safety: flush mid-walk, invalidate mid-walk, watchdog timeout, and bounded
  ECC (Error Correcting Code) retry are all implemented and tested.

### Deliberately refused

- **Guest-mode radix walks fault rather than proceed.** Guest mode is `MSR[GS]=1` — the
  guest-state bit of the Machine State Register, set when the core is running a virtualised
  guest rather than the hypervisor. In guest mode every page-table address after the root is
  read out of guest-writable memory and must be validated through the LRAT (Logical to Real
  Address Translation, A2O's hypervisor-level translation array) before leaving the core.
  A2O's LRAT (`mmq_tlb_lrat.v`) is a
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
- **`mmq_inval.v` re-verification.** The invalidate sequencer contains eight deliberate
  "service the tablewalker or deadlock" detours — four commented "could hang waiting on
  empty" (`mmq_inval.v:1037, 1133, 1236, 1338`) and four commented "could be ucode"
  (`:1019, 1102, 1219, 1324`). A multi-level walker exercises them four to five times harder.
  This has not been re-examined.
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

A one-page summary of the branch, for a reader arriving from the repository root, is in
[RADIX-MMU.md](../../../RADIX-MMU.md).

---

## Glossary

Every term is also expanded where it first appears in the text. This table is for looking one
up again later.

| Term | Expansion | What it is here |
|---|---|---|
| **A2O** | — | The out-of-order POWER processor core this work modifies |
| **Book-E / Book III-E** | — | The *embedded* variant of the Power ISA supervisor book. A2O implements it; ISA 3.0 onward specifies the server variant instead |
| **ECC** | Error Correcting Code | Detects and corrects bit errors in memory. An uncorrectable one on a table read is a fault |
| **EMQ** | ERAT Miss Queue | Four entries in the load/store unit, one per outstanding translation miss. Freed only by a returning reload |
| **ERAT** | Effective-to-Real Address Translation | The small per-thread translation cache next to the pipeline, ahead of the TLB |
| **E.PT** | — | The Book-E category defining the indirect-TLB-entry page-table scheme A2O already had |
| **LPID** | Logical Partition Identifier | Names the partition (guest) a translation belongs to |
| **LRAT** | Logical to Real Address Translation | A2O's 8-entry hypervisor-level translation array |
| **LSU** | Load/Store Unit | Issues the walker's memory requests; source of the single credit token |
| **MMU** | Memory Management Unit | The hardware translating program addresses to memory addresses |
| **MSR** | Machine State Register | Architected processor state. `MSR[GS]` is the guest-state bit |
| **NLS** | Next Level Size | Index width of the level below, carried in each PDE |
| **PDE** | Page Directory Entry | A non-leaf tree entry; supplies the base of the next level |
| **PID** | Process Identifier | Selects a process's entry in the process table |
| **PTCR** | Partition Table Control Register | SPR 464; the root of the whole structure |
| **PTE** | Page Table Entry | A leaf entry; supplies a real page number and access permissions |
| **R / C bits** | Referenced / Changed | Record that a page has been read or written. This walker checks them, never sets them |
| **RPDS** | Root Page Directory Size | Index width of the tree's first level |
| **RPN** | Real Page Number | The physical page a translation resolves to |
| **RTS** | Radix Tree Size | Sets the address-space width, as `RTS + 31` bits |
| **SPR** | Special Purpose Register | An architected control register, accessed by `mfspr` / `mtspr` |
| **TLB** | Translation Lookaside Buffer | The core's 512-entry, 4-way shared translation cache, behind the ERATs |
