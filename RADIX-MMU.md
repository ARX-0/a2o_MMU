# Radix page-table walk for the A2O MMU — branch `a2o_MMU`

This branch adds a **Power ISA 3.1C radix multi-level page-table walker** to the OpenPOWER
A2O core, ported from Microwatt's `mmu.vhdl`. A2O is the out-of-order POWER processor core
released by IBM through the OpenPOWER Foundation; an MMU (Memory Management Unit) is the
hardware that turns the addresses a program uses into the addresses memory actually has.

It is cut directly from upstream `master` and touches nothing unrelated, so

```bash
git diff master..a2o_MMU
```

**is** the change set, with no filtering required:

| Measured over | Files | Lines |
|---|---:|---|
| Everything on the branch | 19 | +5046 / −48 |
| RTL and testbenches (what an upstream PR would carry) | 9 | +2883 / −48 |
| RTL alone (`rel/src/verilog/work/`) | 6 | +2359 / −48 |

The difference is documentation and two diagram images.

---

## Why

A2O implements Power ISA 2.07 with a Book-III-E embedded MMU. Book III-E is the *embedded*
variant of the Power ISA's supervisor book; it defines a different translation mechanism
from the server variant that ISA 3.0 onwards specifies. A2O's hardware tablewalker
(`mmq_htw.v`) performs a **single-level** walk: software installs an indirect entry in the
TLB (Translation Lookaside Buffer, the on-chip cache of translations) whose RPN (Real Page
Number) field is the base of a flat page-table array, and the walker issues exactly one
8-byte load. There is no root-pointer register, no level counter, and no multi-level descent
anywhere in the design. A2O's own release notes name radix translation as the first
requirement for ISA 3.0c/3.1 compliance.

The existing Book-E walker is **left completely untouched and fully functional**. A boot
configuration bit (`tlb0cfg_radix`) selects between the two, and it defaults to *off* — a
core built from this branch behaves exactly as upstream unless the bit is deliberately set.

## What changed

| File | Lines | What |
|---|---:|---|
| `rel/src/verilog/work/mmq_rtw.v` | +1979 | **New** — the radix walker |
| `rel/src/verilog/work/mmq.v` | +210 −18 | Instantiation, walker mux, exception merge |
| `rel/src/verilog/work/mmq_spr.v` | +101 −15 | PTCR (SPR 464), invalidate strobes, radix enable |
| `rel/src/verilog/work/mmq_tlb_cmp.v` | +27 −1 | Walker handoff, TLB-miss suppression |
| `rel/src/verilog/work/mmu_a2o.vh` | +23 | Radix PDE/PTE field definitions |
| `rel/src/verilog/work/xu_spr_cspr.v` | +19 −14 | PTCR decode and privilege qualification |
| `rel/src/verilog/sim/` | +524 | Two testbenches and a run script |
| `rel/doc/radix-mmu/` | — | This documentation set |

Exactly **one** new SPR (Special Purpose Register — an architected control register read and
written by `mfspr`/`mtspr`) number is claimed: PTCR (Partition Table Control Register) = 464,
the ISA 3.1C number, which is free in A2O. The nine places where Microwatt's SPR numbering
collides with A2O's are listed in
[04-integration](rel/doc/radix-mmu/04-integration.md#44-spr-work); none of them blocks this
port.

## How to read it

The six commits are layered so each answers one question on its own, in dependency order:

```bash
git log --oneline master..a2o_MMU
```

1. `mmu_a2o.vh` — radix field definitions (inert)
2. `mmq_rtw.v` — the walker (new module, not yet instantiated)
3. `mmq_spr.v`, `xu_spr_cspr.v` — PTCR and the enable bit (still inert)
4. `mmq.v`, `mmq_tlb_cmp.v` — **the commit that turns it on**
5. `sim/` — testbenches
6. `rel/doc/radix-mmu/` — documentation

Commits 1–5 are the RTL change; commit 6 is documentation only and can be dropped without
affecting the design.

## Reproducing the test results

```bash
rel/src/verilog/sim/run_rtw_tests.sh     # ~5 seconds; needs verilator and iverilog
```

This runs a verilator lint of `mmq_rtw`, then 400 random vectors comparing the ported bit
manipulation against a direct model of Microwatt's `mmu.vhdl`, then 11 end-to-end walk
scenarios against a behavioural L2 and a real radix tree in memory. All pass.

To confirm the integration introduced no new lint errors, compare against pristine upstream:

```bash
git worktree add /tmp/a2o-upstream master
cd /tmp/a2o-upstream/rel/src/verilog && verilator --lint-only --language 1364-2005 \
    -Wno-fatal -Wno-LITENDIAN -Wno-DECLFILENAME -Wno-TIMESCALEMOD \
    -Itrilib -Iwork work/mmq.v --top-module mmq 2>&1 | grep -c '^%Error'   # 3
# same command in this tree's rel/src/verilog                              # 3
git worktree remove /tmp/a2o-upstream
```

Both report three, all pre-existing references to Xilinx `RAMB16` primitives that are not in
the source tree.

## What this does *not* cover

The gaps, stated up front:

- **No whole-core simulation.** All testing is `mmq_rtw` in isolation against a behavioural
  L2. The handoff from `mmq_tlb_cmp`, the walker mux in `mmq.v`, and the exception merge are
  lint-clean but **have not been simulated**.
- **No FPGA run** — no synthesis or timing-closure results.
- **Guest-mode (`MSR[GS]=1`, the Machine State Register's guest-state bit) walks are
  deliberately refused**, not implemented. They raise `lrat_miss`. The LRAT (Logical to Real
  Address Translation) is A2O's hypervisor-level translation array; validating every level of
  a guest walk through it needs a second compare port that does not exist yet. This fails
  closed — the alternative would be issuing memory requests to guest-supplied addresses
  without validation.
- **No hardware reference/change bit writeback.** The R (referenced) and C (changed) bits in
  a page-table entry record that a page has been read or written. This walker checks them but
  never sets them, so software must. Microwatt behaves the same way.
- **Leaf-size demotion is untested.** Radix produces 2 MB leaves; A2O has no encoding for that
  size, so leaves are installed at the largest representable sub-page instead. The safety
  argument is structural rather than cited, and no testbench scenario builds a 2 MB leaf and
  checks the demoted entry. This is the most significant untested behaviour in the port.
- **No page-walk cache.** Cold-walk latency is four to six serial L2 round trips.

The full list, with reasoning, is in
[rel/doc/radix-mmu/00-README.md](rel/doc/radix-mmu/00-README.md#status-and-limitations).

## Full documentation

**[rel/doc/radix-mmu/](rel/doc/radix-mmu/00-README.md)** — seven documents, written to be
read without prior familiarity with A2O, Book-E, or radix translation:

| | |
|---|---|
| [00-README](rel/doc/radix-mmu/00-README.md) | Executive summary, status and limitations |
| [01-background](rel/doc/radix-mmu/01-background.md) | Why A2O needs this; what Book-E does instead |
| [02-fsm](rel/doc/radix-mmu/02-fsm.md) | The state machine, cold walk and warm walk |
| [03-datapath](rel/doc/radix-mmu/03-datapath.md) | Bit order, address shifting, leaf size demotion |
| [04-integration](rel/doc/radix-mmu/04-integration.md) | SPR work, the mux, exception routing |
| [05-ooo-safety](rel/doc/radix-mmu/05-ooo-safety.md) | Five hazard classes in an out-of-order core |
| [06-verification](rel/doc/radix-mmu/06-verification.md) | What was tested, how, and what was not |

Reference implementation: Microwatt's
[`mmu.vhdl`](https://github.com/antonblanchard/microwatt/blob/master/mmu.vhdl).
