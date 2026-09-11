# Radix page-table walk for the A2O MMU — branch `a2o_MMU`

This branch adds a **Power ISA 3.1C radix multi-level page-table walker** to the OpenPOWER
A2O core, ported from Microwatt's `mmu.vhdl`.

It is cut directly from upstream `master` and touches nothing unrelated, so

```bash
git diff master..a2o_MMU
```

**is** the change set — 10 files, roughly 2 900 added lines, and no filtering required.

---

## Why

A2O implements Power ISA 2.07 with a Book-III-E embedded MMU. Its hardware tablewalker
(`mmq_htw.v`) performs a **single-level** walk: software installs an indirect TLB entry
whose RPN field is the base of a flat page-table array, and the walker issues exactly one
8-byte load. There is no root-pointer register, no level counter, and no multi-level descent
anywhere in the design. A2O's own release notes name radix translation as the first
requirement for ISA 3.0c/3.1 compliance.

The existing Book-E walker is **left completely untouched and fully functional**. A boot
configuration bit (`tlb0cfg_radix`) selects between the two, and it defaults to *off* — a
core built from this branch behaves exactly as upstream unless the bit is deliberately set.

## What changed

| File | Lines | What |
|---|---:|---|
| `rel/src/verilog/work/mmq_rtw.v` | +1978 | **New** — the radix walker |
| `rel/src/verilog/work/mmq.v` | +210 −18 | Instantiation, walker mux, exception merge |
| `rel/src/verilog/work/mmq_spr.v` | +99 −15 | PTCR (SPR 464), invalidate strobes, radix enable |
| `rel/src/verilog/work/mmq_tlb_cmp.v` | +26 −1 | Walker handoff, TLB-miss suppression |
| `rel/src/verilog/work/mmu_a2o.vh` | +23 | Radix PDE/PTE field definitions |
| `rel/src/verilog/work/xu_spr_cspr.v` | +19 −14 | PTCR decode and privilege qualification |
| `rel/src/verilog/sim/` | +524 | Two testbenches and a run script |
| `rel/doc/radix-mmu/` | — | This documentation set |

Exactly **one** new SPR number is claimed: PTCR = 464, the ISA 3.1C number, which is free in
A2O.

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

Stated up front, because the boundaries matter as much as the achievements:

- **No whole-core simulation.** All testing is `mmq_rtw` in isolation against a behavioural
  L2. The handoff from `mmq_tlb_cmp`, the walker mux in `mmq.v`, and the exception merge are
  lint-clean but **have not been simulated**.
- **No FPGA run** — no synthesis or timing-closure results.
- **Guest-mode (`MSR[GS]=1`) walks are deliberately refused**, not implemented. They raise
  `lrat_miss`, because per-level LRAT validation needs a second LRAT compare port that does
  not exist yet. This fails closed.
- **No hardware reference/change bit writeback.** Software must set R and C. Microwatt
  behaves the same way.
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
