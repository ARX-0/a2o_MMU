# 05 — Out-of-order safety

[← 04 Integration](04-integration.md) · [Index](00-README.md) · Next: [06 — Verification](06-verification.md)

Transcribing the radix algorithm was the straightforward part of this project. Making it
safe in an out-of-order, two-threaded core was not. This document explains why, and what the
walker does about it.

---

## 5.1 The assumption that does not survive the port

Microwatt dispatches TLB invalidations **through the same state machine as a walk**:

```vhdl
-- microwatt/mmu.vhdl:1524-1525
v.tlbie_req := '1';
v.state := DO_TLBIE;
```

Because the MMU is in `DO_TLBIE`, it structurally cannot simultaneously be in
`RADIX_READ_WAIT`. A walk and an invalidate are mutually exclusive by construction. One
core, one state machine, one outstanding request, no races — the in-order single-threaded
design does the ordering for free.

**None of that holds in A2O.** It has two threads, out-of-order issue with register
renaming, a completion buffer, a store queue, a separate invalidate sequencer, and a shared
load/store port. Every hazard below follows from losing that one assumption.

The existing Book-E walker gets away with ignoring most of these because its walk is a
*single load* — it is over before most hazard windows even open. A radix walk lasts roughly
a hundred times longer.

## 5.2 A2O's two contract rules

Reading the existing MMU establishes two invariants that any new walker must preserve.

### Rule 1 — never leave the core for a speculative request

The `nonspec` bit gates the walk handoff absolutely (`mmq_tlb_cmp.v:5071`) and gates even
the *search* for an indirect entry (`mmq_tlb_ctl.v:1603, 1641, 1679, 1717, 1750`).

Critically, `nonspec` does **not** mean "committed". It means *"this request belongs to the
instruction currently next-to-complete in its thread"* — the oldest un-completed instruction
(`lq_derat.v:4628-4632`). A speculative ERAT miss is probed against the TLB and then
silently dropped; the load recirculates and re-requests once it becomes oldest.

What a speculative walk would cost: TLB and ERAT pollution, replacement-policy corruption,
spurious LRAT-miss and page-fault exceptions, machine checks from walking a garbage
directory entry into a nonexistent address, a Spectre-class timing footprint — and, new to
radix, reference/change bit writes, which are architecturally visible and cannot be undone.

### Rule 2 — walks are not abortable, they are made *harmless*

A2O cannot cancel an in-flight walk. Instead three mechanisms interlock so that a stale
result writes nothing:

1. a per-request **reservation bit** (`mmq_htw.v:786-793`, eleven documented clear conditions
   at `:933-957`),
2. the **`wq == 2'b10` gate** on the TLB write (`mmq_tlb_ctl.v:2980`),
3. the **never-recycled EMQ entry** (`lq_derat.v:4503-4512`).

A stale reload still returns; it just installs nothing. A walker that lengthens the
vulnerable window by two orders of magnitude must strengthen all three legs or add genuine
abort capability. `mmq_rtw` does both.

## 5.3 The five P0 hazards

### P0-1 — the flush window is ~5 cycles; a radix walk is ~100× longer

The tag-pipeline flush signals `tlb_ctl_tag{1,2,3,4}_flush_sig` are **hard-wired to zero**
for the `derat`, `ierat`, `snoop` and `ptereload` tag types (`mmq_tlb_ctl.v:2332-2342`).
Only architected TLB-management instructions are flushable. The RTL states it outright at
`mmq_tlb_ctl.v:2057`: *"tag0 (ex2) tlbre,tlbwe (flushable), or ptereload (not flushable)"*.

Consequently `tlb_seq_abort` can never fire for a walk, and the flush accumulation chain is
only five latches deep (`:926-971`) — the window closes about five cycles after the request
enters the MMU. A single-load walk is finished before the chain even fills.

**What the walker does.** A per-context `killed` bit, set from `xu_ex5_flush` matched against
the context's thread, and tested at every state boundary:

```verilog
// mmq_rtw.v:888
assign ctx_kill_now[i] = (`MM_THREADS > i) ? xu_ex5_flush[i] : 1'b0;

// P0-1: a flush arriving mid-walk is remembered here and acted on at the
// next level boundary. It is never acted on mid-load: the L2 reload is
// already in flight and must be drained.
assign ctx_killed_d[i] = (seq_valid_set) ? 1'b0 :
                         (ctx_valid_q[i] & ctx_kill_now[i]) ? 1'b1 : ctx_killed_q[i];
```

Killing mid-load is deliberately *not* done. The reload is already in flight, will arrive
tagged, and must be consumed.

### P0-2 — R/C writeback

A2O's `WAIT_UPDATES` machinery (`mmu_a2o.vh:52`, `mmq_spr.v:1420-1497`) states the design
intent plainly: no architected state may be updated until the completion unit confirms the
interrupt was actually taken. Six per-thread pending flags hold MAS, LPER and MMUCR1 updates,
released only by a type-matched exception-taken code and cleared on flush.

That machinery covers **SPR latches only**. There is no pending-memory-write path anywhere in
the MMU, and the MMU→LSU port is load-only (`lq_imq.v:107` decodes the type as TLBIVAX,
TLBI_COMPLETE, LOAD, LOAD).

**What the walker does.** It checks R and C but never writes them; software must set them.
This matches Microwatt, which also has no write path. It is a P0 constraint rather than a
preference because "oldest in thread" is not "committed" — an older machine check or
asynchronous interrupt can still flush a `nonspec` request, and a memory write issued on its
behalf could not be undone.

The architectural consequence is that this is a **software-managed-R/C radix
implementation**, a documented deviation from ISA 3.1 where hardware normally sets them.

### P0-3 — the EMQ entry is held for the whole walk

An LSU ERAT-Miss-Queue entry leaves its pending state only on reload, block, or reset
(`lq_derat.v:4503-4512`). A pipeline flush sets a kill flag but does **not** deallocate
(`:4550-4554`) — which is exactly what makes a stale reload safe. There are four entries and
entry 0 is reserved for the oldest instruction (`:4536-4541`); that reservation is the only
anti-starvation guarantee in the whole stack.

**What the walker does.** Returns `ptereload_req_*` on **every** termination path without
exception. See [02-fsm §2.6](02-fsm.md#26-why-every-exit-goes-through-the-same-handshake).
The testbench asserts this on every fault path with an explicit "EMQ LEAK" failure.

### P0-4 — the reservation tracks only the leaf address

A2O's reservation-clear comparison is a single match on
`(lpid, pid, gs, as, sized-EPN)` (`mmq_htw.v:1096-1160`). For a one-load walk that address
*is* the walk. A radix walk touches four to six *different* memory locations, and an
invalidate arriving mid-walk may target a **directory** level whose contents have already
been consumed — invisible to a leaf-address comparison.

**What the walker does.** Clears the reservation on any matching invalidate for the
context's `(lpid, pid, gs, as)`, **deliberately not comparing EPN** (`mmq_rtw.v:892`):

```verilog
// EPN is deliberately NOT compared: a radix walk reads 4-5 different
// addresses and an invalidate may target a directory level we already
// consumed. Matches the tlbilx T=0 semantics at mmq_htw.v:963.
assign ctx_inv_match[i] = inv_seq_inprogress &
                          (inv_all |
                           ((inv_lpid == ctx_tag_q[i][`tagpos_lpid:...]) &
                            (inv_gs   == ctx_tag_q[i][`tagpos_gs]) &
                            (inv_as   == ctx_tag_q[i][`tagpos_as]) &
                            ((inv_pid == ctx_pid) | (inv_pid == {`PID_WIDTH{1'b0}}))));
```

Redoing a walk after an invalidate is correct and rare. Being clever here would be a
correctness risk for a negligible performance gain.

### P0-5 — one credit token serialises everything

The MMU holds a single LSU credit token (`mmq_inval.v:1598-1616`), shared between three
consumers: page-table loads, TLB invalidate broadcasts, and invalidate-sync completions.
The queue behind it is two entries deep, and the token is returned when the request is
*sent* to the L2, not when data returns.

A four-level walk is therefore four strictly serial L2 round trips through a port that also
carries invalidate traffic.

**What the walker does.** One load in flight per context, re-arbitrated at every level, with
two contexts sharing the port round-robin. The root cache
([02-fsm §2.5](02-fsm.md#25-the-warm-walk)) exists primarily to reduce the number of times
this bottleneck is traversed.

**Open item.** `mmq_inval.v` contains six deliberate *"service the tablewalker or we
deadlock"* detours (`:1015, 1030, 1100, 1130, 1321, 1332`), each with an explicit "could
hang" comment. They are the only intentional deadlock breakers in the MMU, and a multi-level
walker exercises them four to five times harder. **They have not been re-verified.**

## 5.4 P1 and P2 items

| # | Hazard | Response |
|---|---|---|
| P1-6 | `nonspec` is sampled **once**, at handoff. "Was oldest 300 cycles ago" is not "is oldest now". | Re-tested at every level boundary via the `killed` bit. The gate is never *widened* to hide latency. |
| P1-7 | **No timeout exists anywhere in the A2O MMU.** Every wait loop is unbounded; if the L2 never answers, the slot never frees, quiesce never asserts and the thread hangs permanently. | Per-context watchdog. See below. |
| P1-8 | ECC retry is one-shot and passive — a persistently failing line loops forever with no counter and no escalation. | Bounded retry counter, then escalation to a machine check. |
| P1-9 | The walk bypasses the D-cache, load queue and store queue entirely and sees only L2 state. | Documented: software must use the architected store-PTE → `msync` → walk sequence. No hardware interlock was added; doing so would deadlock against the invalidate sequencer without also extending the P0-5 detours. |
| P2-10 | Thread starvation — `mmq_htw` allocates its four slots round-robin, thread-agnostically. | Contexts are **statically bound** to threads. See §5.5. |
| P2-11 | Walk addresses after the root come from guest-writable memory, with no LRAT check and no bounds check. | Bounds check implemented; LRAT check **not yet possible**, so guest walks are refused. See §5.6. |

### The watchdog, and a bug worth recording

The watchdog was initially written to count only while a load was outstanding. Testing
showed that was the wrong interval: a request the LSU arbiter never *grants* hangs just as
hard as one whose data never returns, and the first version did not count that case at all.

It now measures **time since last progress** (`mmq_rtw.v:1325`):

```verilog
// It measures TIME SINCE LAST PROGRESS, not time since the load was issued:
// a request that is never granted by the LSU arbiter stalls just as hard as
// one whose data never returns, and an earlier version that counted only
// while a load was outstanding missed the former entirely.
// Terminal states are excluded -- they are waiting on ptereload_req_taken,
// and tripping there would only swap one wait for an identical one.
assign ctx_wd_run[i] = ctx_valid_q[i] &
                       (ctx_seq_q[i] != RtwSeq_Reload) & (ctx_seq_q[i] != RtwSeq_Fault) &
                       (ctx_seq_q[i] != RtwSeq_Killed) & (ctx_seq_q[i] != RtwSeq_Timeout);
```

It trips at 4096 cycles, forces `RtwSeq_Timeout`, returns a reload so the EMQ entry is freed,
and raises a machine check.

## 5.5 Two contexts, statically bound to threads

`mmq_htw` has four request slots because an E.PT slot is *parked* between its handoff and
its single load. A radix context is continuously active, and A2O provides only two L2 core
tags (`01100`, `01101`) — the only mechanism by which returning data identifies itself. Two
is therefore the real concurrency limit.

With two contexts and two threads, binding context *N* to thread *N* makes the per-thread
resource reservation **structural rather than a policy**: neither thread can deprive the
other of walk capacity, which is the strongest possible answer to P2-10 and costs nothing.

```verilog
// context N is statically bound to thread N (P2-10): the per-thread
// resource reservation is structural, so neither thread can starve the
// other of walk capacity the way they can with mmq_htw's shared slots.
assign accept = tlb_rtw_req_valid & mmucr1_rxe & (~ctx_valid_q[i]) &
                tlb_rtw_req_tag[`tagpos_thdid + i];
```

## 5.6 Guest mode: why it is refused

In hypervisor state the walk's root address comes from PTCR, a privileged register. In
**guest** state (`MSR[GS]=1`) every address after the root is read out of guest-writable
memory. Without validation, a guest could author a page-directory entry pointing at an
arbitrary real address and the MMU would fetch it.

A2O never needed this check: its single walk address comes from a hypervisor-installed
indirect TLB entry, trusted by construction. The correct mechanism is the LRAT
(`mmq_tlb_lrat.v`), which translates and validates guest real addresses.

**The problem.** The LRAT is a *pipelined* lookup driven from `tlb_tag0_*`, the TLB tag
pipeline — not a standalone request/response port. Routing a per-level walker address
through it needs a second compare port, which is a substantial change to a module outside
this port's footprint.

**The interim decision.** The walker's LRAT-hit input is tied **low**, so a guest-mode walk
raises `lrat_miss` and installs nothing:

```verilog
// P2-11.  mmq_tlb_lrat is a PIPELINED lookup driven from tlb_tag0_*, not a
// standalone request port, so a per-level walker check needs a second
// compare port rather than a wire.  Until that exists, guest-mode radix
// walks are refused outright (hit tied low -> Flt_LratMiss) instead of
// proceeding with addresses read out of guest-writable memory.  Radix in
// hypervisor state (gs=0) is unaffected and fully functional.
.rtw_lrat_hit(1'b0),
```

This **fails closed, not open**. Guest radix does not work; it also cannot be used as an
attack surface. The alternative — tying the input high — would have made guest mode appear
functional while silently trusting guest-supplied addresses.

## 5.7 Summary

Two sentences worth carrying away:

1. **Never leave the core on behalf of a speculative request, and never update architected
   state until the completion unit confirms the exception was taken.**
2. **Walks are not abortable — they are made harmless.** The reservation bit, the TLB-write
   gate and the never-recycled queue entry are the three legs of that stool. Lengthening the
   walk by two orders of magnitude means strengthening all three.

---

**Not covered here:** the evidence that these mechanisms actually work
([06](06-verification.md)).
