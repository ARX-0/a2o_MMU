# 03 — Datapath and bit manipulation

[← 02 The state machine](02-fsm.md) · [Index](00-README.md) · Next: [04 — Integration](04-integration.md)

This is the part of the port most likely to contain silent bugs: the state machine is
visible in simulation, but an off-by-one in a mask produces a plausible-looking wrong
address. It is therefore documented in full and verified against a direct model of the
source (see [06-verification](06-verification.md)).

---

## 3.1 Bit order

Microwatt is written little-endian (`downto`); A2O is MSB-first (`[0:N]`). Radix tables are
big-endian in memory and A2O's L2 returns them MSB-first, so **no byte swap is required** —
the translation is pure index inversion:

```
a2o_index = 63 − microwatt_index
```

Microwatt does byte-swap on the way in (`mmu.vhdl:1490-1494`) because its dcache returns
little-endian data. That step has no counterpart here, and importing it would have been a
bug.

The radix fields, with both numberings, added to `mmu_a2o.vh:251` onward:

```verilog
`define   radixpos_v         0    // mw 63     valid
`define   radixpos_leaf      1    // mw 62     leaf (1=PTE, 0=directory PDE)
`define   radixpos_rts_hi    1    // mw 62:61  RTS high 2 bits (PRTE0)
`define   radixpos_rpn       8    // mw 55:12  RPN, 44 bits (only 41:12 fit RA)
`define   radixpos_nlb       8    // mw 55:8   next-level base, 48 bits
`define   radixpos_rref     55    // mw  8     R, reference
`define   radixpos_c        56    // mw  7     C, change
`define   radixpos_rts_lo   56    // mw  7:5   RTS low 3 bits (PRTE0)
`define   radixpos_ci       58    // mw  5     I, cache inhibited
`define   radixpos_priv     60    // mw  3     privileged
`define   radixpos_read     61    // mw  2     read
`define   radixpos_write    62    // mw  1     write
`define   radixpos_exec     63    // mw  0     execute
`define   radixpos_nls      59    // mw  4:0   NLS / RPDS / PRTS, 5 bits
```

Some positions overlap (`radixpos_leaf` and `radixpos_rts_hi`, `radixpos_c` and
`radixpos_rts_lo`). That is not an error: process-table entries and page-directory entries
are different record types that happen to reuse the same bits.

**RTS extraction** is the awkward one — the field is split across the doubleword
(`mmq_rtw.v:779`):

```verilog
// RTS = '0' & data(62:61) & data(7:5)        mmu.vhdl:1655-1657
assign root_rts = {1'b0, ctx_data_q[i][1], ctx_data_q[i][2],
                         ctx_data_q[i][56], ctx_data_q[i][57], ctx_data_q[i][58]};
```

## 3.2 The barrel shifter

Each level's index comes from shifting the effective page number right by the current shift
amount and taking sixteen bits. Microwatt implements this as a three-stage case mux over
`addr(61 downto 12)` (`mmu.vhdl:1380-1414`).

Converting to A2O indices: the tag's `EPN[0:51]` holds EA[63:12], so `EPN[0]` is EA63 and
`EPN[51]` is EA12. Microwatt's `addrsh(a) = addr(12 + shift + a)` becomes
`addrsh[k] = EPN[36 − shift + k]`.

The implementation (`mmq_rtw.v:740-741`):

```verilog
// addrsh_mw(a) = addr(12+shift+a).  Converting: addrsh[k] = EPN[36-shift+k].
// The shifter input is addr(61:12) == epn[2:51], so epn[0:1] (EA63:62)
// must NOT be visible: Microwatt shifts in zeros from above bit 61.
// Padding with 34 zeros both excludes them and keeps the part-select base
// non-negative for any 6-bit shift, which a 3-stage case-mux would
// otherwise have to special-case.
assign epn_pad = {34'b0, epn[2:`EPN_WIDTH-1]};
assign addrsh  = epn_pad[(68 - ctx_shift_q[i]) +: 16];
```

**The 34 is load-bearing.** An earlier version padded with 32 zeros, which left `EPN[0:1]`
(EA63:62) inside the window. For shifts of 35 or more those bits entered the index, and the
result diverged from Microwatt on every such vector. The comparison bench caught it; see
[06-verification](06-verification.md#64-bugs-found-by-the-benches).

A single indexed part-select replaces Microwatt's three-stage mux. The zero padding
guarantees the base stays in range for any 6-bit shift value, so no special cases are
needed.

## 3.3 The three masks

All three are generated bit-by-bit in `generate` loops, each with its index relation to the
Microwatt original stated in a comment.

| Mask | Width | Purpose | A2O ↔ Microwatt relation |
|---|---:|---|---|
| `mask` | 16 | Select this level's index bits within the directory base | `mask[k] = mask_mw(15−k)` |
| `fm30` | 30 | Merge EA bits into the RPN for pages larger than 4 kB | `fm30[r] = fm_mw(29−r)` |
| `segmask` | 31 | Check that EA bits above `31+RTS` are zero | `segmask[s] = fm_mw(30−s)` |

```verilog
// index mask -- Microwatt addrmaskgen, mmu.vhdl:1417-1432
// mask_mw(b) = 1 for b<5 (seeded 0x001f), else b < mask_size.
for (k = 0; k < 16; k = k + 1)
begin : gen_mask
   assign mask[k] = ((15 - k) < 5) ? 1'b1 :
                    ((15 - k) < ctx_masksize_q[i]) ? 1'b1 : 1'b0;
end
```

The `< 5` seed reproduces Microwatt's `m := x"001f"` initialisation: index widths are never
below five bits, so the low five mask bits are unconditionally set.

## 3.4 Address formation

Three addresses, all 42 bits wide (`REAL_ADDR_WIDTH`, `mmu_a2o.vh:108`), at
`mmq_rtw.v:809-817`:

```verilog
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
```

Two details carried over deliberately from the reference:

- The level index is **OR-ed** into the directory base, not added. Directory bases are
  aligned such that the index bits are already zero, so the two are equivalent — but only
  if the base really is aligned, and OR makes that assumption explicit.
- The partition table is read at **entry 0, doubleword 1**, unconditionally. LPID and the
  PATS size field are ignored. Microwatt's partition table is vestigial
  (`mmu.vhdl:8-10` states there is no guest-real to host-real translation), and A2O's LRAT
  covers partition-scope translation instead.

### Real-address bounds

Microwatt forms 56-bit addresses; A2O's real address is 42 bits. A tree authored for a wider
machine can therefore point outside the addressable range. That is treated as a machine
check rather than silently truncated:

```verilog
// A2O real addresses are 42 bits (`REAL_ADDR_WIDTH). Microwatt forms 56-bit
// PDE addresses, so a tree built for a wider machine can point outside what
// this core can address. That is a machine check, not a silent truncation.
assign ra_overflow = (ctx_fetch_q[i] == Fetch_Pde) ? |(ctx_data_q[i][8:21]) : 1'b0;
```

## 3.5 Final translation assembly

The real page number merges the leaf entry's RPN with effective-address bits, the split
controlled by `fm30` so that pages larger than 4 kB work (`mmq_rtw.v:847`):

```verilog
// RPN = (pde & ~finalmask) | (ea & finalmask)            mmu.vhdl:1831-1833
// RA[41:12] lives at data[22:51] and at epn[22:51].
assign rpn_out = (ctx_data_q[i][22:51] & (~fm30)) | (epn[22:51] & fm30);
```

### Permissions

Microwatt's `check_perm_c` (`mmu.vhdl:351-371`) tests permission per access type. A2O
instead stores a six-bit permission set in the TLB way and lets the ERAT enforce it per
access, so the walker's job is to *produce* that set rather than to test it
(`mmq_rtw.v:854`):

```verilog
// Execute is denied on cache-inhibited pages (mmu.vhdl:367). There is no
// IAMR/AMR in either core, so no KUEP/KUAP (mmu.vhdl:365, dcache.vhdl:1103).
assign usxwr[0] = pde_x & (~pde_ci) & (~pde_priv);   // UX
assign usxwr[1] = pde_x & (~pde_ci);                 // SX
assign usxwr[2] = pde_w & (~pde_priv);               // UW
assign usxwr[3] = pde_w;                             // SW
assign usxwr[4] = pde_r & (~pde_priv);               // UR
assign usxwr[5] = pde_r;                             // SR
```

Neither core implements the authority-mask registers, so there is no key-based protection to
port — this is a documented property of both designs, not an omission here.

### Reference and change bits

```verilog
// P0-2: R and C are CHECKED, never written back. There is no store path
// from the MMU (lq_imq.v:107) and no pending-memory-write mechanism, so a
// hardware R/C update would be an architecturally visible write on behalf
// of a non-committed instruction. Software must set them, as in Microwatt.
// Conservative: the A2O tag carries no load/store bit, so C is required
// unconditionally rather than only for stores (Microwatt mmu.vhdl:1700
// can test r.store because loadstore1 tells it).
assign rc_ok = pde_rref & pde_c;
```

The conservatism is forced: A2O's translation request tag carries no load/store
indication, so the walker cannot know whether `C` is actually required. Requiring it
unconditionally means a store to a page with `C` clear faults — which is correct — and a
*load* to such a page also faults, which is stricter than the architecture demands. The
alternative, installing the entry and letting the ERAT decide, creates a stale-permission
livelock because nothing would ever re-walk to pick up the updated bit.

### Output format

The walker emits a standard A2O `ptepos_*` doubleword rather than a TLB way. This was a
deliberate choice and it is why `mmq_tlb_ctl.v` needed no modification at all — the existing
page-table-reload path carries the result unchanged (`mmq_rtw.v:874`):

```verilog
// A2O ptepos format; mmq_tlb_cmp.v:3576-3581 folds r/c into usxwr, and
// RA[41:12] sits at ptepos_rpn+10 .. +39 (see mmq_tlb_ctl.v:3229).
assign ctx_pte_out[i] = {10'b0, rpn_out,       // ptepos_rpn   0:39
                         wimge_out,            // ptepos_wimge 40:44
                         1'b1,                 // ptepos_r     45
                         4'b0,                 // ptepos_ubits 46:49
                         1'b0,                 // ptepos_sw0   50
                         1'b1,                 // ptepos_c     51
                         inst_size,            // ptepos_size  52:55
                         usxwr,                // ptepos_usxwr 56:61
                         1'b0,                 // ptepos_sw1   62
                         seq_install_valid};   // ptepos_valid 63
```

`ptepos_r` and `ptepos_c` are driven to 1 because the walker has *already* validated them;
downstream logic ANDs them into the permission bits, so passing 1 lets the validated
permissions through unmodified.

## 3.6 Leaf-size demotion

Radix produces page sizes 4 K, 64 K, 2 M and 1 G. A2O can install none of the upper two.
Two independent limits:

**A2O's size code is log₄(size/1 KB).** The encoding admits only power-of-four sizes:

| Size | 4 K | 64 K | 1 M | 16 M | 256 M | 1 G |
|---|---|---|---|---|---|---|
| Code | `0001` | `0011` | `0101` | `0111` | `1001` | `1010` |

2 MB is log₄ = 5.5. It has no encoding at all.

**The reload path keeps only three size bits.** `mmq_tlb_cmp.v:3486` builds the TLB way's
size field as `{1'b0, pte[ptepos_size+0 : +2]}`, so codes above `0111` cannot survive —
1 GB is unreachable through this path too.

The walker therefore installs each leaf at the largest representable **sub-page**
(`mmq_rtw.v:836`):

| Radix leaf | shift | Installed as | Installed shift |
|---|---:|---|---:|
| 4 KB | 0 | 4 KB | 0 |
| 64 KB | 4 | 64 KB | 4 |
| **2 MB** | 9 | **1 MB** | 8 |
| **1 GB** | 18 | **16 MB** | 12 |

```verilog
assign inst_shift = (ctx_shift_q[i] >= 6'd12) ? InstShift_16MB :
                    (ctx_shift_q[i] >= 6'd8)  ? InstShift_1MB  :
                    (ctx_shift_q[i] >= 6'd4)  ? RadixShift_64KB :
                                                RadixShift_4KB;
```

**Demotion is always architecturally safe.** A smaller page maps a strict subset of the same
translation with identical permissions and attributes. Software observes correct behaviour;
the only cost is more TLB misses on large pages, since one 2 MB region now needs two 1 MB
entries.

The subtlety is that `fm30` must be built from the **installed** shift, not the leaf shift.
Using the smaller shift merges more effective-address bits into the RPN, which is exactly
what produces the correct sub-page translation. Getting this backwards would map the wrong
1 MB of the 2 MB region.

Lifting the restriction requires widening the way's size field through
`mmq_tlb_cmp.v:3486`/`:3527` and adding new compare-mask entries in
`mmq_tlb_matchline.v:194-224`. The matchline's pre-decoded mask scheme could express 2 MB;
the four-bit size code cannot.

---

**Not covered here:** how the module connects to the rest of the MMU
([04](04-integration.md)) or why the walk can be killed part-way ([05](05-ooo-safety.md)).
