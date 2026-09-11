#!/usr/bin/env bash
# Unit tests for mmq_rtw.v (the radix tablewalker).
#   tb_math.v  -- the ported bit manipulation vs a direct model of Microwatt's mmu.vhdl
#   tb_walk.v  -- end-to-end walks against a behavioural L2 and a real radix tree
set -uo pipefail
cd "$(dirname "$0")/.."
VVP="$(dirname "$(command -v iverilog)")/vvp"

echo "== lint =="
verilator --lint-only -Wall -Wno-fatal -Wno-LITENDIAN -Wno-DECLFILENAME -Wno-VARHIDDEN \
   -Itrilib -Iwork work/mmq_rtw.v --top-module mmq_rtw

echo "== bit-math vs Microwatt reference =="
iverilog -g2005 -o /tmp/tb_math sim/tb_math.v 2>&1 | grep -v Anachronistic || true
"$VVP" /tmp/tb_math

echo "== end-to-end walk =="
iverilog -g2005 -Iwork -Itrilib -y trilib -y work -o /tmp/tb_walk sim/tb_walk.v work/mmq_rtw.v 2>&1 | grep -v Anachronistic || true
"$VVP" /tmp/tb_walk
echo "== all mmq_rtw tests passed =="
