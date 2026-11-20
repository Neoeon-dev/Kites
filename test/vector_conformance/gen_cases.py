#!/usr/bin/env python3
"""Generate Kites vector conformance cases from the upstream riscv-vector-tests configs.

Inputs come verbatim from chipsalliance/riscv-vector-tests (configs in ./upstream/).
The sweep (SEW x LMUL x mask x vl) mirrors that project's generator. Upstream gets its
expected values by running the Spike simulator; Spike isn't available here, so the
expected values below come from a small reference model written straight from the
RISC-V V spec (see EXPECTED below). It shares no code with the Kites C++ sources.

Usage:  python3 test/vector_conformance/gen_cases.py
Output: test/vector_conformance/generated/<insn>.cases   (gitignored, regenerate any time)

Case line format (one case per line, fields separated by spaces, arrays by " | "):
  <id> <sew> <lmul> <avl> <masked> <operand> <exp_vl> <group_regs> | v0 | vd | vs2 | vs1 | expected_vd
Arrays are 64-bit hex words, in the same layout as Kites' VectorRegister
(element i of a group sits at byte offset i*SEW/8, little-endian).
"""

import random
import sys
import tomllib
from pathlib import Path

VLEN = 256  # must match architecture::VLEN_BITS in src/processor/processor_types.h
XLEN = 64
WORDS_PER_REG = VLEN // 64

HERE = Path(__file__).resolve().parent
UPSTREAM = HERE / "upstream"
OUT = HERE / "generated"

SEWS = [8, 16, 32, 64]
LMULS = ["mf8", "mf4", "mf2", "m1", "m2", "m4", "m8"]
LMUL_VALUE = {"mf8": 0.125, "mf4": 0.25, "mf2": 0.5, "m1": 1, "m2": 2, "m4": 4, "m8": 8}
LMUL_BITS = {"m1": 0, "m2": 1, "m4": 2, "m8": 3, "mf8": 5, "mf4": 6, "mf2": 7}

# Reference semantics, per element, on SEW-bit unsigned values a = vs2[i], b = vs1[i]/rs1/imm.
def _signed(v, sew):
    return v - (1 << sew) if v >> (sew - 1) else v

EXPECTED = {
    "vadd":  lambda a, b, s: a + b,
    "vsub":  lambda a, b, s: a - b,
    "vrsub": lambda a, b, s: b - a,
    "vminu": lambda a, b, s: min(a, b),
    "vmaxu": lambda a, b, s: max(a, b),
    "vmin":  lambda a, b, s: a if _signed(a, s) < _signed(b, s) else b,
    "vmax":  lambda a, b, s: a if _signed(a, s) > _signed(b, s) else b,
    "vand":  lambda a, b, s: a & b,
    "vor":   lambda a, b, s: a | b,
}


def legal(sew, lmul):
    # Same filter upstream uses: SEW must fit in ELEN * LMUL (ELEN = XLEN = 64 here).
    return LMUL_VALUE[lmul] >= sew / XLEN


def vlmax(sew, lmul):
    return int(VLEN * LMUL_VALUE[lmul] // sew)


def num(v):
    return int(v, 0) if isinstance(v, str) else int(v)


def test_rows(cfg, sew):
    """Upstream testCases(): the 'base' rows followed by the rows for this SEW."""
    tests = cfg["tests"]
    return [[num(x) for x in row] for row in tests.get("base", []) + tests.get(f"sew{sew}", [])]


def pack(elems, sew, nregs):
    """Pack SEW-bit elements into 64-bit words (little-endian), nregs registers long."""
    data = bytearray(nregs * VLEN // 8)
    width = sew // 8
    for i, e in enumerate(elems):
        data[i * width:(i + 1) * width] = (e & ((1 << sew) - 1)).to_bytes(width, "little")
    return [int.from_bytes(data[k:k + 8], "little") for k in range(0, len(data), 8)]


def unpack(words, sew):
    data = b"".join(w.to_bytes(8, "little") for w in words)
    width = sew // 8
    return [int.from_bytes(data[k:k + width], "little") for k in range(0, len(data), width)]


def hexwords(words):
    return " ".join(f"{w:016x}" for w in words)


def vl_values(vmax1, rng, few):
    # Upstream: {0, vlmax/2-1, vlmax/2, vlmax, vlmax-1, vlmax+1} + 3 random picks.
    # We keep one random pick, and for .vx/.vi (which repeat every scalar row) a
    # smaller set, so the suite stays fast.
    if few:
        vals = [vmax1 // 2 - 1, vmax1, vmax1 + 1]
    else:
        vals = [0, vmax1 // 2 - 1, vmax1 // 2, vmax1, vmax1 - 1, vmax1 + 1, rng.randrange(vmax1)]
    return [max(v, 0) for v in vals]


def gen_arith(name, cfg, rng):
    base, form = name.split(".")
    op = EXPECTED[base]
    lines = []
    for lmul in LMULS:
        for sew in SEWS:
            if not legal(sew, lmul):
                continue
            nregs = max(1, int(LMUL_VALUE[lmul]))   # upstream LMUL1
            n_elems = nregs * VLEN // sew           # elements in the whole register group
            vmax1 = n_elems                         # upstream vlmax1 (uses LMUL1)
            rows = test_rows(cfg, sew)
            mask_m = (1 << sew) - 1
            for masked in (0, 1):
                for avl in vl_values(vmax1, rng, few=(form != "vv")):
                    exp_vl = min(avl, vlmax(sew, lmul))
                    # vs2 (and vs1) cycle through the rows, exactly like upstream.
                    vs2 = [rows[i % len(rows)][1] & mask_m for i in range(n_elems)]
                    vs1 = [rows[i % len(rows)][0] & mask_m for i in range(n_elems)] if form == "vv" else None
                    operands = [None] if form == "vv" else [r[0] for r in rows]
                    for operand in operands:
                        vd = [rng.getrandbits(sew) for _ in range(n_elems)]
                        v0 = [rng.getrandbits(64) for _ in range(WORDS_PER_REG)]
                        if form == "vx":
                            b_all = operand & mask_m
                            op_text = f"{operand & mask_m:x}"
                        elif form == "vi":
                            imm = ((operand & 0x1F) ^ 0x10) - 0x10   # simm5, as upstream
                            b_all = imm & mask_m
                            op_text = str(imm)
                        else:
                            op_text = "-"
                        exp = list(vd)
                        for i in range(exp_vl):
                            if masked and not (v0[i // 64] >> (i % 64)) & 1:
                                continue  # mask-undisturbed
                            b = vs1[i] if form == "vv" else b_all
                            exp[i] = op(vs2[i], b, sew) & mask_m
                        fields = [str(len(lines)), str(sew), lmul, str(avl), str(masked), op_text,
                                  str(exp_vl), str(nregs)]
                        arrays = [hexwords(v0), hexwords(pack(vd, sew, nregs)),
                                  hexwords(pack(vs2, sew, nregs)),
                                  hexwords(pack(vs1, sew, nregs)) if vs1 else "-",
                                  hexwords(pack(exp, sew, nregs))]
                        lines.append(" ".join(fields) + " | " + " | ".join(arrays))
    return lines


def gen_vsetvli(cfg):
    """Upstream genCodevsetvli: per legal (SEW, LMUL, ta, ma) and per (rd, avl) row, run
    vsetvli rd, rs1 / rd, zero / zero, zero in sequence and check vl and vtype.
    We additionally check rd == new vl, which the spec requires but upstream doesn't test."""
    lines = []
    rows = test_rows(cfg, 8)
    for lmul in LMULS:
        for sew in SEWS:
            if not legal(sew, lmul):
                continue
            for vta in (0, 1):
                for vma in (0, 1):
                    vtype = LMUL_BITS[lmul] | (SEWS.index(sew) << 3) | (vta << 6) | (vma << 7)
                    vmax = vlmax(sew, lmul)
                    for _rd_init, avl in rows:
                        # form 0: vsetvli t0, t1  -> vl = min(avl, VLMAX)
                        # form 1: vsetvli t0, zero -> vl = VLMAX
                        # form 2: vsetvli zero, zero -> vl unchanged (vtype is unchanged too)
                        vl0 = min(avl, vmax)
                        vl1 = vmax
                        vl2 = vl1
                        lines.append(f"{len(lines)} {sew} {lmul} {'tu' if not vta else 'ta'} "
                                     f"{'mu' if not vma else 'ma'} {avl} {vtype} {vl0} {vl1} {vl2}")
    return lines


def main():
    OUT.mkdir(exist_ok=True)
    rng = random.Random(0x4B495445)  # fixed seed: the same cases every run
    total = 0
    for path in sorted(UPSTREAM.glob("*.toml")):
        with open(path, "rb") as f:
            cfg = tomllib.load(f)
        name = cfg["name"]
        lines = gen_vsetvli(cfg) if name == "vsetvli" else gen_arith(name, cfg, rng)
        (OUT / f"{name}.cases").write_text("\n".join(lines) + "\n")
        total += len(lines)
        print(f"{name:10s} {len(lines):6d} cases")
    print(f"total      {total:6d} cases -> {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
