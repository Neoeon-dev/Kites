# Vector conformance tests

Tests for the vector instructions Kites implements, built from the community test suite
[chipsalliance/riscv-vector-tests](https://github.com/chipsalliance/riscv-vector-tests)
(commit `f76bff1`, Apache-2.0, see `upstream/LICENSE`).

| Folder / file | What it is | In git? |
|---|---|---|
| `upstream/*.toml` | Upstream test inputs, copied verbatim: one file per instruction | yes |
| `gen_cases.py` | Expands the inputs into cases and computes the expected results | yes |
| `generated/*.cases` | The expanded cases (about 20k, 14 MB) | no, regenerate them |
| `../vector_conformance_test.cpp` | Runs every case on RVSS and on RV5S NH_NF | yes |

Covered: `vsetvli` and `vadd/vsub/vrsub/vminu/vmin/vmaxu/vmax/vand/vor` in all the forms
Kites supports (`.vv`, `.vx`, `.vi`). Each case is swept over SEW 8–64, LMUL mf8–m8,
masked and unmasked, and several `vl` values, including `vl = 0` and AVL > VLMAX.

## Running them

```bash
# 1. Generate the cases (once, and again after changing gen_cases.py or upstream/)
python3 test/vector_conformance/gen_cases.py

# 2. Build the tests (tests are off by default)
cmake -S . -B build/test -G Ninja -DENABLE_TESTS=ON
cmake --build build/test --target kites_test

# 3. Run only the conformance tests
./build/test/test/kites_test --gtest_filter='*VectorConformance*'

#    ...or just one instruction on one processor
./build/test/test/kites_test --gtest_filter='*RVSS*vmin_vx'
```

If step 1 was skipped, the tests show as `SKIPPED` along with the command to run.

## Reading a failure

```
vmin.vv case 1 (line 2 of generated/vmin.vv.cases): e8 mf8 avl=15 unmasked
  program:
    vsetvli t1, a0, e8, mf8, tu, mu
    vmin.vv v8, v16, v24
  element 3: vs2=0x00 vs1=0xf8 vd_before=0xe5  expected=0x00 got=0xf8
```

- **program**: exactly what ran. Beforehand `a0` = AVL, `a1` = the `.vx` scalar,
  `v0` = mask, `v8` = vd, `v16` = vs2 and `v24` = vs1.
- **element N**: the inputs to that one element and what came out. `(tail)` means the
  element is at or past `vl`, so it should have been left alone. `mask=0` means the
  mask turned it off, so it should also have been left alone.
- Work the element out by hand using the spec. Here, signed `min(0x00, 0xf8)` is
  `min(0, -8) = -8 = 0xf8`, so `got` is correct and the *expected* value is wrong. A
  failure only means the two disagree; it's your job to decide which one is wrong.

## Where expected values come from

Upstream gets expected results by running the Spike simulator. Spike isn't installed
here, so `gen_cases.py` uses a short reference model written from the RISC-V V spec
(the `EXPECTED` table plus tail/mask-undisturbed handling). It shares no code with the
C++ implementation. If you install Spike later, you can cross-check the expected values
against it.

Differences from upstream:
- upstream's three random `vl` picks are reduced to one fixed-seed pick
- `.vx`/`.vi` use 3 `vl` values instead of 9, because every scalar row is already
  repeated for each one
- `vsetvli` additionally checks that `rd` receives the new `vl`.

## Adding an instruction

1. Copy its config from upstream's `configs/v/` into `upstream/`.
2. Add its semantics to `EXPECTED` in `gen_cases.py`.
3. Add its name to `kArithInsns` in `vector_conformance_test.cpp`.
4. Regenerate the cases, rebuild, and run.
