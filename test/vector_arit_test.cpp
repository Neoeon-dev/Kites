// test/vector_arith_test.cpp
#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "assembler/assembler.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

namespace
{
// funct3 = operand form
constexpr uint32_t OPIVV = 0b000, OPMVV = 0b010, OPIVI = 0b011, OPIVX = 0b100;

// Builds one OP-V instruction:  funct6 | vm | vs2 | vs1/rs1/imm | funct3 | vd | opcode
// vm = 1 means unmasked.
uint32_t EncodeOpV(uint32_t funct6, uint32_t vm, uint32_t vs2, int vs1, uint32_t funct3,
                   uint32_t vd)
{
    return (funct6 << 26) | (vm << 25) | (vs2 << 20) | ((static_cast<uint32_t>(vs1) & 31) << 15) |
           (funct3 << 12) | (vd << 7) | 0b1010111;
}

// vtype value for a given SEW (8/16/32/64) and integer LMUL (1/2/4/8)
uint64_t VType(unsigned sew, unsigned lmul)
{
    unsigned vsew = __builtin_ctz(sew / 8);
    unsigned vlmul = __builtin_ctz(lmul);
    return (vsew << 3) | vlmul;
}

class VectorArithTest : public ::testing::Test
{
  protected:
    std::unique_ptr<Kites::RVSSProcessor> vm;

    // Step 1: load the program and set vl/vtype. Call before writing input registers.
    void Load(const std::vector<uint32_t> &program, uint64_t vl, uint64_t vtype)
    {
        Kites::setupVmStateDirectory();
        vm = std::make_unique<Kites::RVSSProcessor>();

        std::string nops;
        for (size_t i = 0; i < program.size(); ++i)
            nops += "addi x0, x0, 0\n";
        std::istringstream src(nops);
        vm->LoadProgram(Kites::assemble(src));

        for (size_t i = 0; i < program.size(); ++i)
            vm->memory_controller_.writeWord(i * 4, program[i]);

        vm->registers_.WriteCsr(0xC20, vl);
        vm->registers_.WriteCsr(0xC21, vtype);
    }

    // Step 2: run everything.
    void Run() { vm->DebugRun(); }

    // Helpers so the tests read nicely.
    void SetElem(size_t vreg, size_t i, unsigned sew, uint64_t v)
    {
        vm->registers_.WriteVectorElement(vreg, i, sew, v);
    }
    uint64_t Elem(size_t vreg, size_t i, unsigned sew)
    {
        return vm->registers_.ReadVectorElement(vreg, i, sew);
    }
};
} // namespace

// ---- One normal case ------------------------------------------------------
TEST_F(VectorArithTest, VaddVvAddsEachElement)
{
    Load({EncodeOpV(0b000000, 1, /*vs2=*/1, /*vs1=*/2, OPIVV, /*vd=*/3)}, // vadd.vv v3, v1, v2
         /*vl=*/4, VType(32, 1));
    for (int i = 0; i < 4; ++i)
    {
        SetElem(1, i, 32, 10 * i);
        SetElem(2, i, 32, 1);
    }

    Run();

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(Elem(3, i, 32), 10u * i + 1) << "element " << i;
}

// ---- Edge case: overflow must wrap inside the element, not carry into the next
TEST_F(VectorArithTest, VaddWrapsWithinElement)
{
    Load({EncodeOpV(0b000000, 1, 1, 2, OPIVV, 3)}, 2, VType(8, 1));
    SetElem(1, 0, 8, 0xFF);
    SetElem(2, 0, 8, 0x01);

    Run();

    EXPECT_EQ(Elem(3, 0, 8), 0x00u);
    EXPECT_EQ(Elem(3, 1, 8), 0x00u); // no carry leaked into element 1
}

// ---- Edge case: signed vs unsigned -----------------------------------------
TEST_F(VectorArithTest, VsraIsArithmeticVsrlIsLogical)
{
    Load({EncodeOpV(0b101001, 1, 1, 1, OPIVI, 2),  // vsra.vi v2, v1, 1
          EncodeOpV(0b101000, 1, 1, 1, OPIVI, 3)}, // vsrl.vi v3, v1, 1
         1, VType(8, 1));
    SetElem(1, 0, 8, 0x80); // -128

    Run();

    EXPECT_EQ(Elem(2, 0, 8), 0xC0u); // sign bit copied in
    EXPECT_EQ(Elem(3, 0, 8), 0x40u); // zero shifted in
}

// ---- Behaviour: elements past vl are left alone ----------------------------
TEST_F(VectorArithTest, TailElementsUntouched)
{
    Load({EncodeOpV(0b000000, 1, 0, 7, OPIVI, 1)}, /*vl=*/2, VType(8, 1)); // vadd.vi v1, v0, 7
    SetElem(1, 2, 8, 0xAA); // already in the tail

    Run();

    EXPECT_EQ(Elem(1, 1, 8), 7u);
    EXPECT_EQ(Elem(1, 2, 8), 0xAAu);
}

// ---- Behaviour: masking with v0 --------------------------------------------
TEST_F(VectorArithTest, MaskedOffElementsUntouched)
{
    Load({EncodeOpV(0b000000, /*vm=*/0, 2, 7, OPIVI, 1)}, 4, VType(8, 1));
    SetElem(0, 0, 8, 0b0101); // mask: elements 0 and 2 active

    Run();

    EXPECT_EQ(Elem(1, 0, 8), 7u);
    EXPECT_EQ(Elem(1, 1, 8), 0u);
    EXPECT_EQ(Elem(1, 2, 8), 7u);
    EXPECT_EQ(Elem(1, 3, 8), 0u);
}

// ---- Undo should restore the destination register ------------------------
TEST_F(VectorArithTest, UndoRestoresVd)
{
    Load({EncodeOpV(0b000000, 1, 0, 5, OPIVI, 1)}, 4, VType(8, 1));
    Run();
    ASSERT_EQ(Elem(1, 0, 8), 5u);

    vm->Undo();

    EXPECT_EQ(Elem(1, 0, 8), 0u);
}

// vsetivli rd, avl, vtype  — bits: 11 | zimm[29:20] | uimm[19:15] | 111 | rd | 1010111
// (EncodeOpV can't build this: vset* has funct3=111 and vtype where funct6/vm/vs2 would be.)
static uint32_t EncodeVsetivli(uint32_t rd, uint32_t avl, uint64_t vtype)
{
    return (0b11u << 30) | ((static_cast<uint32_t>(vtype) & 0x3FF) << 20) | ((avl & 31) << 15) |
           (0b111 << 12) | (rd << 7) | 0b1010111;
}

// ---- vsetivli writes vl, vtype and rd; undo puts them back -----------------
TEST_F(VectorArithTest, VsetivliChangesVlAndVtype)
{
    // Start from values that differ from what vsetivli sets, so we can see it act.
    Load({EncodeVsetivli(/*rd=*/5, /*avl=*/4, VType(32, 1))}, /*vl=*/0, VType(8, 1));

    Run();

    EXPECT_EQ(vm->registers_.ReadCsr(0xC20), 4u);           // vl
    EXPECT_EQ(vm->registers_.ReadCsr(0xC21), VType(32, 1)); // vtype
    EXPECT_EQ(vm->registers_.ReadGpr(5), 4u);               // rd = new vl

    vm->Undo();

    EXPECT_EQ(vm->registers_.ReadCsr(0xC20), 0u);          // vl restored
    EXPECT_EQ(vm->registers_.ReadCsr(0xC21), VType(8, 1)); // vtype restored
    EXPECT_EQ(vm->registers_.ReadGpr(5), 0u);              // rd restored
}

// ---- vl is capped at VLMAX = VLEN/SEW * LMUL --------------------------------
TEST_F(VectorArithTest, VsetivliCapsVlAtVlmax)
{
    // e64, m1 with VLEN=256 -> VLMAX = 4, so asking for 31 gives 4.
    Load({EncodeVsetivli(5, 31, VType(64, 1))}, 0, VType(8, 1));

    Run();

    EXPECT_EQ(vm->registers_.ReadCsr(0xC20), 4u);
    EXPECT_EQ(vm->registers_.ReadGpr(5), 4u);
}

// ---- vadd after vsetivli must use the NEW SEW -------------------------------
TEST_F(VectorArithTest, VsetivliThenVaddUsesNewSew)
{
    // Load with e8 so a vtype-writeback bug would make vadd work on bytes.
    Load({EncodeVsetivli(5, 4, VType(32, 1)),             // vsetivli x5, 4, e32, m1
          EncodeOpV(0b000000, 1, 1, 2, OPIVV, 3)},        // vadd.vv v3, v1, v2
         /*vl=*/0, VType(8, 1));
    for (int i = 0; i < 4; ++i)
    {
        SetElem(1, i, 32, 300 + i); // > 255: only survives if elements are 32-bit
        SetElem(2, i, 32, 1);
    }

    Run();

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(Elem(3, i, 32), 301u + i) << "element " << i;
}