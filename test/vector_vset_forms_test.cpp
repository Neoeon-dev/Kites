// vsetivli and vsetvl: assembler encoding, disassembly, parser errors, and execution on
// RVSS and on the 5-stage pipeline without hazard detection / forwarding (RV5S NH_NF).
#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>

#include "assembler/assembler.h"
#include "common/instructions.h"
#include "processor/rv5s/rv5s_processor_nh_nf.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

namespace
{
uint32_t AssembleOne(const std::string &line)
{
    std::istringstream source(".text\n" + line + "\n");
    auto program = Kites::assemble(source);
    EXPECT_EQ(program.text_buffer.size(), 1u) << line;
    return program.text_buffer.empty() ? 0 : program.text_buffer[0];
}

bool Assembles(const std::string &line)
{
    std::istringstream source(".text\n" + line + "\n");
    try
    {
        Kites::assemble(source);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}
} // namespace

// ---------------------------------------------------------------------------------------
// Encoding (expected values worked out by hand from the RVV 1.0 field layout)
// ---------------------------------------------------------------------------------------

// vsetivli: 11 | zimm[9:0] | uimm[4:0] | 111 | rd | 1010111
TEST(VsetForms, VsetivliEncoding)
{
    // vtype e32, m1, ta, ma = 0x0D0; AVL 4; rd x5
    EXPECT_EQ(AssembleOne("vsetivli x5, 4, e32, m1, ta, ma"), 0xCD0272D7u);
    // vtype e8, mf2, tu, mu = 0x007; AVL 31 (largest uimm); rd x0
    EXPECT_EQ(AssembleOne("vsetivli x0, 31, e8, mf2, tu, mu"), 0xC07FF057u);
}

// vsetvl: 1 | 000000 | rs2 | rs1 | 111 | rd | 1010111
TEST(VsetForms, VsetvlEncoding)
{
    EXPECT_EQ(AssembleOne("vsetvl x5, x10, x11"), 0x80B572D7u);
    EXPECT_EQ(AssembleOne("vsetvl t0, a0, a1"), 0x80B572D7u); // ABI names map to the same regs
}

// vsetvli is unchanged by the refactor.
TEST(VsetForms, VsetvliEncodingUnchanged)
{
    EXPECT_EQ(AssembleOne("vsetvli x5, x10, e32, m1, ta, ma"), 0x0D0572D7u);
}

TEST(VsetForms, DisassemblesAllThreeForms)
{
    using Kites::instruction_set::disassemble;
    EXPECT_EQ(disassemble(0xCD0272D7u), "vsetivli x5, 4, e32, m1, ta, ma");
    EXPECT_EQ(disassemble(0xC07FF057u), "vsetivli x0, 31, e8, mf2, tu, mu");
    EXPECT_EQ(disassemble(0x80B572D7u), "vsetvl x5, x10, x11");
    EXPECT_EQ(disassemble(0x0D0572D7u), "vsetvli x5, x10, e32, m1, ta, ma");
}

TEST(VsetForms, RoundTripsThroughDisassembler)
{
    for (const std::string line : {"vsetivli x7, 13, e16, m4, tu, ma",
                                   "vsetivli x1, 0, e64, m1, ta, mu",
                                   "vsetvl x31, x1, x2"})
    {
        EXPECT_EQ(Kites::instruction_set::disassemble(AssembleOne(line)), line);
    }
}

TEST(VsetForms, ParserRejectsBadOperands)
{
    EXPECT_FALSE(Assembles("vsetivli x5, 32, e32, m1, ta, ma")); // uimm is 0..31
    EXPECT_FALSE(Assembles("vsetivli x5, -1, e32, m1, ta, ma"));
    EXPECT_FALSE(Assembles("vsetivli x5, x6, e32, m1, ta, ma")); // AVL must be a number
    EXPECT_FALSE(Assembles("vsetivli x5, 4, e32, m1"));          // tail/mask are required
    EXPECT_FALSE(Assembles("vsetvl x5, x10, e32"));              // vsetvl takes rs2, not vtype
}

// ---------------------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------------------
namespace
{
// NH_NF has no hazard detection: two nops after a GPR producer (read in ID).
constexpr const char *kPad = "addi x0, x0, 0\n"
                             "addi x0, x0, 0\n";

template <typename VmType>
class VsetFormsRun : public ::testing::Test
{
  protected:
    std::unique_ptr<VmType> vm;

    void Run(const std::string &source)
    {
        Kites::setupVmStateDirectory();
        vm = std::make_unique<VmType>();
        std::istringstream stream(source);
        vm->LoadProgram(Kites::assemble(stream));
        vm->DebugRun();
    }
    uint64_t Vl() { return vm->registers_.ReadCsr(0xC20); }
    uint64_t Vtype() { return vm->registers_.ReadCsr(0xC21); }
    uint64_t Gpr(size_t r) { return vm->registers_.ReadGpr(r); }
};

using VmTypes = ::testing::Types<Kites::RVSSProcessor, Kites::RV5StageProcessorNHNF>;
TYPED_TEST_SUITE(VsetFormsRun, VmTypes);
} // namespace

// vsetivli takes AVL from its immediate.
TYPED_TEST(VsetFormsRun, VsetivliSetsVlFromImmediate)
{
    this->Run("vsetivli t0, 3, e32, m1, ta, ma\n");
    EXPECT_EQ(this->Vl(), 3u);
    EXPECT_EQ(this->Vtype(), 0xD0u);
    EXPECT_EQ(this->Gpr(5), 3u); // t0 = new vl
}

// AVL larger than VLMAX is capped (VLEN=256, e64, m1 -> VLMAX = 4).
TYPED_TEST(VsetFormsRun, VsetivliCapsAtVlmax)
{
    this->Run("vsetivli t0, 31, e64, m1, ta, ma\n");
    EXPECT_EQ(this->Vl(), 4u);
    EXPECT_EQ(this->Gpr(5), 4u);
}

// The vl set by vsetivli limits a following vector instruction.
TYPED_TEST(VsetFormsRun, VsetivliThenVaddUsesNewVl)
{
    Kites::setupVmStateDirectory();
    this->vm = std::make_unique<TypeParam>();
    std::istringstream stream(std::string("vsetivli t0, 2, e32, m1, ta, ma\n") + kPad +
                              "vadd.vv v3, v1, v2\n");
    this->vm->LoadProgram(Kites::assemble(stream));
    for (int i = 0; i < 4; ++i)
    {
        this->vm->registers_.WriteVectorElement(1, i, 32, 10 + i);
        this->vm->registers_.WriteVectorElement(2, i, 32, 1);
    }
    this->vm->DebugRun();

    EXPECT_EQ(this->vm->registers_.ReadVectorElement(3, 0, 32), 11u);
    EXPECT_EQ(this->vm->registers_.ReadVectorElement(3, 1, 32), 12u);
    EXPECT_EQ(this->vm->registers_.ReadVectorElement(3, 2, 32), 0u); // past vl: untouched
}

// vsetvl takes vtype from a GPR (rs2) and AVL from another (rs1).
TYPED_TEST(VsetFormsRun, VsetvlReadsVtypeFromRegister)
{
    this->Run(std::string("addi a0, x0, 5\n"
                          "addi a1, x0, 208\n") + // 208 = 0xD0 = e32, m1, ta, ma
              kPad + "vsetvl t0, a0, a1\n");
    EXPECT_EQ(this->Vl(), 5u);
    EXPECT_EQ(this->Vtype(), 0xD0u);
    EXPECT_EQ(this->Gpr(5), 5u);
}

// An illegal vtype in rs2 (reserved bit 8 set) sets vill and vl = 0.
TYPED_TEST(VsetFormsRun, VsetvlIllegalVtypeSetsVill)
{
    this->Run(std::string("addi a0, x0, 5\n"
                          "addi a1, x0, 256\n") +
              kPad + "vsetvl t0, a0, a1\n");
    EXPECT_EQ(this->Vl(), 0u);
    EXPECT_EQ(this->Vtype(), uint64_t{1} << 63);
    EXPECT_EQ(this->Gpr(5), 0u);
}
