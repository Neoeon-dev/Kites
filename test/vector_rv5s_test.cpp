// Vector instructions through the shared vector_unit on RVSS and on the
// 5-stage pipeline without hazard detection / forwarding (RV5S NH_NF).
#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>

#include "assembler/assembler.h"
#include "processor/rv5s/rv5s_processor_nh_nf.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

namespace
{
// NH_NF has no hazard detection: a dependent instruction needs two nops after a
// GPR producer (read in ID). Two nops are also always enough for vector operands.
constexpr const char *kPad = "addi x0, x0, 0\n"
                             "addi x0, x0, 0\n";

template <typename VmType>
class VectorPipelineTest : public ::testing::Test
{
  protected:
    std::unique_ptr<VmType> vm;

    void Load(const std::string &source)
    {
        Kites::setupVmStateDirectory();
        vm = std::make_unique<VmType>();
        std::istringstream stream(source);
        vm->LoadProgram(Kites::assemble(stream));
    }

    void Run() { vm->DebugRun(); }

    void SetElem(size_t vreg, size_t i, unsigned sew, uint64_t v)
    {
        vm->registers_.WriteVectorElement(vreg, i, sew, v);
    }
    uint64_t Elem(size_t vreg, size_t i, unsigned sew)
    {
        return vm->registers_.ReadVectorElement(vreg, i, sew);
    }
};

using VmTypes = ::testing::Types<Kites::RVSSProcessor, Kites::RV5StageProcessorNHNF>;
TYPED_TEST_SUITE(VectorPipelineTest, VmTypes);
} // namespace

// vsetvli sets vl/vtype/rd, then vadd.vv uses the new SEW.
TYPED_TEST(VectorPipelineTest, VsetvliThenVaddVv)
{
    this->Load(std::string("addi a0, x0, 8\n") + kPad +
               "vsetvli t0, a0, e32, m1, ta, ma\n" + kPad +
               "vadd.vv v3, v1, v2\n");
    for (int i = 0; i < 8; ++i)
    {
        this->SetElem(1, i, 32, 1000 + i); // > 255: only right if elements are 32-bit
        this->SetElem(2, i, 32, 1);
    }

    this->Run();

    EXPECT_EQ(this->vm->registers_.ReadGpr(5), 8u);      // t0 = vl
    EXPECT_EQ(this->vm->registers_.ReadCsr(0xC20), 8u);  // vl
    EXPECT_EQ(this->vm->registers_.ReadCsr(0xC21), 208u); // e32, m1, ta, ma
    for (int i = 0; i < 8; ++i)
        EXPECT_EQ(this->Elem(3, i, 32), 1001u + i) << "element " << i;
}

// .vx takes its scalar from a GPR read in ID.
TYPED_TEST(VectorPipelineTest, VsubVxUsesGprScalar)
{
    this->Load(std::string("addi a0, x0, 4\n"
                           "addi a1, x0, 3\n") +
               kPad + "vsetvli t0, a0, e16, m1, ta, ma\n" + kPad + "vsub.vx v4, v1, a1\n");
    for (int i = 0; i < 4; ++i)
        this->SetElem(1, i, 16, 10 * (i + 1));

    this->Run();

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(this->Elem(4, i, 16), 10u * (i + 1) - 3) << "element " << i;
    EXPECT_EQ(this->Elem(4, 4, 16), 0u); // tail untouched
}

// LMUL=2: the result spans a two-register group (v4, v5).
TYPED_TEST(VectorPipelineTest, Lmul2WritesWholeGroup)
{
    // e64, m2 with VLEN=256 -> VLMAX = 8 elements over two registers.
    this->Load(std::string("addi a0, x0, 8\n") + kPad + "vsetvli t0, a0, e64, m2, ta, ma\n" +
               kPad + "vadd.vv v4, v2, v6\n");
    for (int i = 0; i < 4; ++i)
    {
        this->SetElem(2, i, 64, i);       // group v2..v3: elements 0..3 in v2
        this->SetElem(3, i, 64, 100 + i); // elements 4..7 in v3
        this->SetElem(6, i, 64, 1);
        this->SetElem(7, i, 64, 1);
    }

    this->Run();

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_EQ(this->Elem(4, i, 64), static_cast<uint64_t>(i) + 1) << "v4[" << i << "]";
        EXPECT_EQ(this->Elem(5, i, 64), 101u + i) << "v5[" << i << "]";
    }
}

// Undoing every step puts vl, vtype, rd and the vector destination back.
TYPED_TEST(VectorPipelineTest, UndoAllRestoresState)
{
    this->Load(std::string("addi a0, x0, 4\n") + kPad + "vsetvli t0, a0, e32, m1, ta, ma\n" +
               kPad + "vadd.vv v3, v1, v1\n");
    for (int i = 0; i < 4; ++i)
        this->SetElem(1, i, 32, 7);
    const uint64_t initial_vtype = this->vm->registers_.ReadCsr(0xC21); // vill after reset
    ASSERT_EQ(initial_vtype, uint64_t{1} << 63);

    this->Run();
    ASSERT_EQ(this->Elem(3, 0, 32), 14u);

    for (int i = 0; i < 100; ++i) // more than enough steps; Undo() ignores an empty stack
        this->vm->Undo();

    EXPECT_EQ(this->Elem(3, 0, 32), 0u);
    EXPECT_EQ(this->vm->registers_.ReadCsr(0xC20), 0u);
    EXPECT_EQ(this->vm->registers_.ReadCsr(0xC21), initial_vtype);
    EXPECT_EQ(this->vm->registers_.ReadGpr(5), 0u);
    EXPECT_EQ(this->Elem(1, 0, 32), 7u); // inputs untouched
}

// RV5S NH_NF reads vector registers, vl and vtype in EX, and WB runs before EX
// within a cycle, so a vector-to-vector dependency needs just one nop.
TEST(VectorRv5sNhNf, VectorDependencyNeedsOneNop)
{
    Kites::setupVmStateDirectory();
    auto vm = std::make_unique<Kites::RV5StageProcessorNHNF>();
    std::istringstream stream(std::string("addi a0, x0, 4\n") + kPad +
                              "vsetvli t0, a0, e32, m1, ta, ma\n"
                              "addi x0, x0, 0\n"
                              "vadd.vv v2, v1, v1\n"
                              "addi x0, x0, 0\n"
                              "vadd.vv v3, v2, v1\n");
    vm->LoadProgram(Kites::assemble(stream));
    for (int i = 0; i < 4; ++i)
        vm->registers_.WriteVectorElement(1, i, 32, 5);

    vm->DebugRun();

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, i, 32), 15u) << "element " << i;
}
