// Hazard detection for vector instructions on the 5-stage pipelines that detect hazards:
// RV5S H_NF (no forwarding) and H_F (forwarding).
//
// Timing being checked: GPR operands are read in ID, vector registers / vl / vtype in EX,
// and all results are written in WB (which runs before EX within a cycle). So
//  - vector -> vector (vregs, or vset -> vl/vtype): 1 stall at distance 1, none further;
//    no vector forwarding exists, so H_F stalls the same.
//  - vset rd -> GPR reader, GPR producer -> .vx / vle / vset: ordinary GPR rules
//    (H_NF: 2 stalls at distance 1; H_F: forwarded, no stall).
#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>

#include "assembler/assembler.h"
#include "processor/rv5s/rv5s_processor_h_f.h"
#include "processor/rv5s/rv5s_processor_h_nf.h"
#include "utils/utils.h"

namespace
{
template <typename VmType>
struct Mode;
template <>
struct Mode<Kites::RV5StageProcessorHNF>
{
    static constexpr unsigned kGprStalls = 2; // GPR dependency at distance 1
};
template <>
struct Mode<Kites::RV5StageProcessorHF>
{
    static constexpr unsigned kGprStalls = 0; // forwarded
};

// a0 = 4, then vl = 4 with e32/m1, then enough nops that nothing above can stall
// whatever follows: each test only measures the instructions after this.
const std::string kPrologue = "addi a0, x0, 4\n"
                              "lui a2, 0x10000\n"
                              "addi a1, a2, 256\n"
                              "addi x0, x0, 0\n"
                              "addi x0, x0, 0\n"
                              "vsetvli t0, a0, e32, m1, ta, ma\n"
                              "addi x0, x0, 0\n"
                              "addi x0, x0, 0\n";

template <typename VmType>
class VectorHazardTest : public ::testing::Test
{
  protected:
    std::unique_ptr<VmType> vm;

    void Load(const std::string &source)
    {
        Kites::setupVmStateDirectory();
        vm = std::make_unique<VmType>();
        std::istringstream stream(source);
        vm->LoadProgram(Kites::assemble(stream));
        for (int i = 0; i < 8; ++i)
        {
            vm->registers_.WriteVectorElement(1, i, 32, 5);
            vm->registers_.WriteVectorElement(2, i, 32, 7);
        }
    }

    unsigned Cycles(const std::string &source)
    {
        Load(source);
        vm->DebugRun();
        return vm->cycle_s_;
    }

    // Stalls of `dependent` beyond those of `independent` (same length, same prologue).
    int ExtraStalls(const std::string &dependent, const std::string &independent)
    {
        const int with = static_cast<int>(Cycles(kPrologue + dependent));
        const int without = static_cast<int>(Cycles(kPrologue + independent));
        return with - without;
    }

    uint64_t Elem(size_t vreg, size_t i, unsigned sew = 32)
    {
        return vm->registers_.ReadVectorElement(vreg, i, sew);
    }
};

using HazardVms = ::testing::Types<Kites::RV5StageProcessorHNF, Kites::RV5StageProcessorHF>;
TYPED_TEST_SUITE(VectorHazardTest, HazardVms);
} // namespace

// ---------------------------------------------------------------------------------------
// Correctness: back-to-back dependent vector code gives the right answer with no nops.
// ---------------------------------------------------------------------------------------

TYPED_TEST(VectorHazardTest, DependentChainWithoutNops)
{
    this->Load("addi a0, x0, 4\n"
               "vsetvli t0, a0, e32, m1, ta, ma\n" // GPR a0 -> vset, vl/vtype -> vadd
               "vadd.vv v3, v1, v2\n"              // 12
               "vadd.vv v4, v3, v1\n"              // 17 (v3 RAW)
               "vsub.vv v5, v4, v2\n"              // 10 (v4 RAW)
               "addi t1, t0, 10\n");               // vset rd -> GPR reader
    this->vm->DebugRun();

    EXPECT_EQ(this->vm->registers_.ReadGpr(5), 4u);  // t0 = vl
    EXPECT_EQ(this->vm->registers_.ReadGpr(6), 14u); // t1 = vl + 10
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_EQ(this->Elem(3, i), 12u) << "v3[" << i << "]";
        EXPECT_EQ(this->Elem(4, i), 17u) << "v4[" << i << "]";
        EXPECT_EQ(this->Elem(5, i), 10u) << "v5[" << i << "]";
    }
    EXPECT_EQ(this->Elem(5, 4), 0u); // tail untouched
}

TYPED_TEST(VectorHazardTest, LoadComputeStoreReloadWithoutNops)
{
    this->Load("lui a0, 0x10000\n"
               "addi a1, a0, 256\n"
               "addi a2, x0, 4\n"
               "vsetvli t0, a2, e32, m1, ta, ma\n"
               "vle32.v v6, (a0)\n"    // GPR a0 (written 4 instructions earlier)
               "vadd.vv v7, v6, v6\n"  // load -> use
               "vse32.v v7, (a1)\n"    // arith -> store data
               "vle32.v v8, (a1)\n");  // store -> load (memory, in order)
    for (int i = 0; i < 4; ++i)
        this->vm->memory_controller_.writeWord(0x10000000 + 4 * i, 100 + i);
    this->vm->DebugRun();

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_EQ(this->vm->memory_controller_.readWord(0x10000100 + 4 * i), 200u + 2 * i);
        EXPECT_EQ(this->Elem(8, i), 200u + 2 * i) << "v8[" << i << "]";
    }
}

TYPED_TEST(VectorHazardTest, GprScalarOperandWithoutNops)
{
    this->Load("addi a0, x0, 4\n"
               "addi a1, x0, 3\n"
               "vsetvli t0, a0, e32, m1, ta, ma\n"
               "addi a1, a1, 1\n"       // a1 = 4
               "vsub.vx v3, v1, a1\n"); // 5 - 4
    this->vm->DebugRun();

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(this->Elem(3, i), 1u) << "v3[" << i << "]";
}

// ---------------------------------------------------------------------------------------
// Stall counts
// ---------------------------------------------------------------------------------------

TYPED_TEST(VectorHazardTest, VregRawStallsOneCycle)
{
    EXPECT_EQ(this->ExtraStalls("vadd.vv v3, v1, v2\nvadd.vv v4, v3, v1\n",
                                "vadd.vv v3, v1, v2\nvadd.vv v4, v2, v1\n"),
              1);
}

TYPED_TEST(VectorHazardTest, VregRawAtDistanceTwoNeedsNoStall)
{
    EXPECT_EQ(this->ExtraStalls("vadd.vv v3, v1, v2\naddi x0, x0, 0\nvadd.vv v4, v3, v1\n",
                                "vadd.vv v3, v1, v2\naddi x0, x0, 0\nvadd.vv v4, v2, v1\n"),
              0);
}

// vd is read too (masked-off / tail elements keep their old values).
TYPED_TEST(VectorHazardTest, ReadOfOldDestinationStalls)
{
    EXPECT_EQ(this->ExtraStalls("vadd.vv v3, v1, v2\nvadd.vv v3, v1, v2\n",
                                "vadd.vv v3, v1, v2\nvadd.vv v4, v1, v2\n"),
              1);
}

TYPED_TEST(VectorHazardTest, MaskRegisterV0Stalls)
{
    EXPECT_EQ(this->ExtraStalls("vadd.vv v0, v1, v2\nvadd.vv v3, v1, v2, v0.t\n",
                                "vadd.vv v0, v1, v2\nvadd.vv v3, v1, v2\n"),
              1);
}

TYPED_TEST(VectorHazardTest, VsetStallsVectorConsumerOneCycle)
{
    // A vset directly ahead stalls exactly as long as one nop would.
    const unsigned direct = this->Cycles(kPrologue + "vsetvli t1, a0, e16, m1, ta, ma\n"
                                                     "vadd.vv v3, v1, v2\n");
    const unsigned padded = this->Cycles(kPrologue + "vsetvli t1, a0, e16, m1, ta, ma\n"
                                                     "addi x0, x0, 0\n"
                                                     "vadd.vv v3, v1, v2\n");
    EXPECT_EQ(direct, padded);
}

// vsetvli x0, x0 keeps the current vl, so it reads vl; vsetivli does not.
TYPED_TEST(VectorHazardTest, VsetKeepVlFormReadsVl)
{
    EXPECT_EQ(this->ExtraStalls("vsetvli t1, a0, e32, m1, ta, ma\nvsetvli x0, x0, e32, m1, ta, ma\n",
                                "vsetvli t1, a0, e32, m1, ta, ma\nvsetivli x0, 4, e32, m1, ta, ma\n"),
              1);
}

// Store data is a vector register read in EX.
TYPED_TEST(VectorHazardTest, StoreDataRawStalls)
{
    EXPECT_EQ(this->ExtraStalls("vadd.vv v3, v1, v2\nvse32.v v3, (a1)\n",
                                "vadd.vv v3, v1, v2\nvse32.v v2, (a1)\n"),
              1);
}

// The written group is v4..v5 (LMUL = 2), so reading v5 alone still collides.
TYPED_TEST(VectorHazardTest, RegisterGroupOverlapStalls)
{
    const std::string m2 = "vsetvli t1, a0, e64, m2, ta, ma\naddi x0, x0, 0\naddi x0, x0, 0\n";
    // vse8 under e64/m2 has EMUL = 1/4: a single register.
    EXPECT_EQ(this->ExtraStalls(m2 + "vadd.vv v4, v2, v6\nvse8.v v5, (a1)\n",
                                m2 + "vadd.vv v4, v2, v6\nvse8.v v7, (a1)\n"),
              1);
}

// vadd.vv v3, v2, v1 has 1 in the rs1 field, but it's vs1, not x1: no GPR hazard.
TYPED_TEST(VectorHazardTest, VectorRegisterFieldsAreNotGprSources)
{
    EXPECT_EQ(this->ExtraStalls("addi x1, x0, 5\nvadd.vv v3, v2, v1\n",
                                "addi x7, x0, 5\nvadd.vv v3, v2, v1\n"),
              0);
}

TYPED_TEST(VectorHazardTest, GprToVxFollowsScalarRules)
{
    EXPECT_EQ(this->ExtraStalls("addi a3, x0, 5\nvadd.vx v3, v1, a3\n",
                                "addi a3, x0, 5\nvadd.vx v3, v1, a4\n"),
              static_cast<int>(Mode<TypeParam>::kGprStalls));
}

TYPED_TEST(VectorHazardTest, VsetRdToGprReaderFollowsScalarRules)
{
    EXPECT_EQ(this->ExtraStalls("vsetvli t1, a0, e32, m1, ta, ma\naddi t2, t1, 1\n",
                                "vsetvli t1, a0, e32, m1, ta, ma\naddi t2, a0, 1\n"),
              static_cast<int>(Mode<TypeParam>::kGprStalls));
}

TYPED_TEST(VectorHazardTest, VsetRdForwardedValueIsVl)
{
    this->Load(kPrologue + "vsetvli t1, a0, e32, m1, ta, ma\naddi t2, t1, 1\n");
    this->vm->DebugRun();
    EXPECT_EQ(this->vm->registers_.ReadGpr(7), 5u); // t2 = vl + 1
}
