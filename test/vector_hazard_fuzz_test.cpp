// Differential fuzz test for vector hazards: random back-to-back vector/scalar programs
// (no nops) must leave the same architectural state on the hazard-detecting pipelines
// (H_NF, H_F) as on the single-cycle RVSS reference.
#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "assembler/assembler.h"
#include "processor/rv5s/rv5s_processor_h_f.h"
#include "processor/rv5s/rv5s_processor_h_nf.h"
#include "processor/rv5s/rv5s_processor_nh_f.h"
#include "processor/rv5s/rv5s_processor_nh_nf.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

namespace
{
constexpr uint64_t kBase = 0x10000000; // a0
constexpr int kMemBytes = 1024;

struct Generator
{
    std::mt19937 rng;
    unsigned group = 1; // current LMUL group size, to keep most operands aligned

    explicit Generator(unsigned seed) : rng(seed) {}

    int Pick(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }
    bool Chance(int percent) { return Pick(0, 99) < percent; }

    std::string Vreg()
    {
        // Small pool of aligned registers so dependencies are frequent.
        const int n_regs = std::max(1, std::min(4, 32 / static_cast<int>(group)));
        return "v" + std::to_string(Pick(0, n_regs - 1) * group);
    }
    std::string Gpr() { return std::vector<std::string>{"a1", "a2", "a3", "t0", "t1"}[Pick(0, 4)]; }

    std::string Vset()
    {
        static const char *lmuls[] = {"mf2", "m1", "m2", "m4", "m8"};
        static const unsigned groups[] = {1, 1, 2, 4, 8};
        static const char *sews[] = {"e8", "e16", "e32", "e64"};
        int l = Pick(0, 4);
        int s = Pick(0, 3);
        if (l == 0 && s == 3) // e64/mf2 is illegal with ELEN=64
            s = 2;
        group = groups[l];
        const std::string vt = std::string(sews[s]) + ", " + lmuls[l] + ", " +
                               (Chance(50) ? "ta" : "tu") + ", " + (Chance(50) ? "ma" : "mu");
        const std::string rd = Chance(25) ? "x0" : Gpr();
        if (Chance(15))
        { // vsetvl takes vtype from a GPR written right before it
            const unsigned vtype = (static_cast<unsigned>(s) << 3) |
                                   (l == 0 ? 0b111u : static_cast<unsigned>(l - 1)) |
                                   (Chance(50) ? 0x40u : 0u) | (Chance(50) ? 0x80u : 0u);
            return "addi t2, x0, " + std::to_string(vtype) + "\nvsetvl " + rd + ", " + Gpr() +
                   ", t2";
        }
        switch (Pick(0, 2))
        {
        case 0: return "vsetivli " + rd + ", " + std::to_string(Pick(0, 31)) + ", " + vt;
        case 1: if (Chance(30)) return "vsetvli x0, x0, " + vt; [[fallthrough]];
        default: return "vsetvli " + rd + ", " + Gpr() + ", " + vt;
        }
    }

    std::string Arith()
    {
        static const char *ops[] = {"vadd", "vsub", "vand", "vor", "vmin", "vmaxu"};
        const std::string op = ops[Pick(0, 5)];
        const bool masked = Chance(30);
        std::string vd = Vreg();
        if (masked && vd == "v0")
            vd = "v" + std::to_string(group);
        const std::string mask = masked ? ", v0.t" : "";
        switch (Pick(0, 2))
        {
        case 0: return op + ".vv " + vd + ", " + Vreg() + ", " + Vreg() + mask;
        case 1: return op + ".vx " + vd + ", " + Vreg() + ", " + Gpr() + mask;
        default:
            if (op == "vadd" || op == "vand" || op == "vor")
                return op + ".vi " + vd + ", " + Vreg() + ", " + std::to_string(Pick(-16, 15)) + mask;
            return op + ".vv " + vd + ", " + Vreg() + ", " + Vreg() + mask;
        }
    }

    std::string Mem()
    {
        static const char *eews[] = {"8", "16", "32", "64"};
        const std::string eew = eews[Pick(0, 3)];
        const bool masked = Chance(25);
        std::string vr = Vreg();
        const bool load = Chance(50);
        if (load && masked && vr == "v0")
            vr = "v" + std::to_string(group);
        return std::string(load ? "vle" : "vse") + eew + ".v " + vr + ", (" +
               (Chance(50) ? "a0" : "a4") + ")" + (masked ? ", v0.t" : "");
    }

    std::string Scalar()
    {
        switch (Pick(0, 3))
        {
        case 0: return "lw " + Gpr() + ", " + std::to_string(4 * Pick(0, 15)) + "(a0)";
        case 1: return "add " + Gpr() + ", " + Gpr() + ", " + Gpr();
        default: return "addi " + Gpr() + ", " + Gpr() + ", " + std::to_string(Pick(-8, 20));
        }
    }

    // One entry per instruction (a vsetvl entry also holds the addi setting its vtype).
    std::vector<std::string> Program(int length)
    {
        std::vector<std::string> p = {"lui a0, 0x10000", "addi a4, a0, 128", "addi a1, x0, 5",
                                      "addi a2, x0, 17", "addi a3, x0, 40", Vset()};
        int label = 0;
        int pending_label_at = -1; // instruction index where the open label goes
        for (int i = 0; i < length; ++i)
        {
            if (i == pending_label_at)
                p.push_back("L" + std::to_string(label++) + ":");
            const int k = Pick(0, 99);
            if (k < 15)
                p.push_back(Vset());
            else if (k < 50)
                p.push_back(Arith());
            else if (k < 70)
                p.push_back(Mem());
            else if (k < 90 || pending_label_at > i)
                p.push_back(Scalar());
            else
            { // forward branch / jump over 1-3 instructions
                const std::string target = "L" + std::to_string(label);
                static const char *br[] = {"beq", "bne", "blt", "bge"};
                p.push_back(Chance(25) ? "jal x0, " + target
                                       : std::string(br[Pick(0, 3)]) + " " + Gpr() + ", " +
                                             Gpr() + ", " + target);
                pending_label_at = i + 1 + Pick(1, 3);
            }
        }
        if (pending_label_at >= length)
            p.push_back("L" + std::to_string(label) + ":");
        return p;
    }
};

// The program with `nops` nops after every instruction (for VMs without hazard detection).
std::string Render(const std::vector<std::string> &lines, int nops)
{
    std::string out;
    for (const auto &entry : lines)
    {
        std::istringstream split(entry);
        for (std::string line; std::getline(split, line);)
        {
            out += line + "\n";
            if (line.back() != ':')
                for (int n = 0; n < nops; ++n)
                    out += "addi x0, x0, 0\n";
        }
    }
    return out;
}

template <typename Vm>
std::unique_ptr<Vm> Fresh(const std::string &program)
{
    Kites::setupVmStateDirectory();
    auto vm = std::make_unique<Vm>();
    std::istringstream stream(program);
    vm->LoadProgram(Kites::assemble(stream));
    for (int i = 0; i < kMemBytes; ++i)
        vm->memory_controller_.writeByte(kBase + i, static_cast<uint8_t>(i * 37 + 11));
    for (size_t r = 0; r < 32; ++r)
        for (size_t e = 0; e < Kites::architecture::VLEN_BITS / 8; ++e)
            vm->registers_.WriteVectorElement(r, e, 8, (r * 29 + e * 7 + 3) & 0xFF);
    return vm;
}

template <typename Vm>
std::unique_ptr<Vm> RunOn(const std::string &program)
{
    auto vm = Fresh<Vm>(program);
    vm->DebugRun();
    return vm;
}

template <typename A, typename B>
::testing::AssertionResult SameState(const A &ref, const B &dut)
{
    std::ostringstream diff;
    for (size_t r = 1; r < 32; ++r)
        if (ref.registers_.ReadGpr(r) != dut.registers_.ReadGpr(r))
            diff << " x" << r << ": " << ref.registers_.ReadGpr(r) << " vs "
                 << dut.registers_.ReadGpr(r) << ";";
    for (size_t csr : {size_t{0xC20}, size_t{0xC21}})
        if (ref.registers_.ReadCsr(csr) != dut.registers_.ReadCsr(csr))
            diff << " csr" << std::hex << csr << std::dec << ": " << ref.registers_.ReadCsr(csr)
                 << " vs " << dut.registers_.ReadCsr(csr) << ";";
    for (size_t r = 0; r < 32; ++r)
        if (ref.registers_.ReadVreg(r) != dut.registers_.ReadVreg(r))
            diff << " v" << r << ";";
    auto &rm = const_cast<A &>(ref).memory_controller_;
    auto &dm = const_cast<B &>(dut).memory_controller_;
    for (int i = 0; i < kMemBytes; ++i)
        if (rm.peekByte(kBase + i) != dm.peekByte(kBase + i))
        {
            diff << " mem+" << i << ";";
            break;
        }
    if (diff.str().empty())
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << diff.str();
}
} // namespace

TEST(VectorHazardFuzz, PipelinesMatchRvss)
{
    int failures = 0;
    unsigned first = 1, last = 300;
    if (const char *only = std::getenv("KITES_FUZZ_SEED"))
        first = last = static_cast<unsigned>(std::atoi(only));
    for (unsigned seed = first; seed <= last && failures < 5; ++seed)
    {
        Generator gen(seed);
        const auto lines = gen.Program(40);
        const std::string program = Render(lines, 0);
        auto ref = RunOn<Kites::RVSSProcessor>(program);
        auto hnf = RunOn<Kites::RV5StageProcessorHNF>(program);
        auto hf = RunOn<Kites::RV5StageProcessorHF>(program);
        // No hazard detection: pad with what each mode's rules require.
        auto ref_pad = RunOn<Kites::RVSSProcessor>(Render(lines, 1));
        auto nhf = RunOn<Kites::RV5StageProcessorNHF>(Render(lines, 1));
        auto ref_pad2 = RunOn<Kites::RVSSProcessor>(Render(lines, 2));
        auto nhnf = RunOn<Kites::RV5StageProcessorNHNF>(Render(lines, 2));

        // Undo everything: back to the initial state.
        auto init = Fresh<Kites::RVSSProcessor>(program);
        for (int i = 0; i < 2000; ++i)
        {
            hnf->Undo();
            hf->Undo();
        }

        const std::pair<const char *, ::testing::AssertionResult> results[] = {
            {"H_NF", SameState(*ref, *RunOn<Kites::RV5StageProcessorHNF>(program))},
            {"H_F", SameState(*ref, *RunOn<Kites::RV5StageProcessorHF>(program))},
            {"NH_F", SameState(*ref_pad, *nhf)},
            {"NH_NF", SameState(*ref_pad2, *nhnf)},
            {"H_NF undo", SameState(*init, *hnf)},
            {"H_F undo", SameState(*init, *hf)},
        };
        std::ostringstream report;
        for (const auto &[name, ok] : results)
            if (!ok)
                report << name << ":" << ok.message() << "\n";
        if (!report.str().empty())
        {
            ++failures;
            ADD_FAILURE() << "seed " << seed << "\n" << report.str() << program;
        }
    }
}
