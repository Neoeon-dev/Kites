// Vector conformance tests built from chipsalliance/riscv-vector-tests.
//
// The cases live in test/vector_conformance/generated/*.cases and are produced by
//   python3 test/vector_conformance/gen_cases.py
// (see test/vector_conformance/README.md). Each case assembles a two-instruction program
//   vsetvli t1, a0, e<SEW>, <LMUL>, tu, mu
//   <insn>  v8, v16, <v24 | a1 | imm>[, v0.t]
// with v0/v8/v16/v24/a0/a1 preloaded, runs it, and compares the whole destination
// register group and vl against the expected values.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

#include "assembler/assembler.h"
#include "processor/rv5s/rv5s_processor_nh_nf.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

#ifndef KITES_VECTOR_CONFORMANCE_DIR
#error "KITES_VECTOR_CONFORMANCE_DIR must be defined by test/CMakeLists.txt"
#endif

namespace
{
namespace fs = std::filesystem;

constexpr unsigned kVd = 8, kVs2 = 16, kVs1 = 24; // aligned for every LMUL up to m8
constexpr size_t kMaxReportedCases = 5;           // detailed reports per test
constexpr size_t kMaxReportedElements = 4;        // mismatching elements per case

// Two nops after a producer are always enough on the 5-stage NH_NF pipeline
// (no hazard detection, no forwarding); see test/vector_rv5s_test.cpp.
constexpr const char *kRv5sPad = "addi x0, x0, 0\naddi x0, x0, 0\n";

fs::path CasesDir()
{
    return fs::path(KITES_VECTOR_CONFORMANCE_DIR) / "generated";
}

// Silences std::cout / std::cerr while the VM runs: DebugRun prints every step.
class QuietScope
{
  public:
    QuietScope() : out_(std::cout.rdbuf(&null_)), err_(std::cerr.rdbuf(&null_)) {}
    ~QuietScope()
    {
        std::cout.rdbuf(out_);
        std::cerr.rdbuf(err_);
    }

  private:
    struct NullBuf : std::streambuf
    {
        int overflow(int c) override { return c; }
    } null_;
    std::streambuf *out_;
    std::streambuf *err_;
};

std::vector<uint64_t> ParseWords(const std::string &text)
{
    std::vector<uint64_t> words;
    std::istringstream in(text);
    std::string w;
    while (in >> w)
        if (w != "-")
            words.push_back(std::stoull(w, nullptr, 16));
    return words;
}

uint64_t Element(const std::vector<uint64_t> &words, size_t i, unsigned sew)
{
    const size_t bit = i * sew;
    const uint64_t mask = sew == 64 ? ~uint64_t{0} : ((uint64_t{1} << sew) - 1);
    return (words[bit / 64] >> (bit % 64)) & mask;
}

std::string Hex(uint64_t v, unsigned sew)
{
    std::ostringstream s;
    s << "0x" << std::hex << std::setw(sew / 4) << std::setfill('0') << v;
    return s.str();
}

struct ArithCase
{
    std::string line_id;
    unsigned sew = 0;
    std::string lmul;
    uint64_t avl = 0;
    bool masked = false;
    std::string operand; // "-" for .vv, hex scalar for .vx, signed imm for .vi
    uint64_t exp_vl = 0;
    unsigned nregs = 0;
    std::vector<uint64_t> v0, vd, vs2, vs1, exp;
};

std::vector<ArithCase> LoadArithCases(const fs::path &file)
{
    std::vector<ArithCase> cases;
    std::ifstream in(file);
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
            continue;
        std::vector<std::string> parts;
        size_t start = 0, bar;
        while ((bar = line.find(" | ", start)) != std::string::npos)
        {
            parts.push_back(line.substr(start, bar - start));
            start = bar + 3;
        }
        parts.push_back(line.substr(start));
        if (parts.size() != 6)
            continue;

        ArithCase c;
        std::istringstream head(parts[0]);
        int masked = 0;
        head >> c.line_id >> c.sew >> c.lmul >> c.avl >> masked >> c.operand >> c.exp_vl >>
            c.nregs;
        c.masked = masked != 0;
        c.v0 = ParseWords(parts[1]);
        c.vd = ParseWords(parts[2]);
        c.vs2 = ParseWords(parts[3]);
        c.vs1 = ParseWords(parts[4]);
        c.exp = ParseWords(parts[5]);
        cases.push_back(std::move(c));
    }
    return cases;
}

std::string ArithProgram(const std::string &insn, const ArithCase &c, const char *pad)
{
    const std::string form = insn.substr(insn.size() - 2);
    std::string third = form == "vv" ? "v24" : form == "vx" ? "a1" : c.operand;
    return "vsetvli t1, a0, e" + std::to_string(c.sew) + ", " + c.lmul + ", tu, mu\n" + pad +
           insn + " v8, v16, " + third + (c.masked ? ", v0.t" : "") + "\n";
}

void WriteGroup(Kites::RegisterFile &rf, unsigned base, const std::vector<uint64_t> &words)
{
    for (size_t r = 0; r * Kites::sizeOfVector < words.size(); ++r)
    {
        Kites::VectorRegister reg{};
        for (size_t w = 0; w < Kites::sizeOfVector; ++w)
            reg[w] = words[r * Kites::sizeOfVector + w];
        rf.WriteVreg(base + r, reg);
    }
}

std::vector<uint64_t> ReadGroup(const Kites::RegisterFile &rf, unsigned base, unsigned nregs)
{
    std::vector<uint64_t> words;
    for (unsigned r = 0; r < nregs; ++r)
        for (uint64_t w : rf.ReadVreg(base + r))
            words.push_back(w);
    return words;
}

template <typename VmType> struct VmTraits;
template <> struct VmTraits<Kites::RVSSProcessor>
{
    static constexpr const char *kPad = "";
};
template <> struct VmTraits<Kites::RV5StageProcessorNHNF>
{
    static constexpr const char *kPad = kRv5sPad;
};

// Assembling is the slow part, and many cases share a program text.
const Kites::AssembledProgram &Assemble(const std::string &source)
{
    static std::map<std::string, Kites::AssembledProgram> cache;
    auto it = cache.find(source);
    if (it == cache.end())
    {
        std::istringstream in(source);
        it = cache.emplace(source, Kites::assemble(in)).first;
    }
    return it->second;
}

template <typename VmType> void RunArithSuite(const std::string &insn)
{
    const fs::path file = CasesDir() / (insn + ".cases");
    if (!fs::exists(file))
        GTEST_SKIP() << file << " not found. Generate it with:\n"
                     << "  python3 test/vector_conformance/gen_cases.py";

    const auto cases = LoadArithCases(file);
    ASSERT_FALSE(cases.empty()) << "no cases parsed from " << file;
    Kites::setupVmStateDirectory();

    size_t failed = 0;
    for (const ArithCase &c : cases)
    {
        const std::string source = ArithProgram(insn, c, VmTraits<VmType>::kPad);
        std::vector<uint64_t> got;
        uint64_t got_vl = 0;
        {
            QuietScope quiet;
            auto vm = std::make_unique<VmType>();
            vm->LoadProgram(Assemble(source));
            auto &rf = vm->registers_;
            rf.WriteGpr(10, c.avl); // a0
            if (insn.compare(insn.size() - 3, 3, ".vx") == 0)
                rf.WriteGpr(11, std::stoull(c.operand, nullptr, 16)); // a1
            WriteGroup(rf, 0, c.v0);
            WriteGroup(rf, kVd, c.vd);
            WriteGroup(rf, kVs2, c.vs2);
            if (!c.vs1.empty())
                WriteGroup(rf, kVs1, c.vs1);
            vm->DebugRun();
            got = ReadGroup(rf, kVd, c.nregs);
            got_vl = rf.ReadCsr(0xC20);
        }

        if (got == c.exp && got_vl == c.exp_vl)
            continue;
        if (++failed > kMaxReportedCases)
            continue;

        std::ostringstream msg;
        msg << insn << " case " << c.line_id << " (line " << (std::stoul(c.line_id) + 1)
            << " of generated/" << insn << ".cases): e" << c.sew << " " << c.lmul
            << " avl=" << c.avl << (c.masked ? " masked" : " unmasked");
        if (c.operand != "-")
            msg << " operand=" << c.operand;
        msg << "\n  program:\n";
        std::istringstream prog(source);
        for (std::string l; std::getline(prog, l);)
            msg << "    " << l << "\n";
        if (got_vl != c.exp_vl)
            msg << "  vl: expected " << c.exp_vl << ", got " << got_vl << "\n";
        size_t shown = 0;
        for (size_t i = 0; i < c.exp.size() * 64 / c.sew && shown < kMaxReportedElements; ++i)
        {
            const uint64_t e = Element(c.exp, i, c.sew), g = Element(got, i, c.sew);
            if (e == g)
                continue;
            ++shown;
            msg << "  element " << i << (i < c.exp_vl ? "" : " (tail)")
                << ": vs2=" << Hex(Element(c.vs2, i, c.sew), c.sew);
            if (!c.vs1.empty())
                msg << " vs1=" << Hex(Element(c.vs1, i, c.sew), c.sew);
            if (c.masked)
                msg << " mask=" << ((c.v0[i / 64] >> (i % 64)) & 1);
            msg << " vd_before=" << Hex(Element(c.vd, i, c.sew), c.sew)
                << "  expected=" << Hex(e, c.sew) << " got=" << Hex(g, c.sew) << "\n";
        }
        ADD_FAILURE() << msg.str();
    }
    EXPECT_EQ(failed, 0u) << failed << " of " << cases.size() << " " << insn
                          << " cases failed (first " << kMaxReportedCases << " shown above)";
}

template <typename VmType> void RunVsetvliSuite()
{
    const fs::path file = CasesDir() / "vsetvli.cases";
    if (!fs::exists(file))
        GTEST_SKIP() << file << " not found. Generate it with:\n"
                     << "  python3 test/vector_conformance/gen_cases.py";
    Kites::setupVmStateDirectory();
    const std::string pad = VmTraits<VmType>::kPad;

    std::ifstream in(file);
    std::string line;
    size_t total = 0, failed = 0;
    while (std::getline(in, line))
    {
        std::istringstream f(line);
        std::string id, lmul, ta, ma;
        unsigned sew;
        uint64_t avl, vtype, exp_vl[3];
        if (!(f >> id >> sew >> lmul >> ta >> ma >> avl >> vtype >> exp_vl[0] >> exp_vl[1] >>
              exp_vl[2]))
            continue;
        const std::string cfg =
            ", e" + std::to_string(sew) + ", " + lmul + ", " + ta + ", " + ma + "\n";
        // Upstream runs the three forms in sequence; we rebuild the prefix up to form k.
        const std::string forms[3] = {"vsetvli t0, t1" + cfg, "vsetvli t0, zero" + cfg,
                                      "vsetvli zero, zero" + cfg};
        std::string source = "addi t1, x0, " + std::to_string(avl) + "\n" + pad;
        for (int k = 0; k < 3; ++k)
        {
            source += forms[k] + pad;
            ++total;
            uint64_t got_vl, got_vtype, got_t0;
            {
                QuietScope quiet;
                auto vm = std::make_unique<VmType>();
                vm->LoadProgram(Assemble(source));
                vm->DebugRun();
                got_vl = vm->registers_.ReadCsr(0xC20);
                got_vtype = vm->registers_.ReadCsr(0xC21);
                got_t0 = vm->registers_.ReadGpr(5);
            }
            // rd = vl for forms 0 and 1; form 2 writes x0, so t0 keeps form 1's value.
            const uint64_t exp_t0 = k == 0 ? exp_vl[0] : exp_vl[1];
            if (got_vl == exp_vl[k] && got_vtype == vtype && got_t0 == exp_t0)
                continue;
            if (++failed <= kMaxReportedCases)
                ADD_FAILURE() << "vsetvli case " << id << " form " << k << " (" << forms[k].substr(0, forms[k].size() - 1)
                              << ", avl=" << avl << ")\n  vl: expected " << exp_vl[k] << ", got "
                              << got_vl << "\n  vtype: expected " << vtype << ", got "
                              << got_vtype << "\n  t0 (rd): expected " << exp_t0 << ", got "
                              << got_t0;
        }
    }
    ASSERT_GT(total, 0u) << "no cases parsed from " << file;
    EXPECT_EQ(failed, 0u) << failed << " of " << total << " vsetvli checks failed";
}

const std::vector<std::string> kArithInsns = {
    "vadd.vv",  "vadd.vx",  "vadd.vi",  "vsub.vv",  "vsub.vx",  "vrsub.vx", "vrsub.vi",
    "vminu.vv", "vminu.vx", "vmin.vv",  "vmin.vx",  "vmaxu.vv", "vmaxu.vx", "vmax.vv",
    "vmax.vx",  "vand.vv",  "vand.vx",  "vand.vi",  "vor.vv",   "vor.vx",   "vor.vi",
};

std::string InsnName(const ::testing::TestParamInfo<std::string> &info)
{
    std::string n = info.param;
    for (char &ch : n)
        if (ch == '.')
            ch = '_';
    return n;
}

class VectorConformanceRVSS : public ::testing::TestWithParam<std::string>
{
};
class VectorConformanceRV5S_NH_NF : public ::testing::TestWithParam<std::string>
{
};
} // namespace

TEST_P(VectorConformanceRVSS, Upstream) { RunArithSuite<Kites::RVSSProcessor>(GetParam()); }
TEST_P(VectorConformanceRV5S_NH_NF, Upstream)
{
    RunArithSuite<Kites::RV5StageProcessorNHNF>(GetParam());
}
INSTANTIATE_TEST_SUITE_P(Arith, VectorConformanceRVSS, ::testing::ValuesIn(kArithInsns), InsnName);
INSTANTIATE_TEST_SUITE_P(Arith, VectorConformanceRV5S_NH_NF, ::testing::ValuesIn(kArithInsns),
                         InsnName);

TEST(VectorConformanceVsetvli, RVSS) { RunVsetvliSuite<Kites::RVSSProcessor>(); }
TEST(VectorConformanceVsetvli, RV5S_NH_NF) { RunVsetvliSuite<Kites::RV5StageProcessorNHNF>(); }
