/**
 * @file vector_unit.cpp
 * @brief Processor-independent RVV execution logic. See vector_unit.h.
 */
#include "processor/vector_unit.h"

#include "processor/alu.h"
#include "processor/processor_base.h"
#include "processor/processor_types.h"

#include <algorithm>

namespace Kites::vector_unit
{
namespace
{
// funct3 values of OP-V: they pick the operand kind, not the operation.
constexpr uint8_t kOpIVV = 0b000; // integer, vector-vector
constexpr uint8_t kOpMVV = 0b010; // mul/div, vector-vector
constexpr uint8_t kOpIVI = 0b011; // integer, vector-immediate
constexpr uint8_t kOpIVX = 0b100; // integer, vector-scalar
constexpr uint8_t kOpMVX = 0b110; // mul/div, vector-scalar
constexpr uint8_t kOpCfg = 0b111; // vsetvli / vsetivli / vsetvl

enum class VectorKind
{
    kAlu,     // result = alu(op, a, b)
    kRsub,    // result = b - a              (vrsub)
    kMin,     // result = (a < b) ? a : b    (op is SLT or SLTU)
    kMax,     // result = (a < b) ? b : a
    kMulHigh, // result = upper SEW bits of a * b (op is the 64-bit MULH variant)
};

struct VectorOp
{
    bool valid = false;
    VectorKind kind = VectorKind::kAlu;
    alu::AluOp op = alu::AluOp::NONE;
    bool sign_a = false; // sign-extend vs2 elements before the ALU (else zero-extend)
    bool sign_b = false; // same for vs1 / rs1 / imm
    bool shift = false;  // b is a shift amount: use only its low log2(SEW) bits
};

VectorOp DecodeVectorOp(uint8_t funct3, uint8_t funct6)
{
    using alu::AluOp;
    using K = VectorKind;
    const bool vv = funct3 == kOpIVV;
    const bool vi = funct3 == kOpIVI;

    if (funct3 == kOpIVV || funct3 == kOpIVI || funct3 == kOpIVX)
    {
        switch (funct6)
        {
        case 0b000000: return {true, K::kAlu, AluOp::ADD};                             // vadd
        case 0b000010: if (!vi) return {true, K::kAlu, AluOp::SUB}; break;             // vsub (no .vi)
        case 0b000011: if (!vv) return {true, K::kRsub, AluOp::SUB}; break;            // vrsub (no .vv)
        case 0b000100: if (!vi) return {true, K::kMin, AluOp::SLTU}; break;            // vminu
        case 0b000101: if (!vi) return {true, K::kMin, AluOp::SLT, true, true}; break; // vmin
        case 0b000110: if (!vi) return {true, K::kMax, AluOp::SLTU}; break;            // vmaxu
        case 0b000111: if (!vi) return {true, K::kMax, AluOp::SLT, true, true}; break; // vmax
        case 0b001001: return {true, K::kAlu, AluOp::AND};                             // vand
        case 0b001010: return {true, K::kAlu, AluOp::OR};                              // vor
        case 0b001011: return {true, K::kAlu, AluOp::XOR};                             // vxor
        case 0b100101: return {true, K::kAlu, AluOp::SLL, false, false, true};         // vsll
        case 0b101000: return {true, K::kAlu, AluOp::SRL, false, false, true};         // vsrl
        case 0b101001: return {true, K::kAlu, AluOp::SRA, true, false, true};          // vsra
        default: break;
        }
    }
    else if (funct3 == kOpMVV || funct3 == kOpMVX)
    {
        switch (funct6)
        {
        case 0b100101: return {true, K::kAlu, AluOp::MUL};                     // vmul
        case 0b100100: return {true, K::kMulHigh, AluOp::MULHU};               // vmulhu
        case 0b100110: return {true, K::kMulHigh, AluOp::MULHSU, true, false}; // vmulhsu
        case 0b100111: return {true, K::kMulHigh, AluOp::MULH, true, true};    // vmulh
        case 0b100000: return {true, K::kAlu, AluOp::DIVU};                    // vdivu
        case 0b100001: return {true, K::kAlu, AluOp::DIV, true, true};         // vdiv
        case 0b100010: return {true, K::kAlu, AluOp::REMU};                    // vremu
        case 0b100011: return {true, K::kAlu, AluOp::REM, true, true};         // vrem
        default: break;
        }
    }
    return {};
}

// Takes the low `bits` bits of `value` and sign- or zero-extends them to 64 bits.
uint64_t ExtendBits(uint64_t value, unsigned bits, bool is_signed)
{
    if (bits >= 64)
        return value;
    const uint64_t mask = (uint64_t{1} << bits) - 1;
    value &= mask;
    if (is_signed && (value >> (bits - 1)) & 1)
        value |= ~mask;
    return value;
}

// Writes one SEW-wide element into a VectorRegister value, same layout as
// RegisterFile::WriteVectorElement (little-endian, packed into uint64_t words).
void SetVectorElement(VectorRegister &reg, size_t index, unsigned sew, uint64_t value)
{
    const size_t byte_offset = index * (sew / 8);
    const size_t word = byte_offset / sizeof(uint64_t);
    const size_t bit = (byte_offset % sizeof(uint64_t)) * 8;
    const uint64_t mask = sew == 64 ? UINT64_MAX : ((uint64_t{1} << sew) - 1);
    reg[word] = (reg[word] & ~(mask << bit)) | ((value & mask) << bit);
}

// vlmul is a signed 3-bit field: mf8 = -3 ... m8 = 3 (-4 is reserved).
int LmulShift(uint64_t vtype)
{
    int lmul_shift = static_cast<int>(vtype & 0b111);
    if (lmul_shift >= 4)
        lmul_shift -= 8;
    return lmul_shift;
}
} // namespace

bool IsOpV(uint32_t instruction)
{
    return (instruction & 0b1111111) == kOpcodeOpV;
}

bool IsVset(uint32_t instruction)
{
    return IsOpV(instruction) && ((instruction >> 12) & 0b111) == kOpCfg;
}

VsetResult ComputeVset(uint32_t instruction, uint64_t rs1_value, uint64_t rs2_value,
                       uint64_t current_vl)
{
    const uint8_t rd = (instruction >> 7) & 0x1F;
    const uint8_t rs1 = (instruction >> 15) & 0x1F;
    uint64_t vtype;
    uint64_t avl;

    if (((instruction >> 31) & 1) == 0)
    { // vsetvli
        vtype = (instruction >> 20) & 0x7FF;
        avl = rs1 ? rs1_value : (rd ? UINT64_MAX : current_vl);
    }
    else if (((instruction >> 30) & 0b11) == 0b11)
    { // vsetivli
        vtype = (instruction >> 20) & 0x3FF;
        avl = rs1; // uimm[4:0]
    }
    else
    { // vsetvl
        vtype = rs2_value;
        avl = rs1 ? rs1_value : (rd ? UINT64_MAX : current_vl);
    }

    const unsigned vsew = (vtype >> 3) & 0b111;
    const int lmul_shift = LmulShift(vtype);
    const unsigned sew = 8u << vsew;

    const bool ill = vsew > 3 || sew > architecture::ELEN_BITS ||
                     lmul_shift == -4 || // vlmul=100 is reserved
                     (lmul_shift < 0 && sew > (architecture::ELEN_BITS >> -lmul_shift)) ||
                     (vtype >> 8) != 0; // reserved bits must be 0

    if (ill)
        return {uint64_t{1} << 63, 0};

    const uint64_t base = architecture::VLEN_BITS / sew;
    const uint64_t vlmax = lmul_shift >= 0 ? base << lmul_shift : base >> -lmul_shift;
    return {vtype, std::min(avl, vlmax)};
}

ArithResult ExecuteArithmetic(uint32_t instruction, const RegisterFile &registers,
                              uint64_t scalar)
{
    ArithResult result;

    const uint8_t funct6 = (instruction >> 26) & 0b111111;
    const uint8_t funct3 = (instruction >> 12) & 0b111;
    const uint8_t vd = (instruction >> 7) & 0b11111;
    const uint8_t vs1 = (instruction >> 15) & 0b11111; // also rs1 / imm5
    const uint8_t vs2 = (instruction >> 20) & 0b11111;
    const bool masked = ((instruction >> 25) & 1) == 0; // vm=0 -> use v0 as mask

    const uint64_t vl = registers.ReadCsr(kVlCsr);
    const uint64_t vtype = registers.ReadCsr(kVtypeCsr);

    if ((vtype >> 63) & 1)
    {
        result.error = "Illegal vector instruction: vtype.vill is set (run vsetvli first)";
        return result;
    }

    const unsigned sew = 8u << ((vtype >> 3) & 0b111);
    const int lmul_shift = LmulShift(vtype);
    // A register group spans 2^lmul_shift registers; fractional LMUL still uses one register.
    const unsigned group = lmul_shift > 0 ? (1u << lmul_shift) : 1u;
    const size_t elems_per_reg = architecture::VLEN_BITS / sew;

    const VectorOp vop = DecodeVectorOp(funct3, funct6);
    if (!vop.valid)
    {
        result.error = "Unsupported vector operation: funct6=" + std::to_string(funct6) +
                       " funct3=" + std::to_string(funct3);
        return result;
    }

    // Register groups must be aligned to the group size, and v0 can't be the
    // destination of a masked instruction (it holds the mask).
    const bool vv = funct3 == kOpIVV || funct3 == kOpMVV;
    if (vd % group || vs2 % group || (vv && vs1 % group) || (masked && vd == 0))
    {
        result.error = "Illegal vector instruction: misaligned register group or vd=v0 when masked";
        return result;
    }

    // The scalar operand for .vx / .vi forms.
    uint64_t b_scalar = 0;
    if (funct3 == kOpIVX || funct3 == kOpMVX)
        b_scalar = scalar;
    else if (funct3 == kOpIVI)
        b_scalar = vop.shift ? vs1                        // shifts use uimm5
                             : ExtendBits(vs1, 5, true);  // others use simm5

    // Start from the current destination values: masked-off and tail elements stay unchanged.
    result.vd = vd;
    result.group.resize(group);
    for (unsigned r = 0; r < group; ++r)
        result.group[r] = registers.ReadVreg(vd + r);

    for (size_t i = 0; i < vl; ++i)
    {
        // Mask bit i lives in bit i of v0.
        if (masked && ((registers.ReadVectorElement(0, i / 8, 8) >> (i % 8)) & 1) == 0)
            continue;

        const size_t reg_off = i / elems_per_reg; // which register in the group
        const size_t idx = i % elems_per_reg;     // which element in that register

        const uint64_t a =
            ExtendBits(registers.ReadVectorElement(vs2 + reg_off, idx, sew), sew, vop.sign_a);
        uint64_t b = vv ? registers.ReadVectorElement(vs1 + reg_off, idx, sew) : b_scalar;
        b = ExtendBits(b, sew, vop.sign_b);
        if (vop.shift)
            b &= sew - 1; // only log2(SEW) bits of the shift amount count

        uint64_t res = 0;
        switch (vop.kind)
        {
        case VectorKind::kAlu:
            res = alu::Alu::execute(vop.op, a, b).first;
            break;
        case VectorKind::kRsub:
            res = alu::Alu::execute(alu::AluOp::SUB, b, a).first;
            break;
        case VectorKind::kMin:
            res = alu::Alu::execute(vop.op, a, b).first ? a : b;
            break;
        case VectorKind::kMax:
            res = alu::Alu::execute(vop.op, a, b).first ? b : a;
            break;
        case VectorKind::kMulHigh:
            if (sew == 64)
                res = alu::Alu::execute(vop.op, a, b).first;
            else // SEW <= 32: the full product fits in 64 bits, so shift it down ourselves
                res = static_cast<uint64_t>(static_cast<int64_t>(a * b) >> sew);
            break;
        }

        SetVectorElement(result.group[reg_off], idx, sew, res); // truncates to SEW bits
    }

    result.ok = true;
    return result;
}

void CommitVset(RegisterFile &registers, uint8_t rd, const VsetResult &result,
                std::vector<RegisterChange> &changes)
{
    const uint64_t old_vtype = registers.ReadCsr(kVtypeCsr);
    registers.WriteCsr(kVtypeCsr, result.vtype);
    if (old_vtype != result.vtype)
        changes.push_back({static_cast<unsigned int>(kVtypeCsr), 1, old_vtype, result.vtype});

    const uint64_t old_vl = registers.ReadCsr(kVlCsr);
    registers.WriteCsr(kVlCsr, result.vl);
    if (old_vl != result.vl)
        changes.push_back({static_cast<unsigned int>(kVlCsr), 1, old_vl, result.vl});

    // rd = new vl (WriteGpr ignores x0)
    const uint64_t old_gpr = registers.ReadGpr(rd);
    registers.WriteGpr(rd, result.vl);
    const uint64_t new_gpr = registers.ReadGpr(rd);
    if (old_gpr != new_gpr)
        changes.push_back({rd, 0, old_gpr, new_gpr});
}

void CommitGroup(RegisterFile &registers, const ArithResult &result,
                 std::vector<RegisterChange> &changes)
{
    for (size_t r = 0; r < result.group.size(); ++r)
    {
        const size_t reg = result.vd + r;
        const VectorRegister old_value = registers.ReadVreg(reg);
        const VectorRegister &new_value = result.group[r];
        registers.WriteVreg(reg, new_value);
        for (size_t w = 0; w < sizeOfVector; ++w)
        {
            if (old_value[w] != new_value[w])
            {
                changes.push_back({static_cast<unsigned int>(reg * sizeOfVector + w),
                                   kRegTypeVectorWord, old_value[w], new_value[w]});
            }
        }
    }
}

void ApplyVectorWord(RegisterFile &registers, unsigned reg_index, uint64_t value)
{
    const size_t reg = reg_index / sizeOfVector;
    VectorRegister v = registers.ReadVreg(reg);
    v[reg_index % sizeOfVector] = value;
    registers.WriteVreg(reg, v);
}

} // namespace Kites::vector_unit
