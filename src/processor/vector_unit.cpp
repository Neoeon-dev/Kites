/**
 * @file vector_unit.cpp
 * @brief Processor-independent RVV execution logic. See vector_unit.h.
 */
#include "processor/vector_unit.h"

#include "processor/alu.h"
#include "processor/memory_controller.h"
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

// Mask bit i lives in bit i of v0.
bool MaskBit(const RegisterFile &registers, size_t i)
{
    return ((registers.ReadVectorElement(0, i / 8, 8) >> (i % 8)) & 1) != 0;
}

// The width field of a vector load/store, as log2 of the element width in bits
// (3 = 8 ... 6 = 64), or -1 if it is not a vector width.
int EewLog2(uint32_t instruction)
{
    switch ((instruction >> 12) & 0b111)
    {
    case 0b000: return 3; // 8-bit
    case 0b101: return 4; // 16-bit
    case 0b110: return 5; // 32-bit
    case 0b111: return 6; // 64-bit
    default: return -1;
    }
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

bool IsVectorMemory(uint32_t instruction)
{
    const uint8_t opcode = instruction & 0b1111111;
    return (opcode == kOpcodeLoadFp || opcode == kOpcodeStoreFp) && EewLog2(instruction) >= 0;
}

bool IsVectorLoad(uint32_t instruction)
{
    return IsVectorMemory(instruction) && (instruction & 0b1111111) == kOpcodeLoadFp;
}

bool IsVectorStore(uint32_t instruction)
{
    return IsVectorMemory(instruction) && (instruction & 0b1111111) == kOpcodeStoreFp;
}

MemoryPlan PlanUnitStride(uint32_t instruction, const RegisterFile &registers, uint64_t base)
{
    MemoryPlan plan;
    plan.is_load = IsVectorLoad(instruction);
    plan.base = base;

    const uint8_t vreg = (instruction >> 7) & 0b11111;  // vd (load) or vs3 (store)
    const uint8_t umop = (instruction >> 20) & 0b11111; // lumop / sumop
    const bool masked = ((instruction >> 25) & 1) == 0; // vm=0 -> use v0 as mask
    const uint8_t mop = (instruction >> 26) & 0b11;
    const uint8_t mew = (instruction >> 28) & 0b1;
    const uint8_t nf = (instruction >> 29) & 0b111;

    // Only plain unit-stride is supported: no strided/indexed (mop), segment (nf),
    // whole-register / mask / fault-only-first (umop) or 128-bit+ (mew) forms.
    if (mop != 0 || mew != 0 || nf != 0 || umop != 0)
    {
        plan.error = "Unsupported vector load/store: only unit-stride vle/vse are implemented";
        return plan;
    }

    const uint64_t vl = registers.ReadCsr(kVlCsr);
    const uint64_t vtype = registers.ReadCsr(kVtypeCsr);
    if ((vtype >> 63) & 1)
    {
        plan.error = "Illegal vector instruction: vtype.vill is set (run vsetvli first)";
        return plan;
    }

    // The element width comes from the instruction (EEW), not from vtype (SEW).
    // The data group then spans EMUL = (EEW / SEW) * LMUL registers, which must be 1/8..8.
    const int eew_log2 = EewLog2(instruction);
    const int sew_log2 = 3 + static_cast<int>((vtype >> 3) & 0b111);
    const int emul_shift = eew_log2 - sew_log2 + LmulShift(vtype);
    if (emul_shift < -3 || emul_shift > 3)
    {
        plan.error = "Illegal vector instruction: EMUL = (EEW/SEW)*LMUL is out of range";
        return plan;
    }
    plan.eew = 1u << eew_log2;
    const unsigned group = emul_shift > 0 ? (1u << emul_shift) : 1u;
    const size_t elems_per_reg = architecture::VLEN_BITS / plan.eew;

    // Register groups must be aligned to the group size, and a masked load can't
    // overwrite v0 (it holds the mask).
    if (vreg % group || (plan.is_load && masked && vreg == 0))
    {
        plan.error = "Illegal vector instruction: misaligned register group or vd=v0 when masked";
        return plan;
    }

    // Load: start from the current destination values, so masked-off and tail
    // elements stay unchanged.
    if (plan.is_load)
    {
        plan.load.vd = vreg;
        plan.load.group.resize(group);
        for (unsigned r = 0; r < group; ++r)
            plan.load.group[r] = registers.ReadVreg(vreg + r);
    }

    for (size_t i = 0; i < vl; ++i)
    {
        if (masked && !MaskBit(registers, i))
            continue;
        plan.elements.push_back(static_cast<uint32_t>(i));
        if (!plan.is_load)
        {
            const size_t reg_off = i / elems_per_reg;
            const size_t idx = i % elems_per_reg;
            plan.store_data.push_back(registers.ReadVectorElement(vreg + reg_off, idx, plan.eew));
        }
    }

    plan.load.ok = plan.is_load;
    plan.ok = true;
    return plan;
}

// ---------------------------------------------------------------------------------------
// Hazard detection helpers
// ---------------------------------------------------------------------------------------
namespace
{
// Mask of `count` registers starting at `first` (clipped at v31).
uint32_t GroupMask(unsigned first, unsigned count)
{
    uint32_t mask = 0;
    for (unsigned r = first; r < first + count && r < 32; ++r)
        mask |= uint32_t{1} << r;
    return mask;
}

// Registers spanned by a group of 2^shift registers (fractional groups use one).
unsigned GroupSize(int shift)
{
    return shift > 0 ? (1u << std::min(shift, 3)) : 1u;
}
} // namespace

bool IsVectorInstruction(uint32_t instruction)
{
    return IsOpV(instruction) || IsVectorMemory(instruction);
}

bool ReadsGprRs1(uint32_t instruction)
{
    if (IsVectorMemory(instruction))
        return true; // base address
    if (!IsOpV(instruction))
        return false;

    switch ((instruction >> 12) & 0b111)
    {
    case kOpIVX:
    case kOpMVX:
        return true; // scalar operand
    case kOpCfg:
        // vsetvli / vsetvl read AVL from rs1; vsetivli has uimm5 there.
        return ((instruction >> 30) & 0b11) != 0b11;
    default:
        return false; // vs1 or imm5
    }
}

bool ReadsGprRs2(uint32_t instruction)
{
    if (IsVectorMemory(instruction))
        return ((instruction >> 26) & 0b11) == 0b10; // strided: rs2 holds the stride
    // vsetvl takes vtype from rs2
    return IsVset(instruction) && ((instruction >> 31) & 1) == 1 &&
           ((instruction >> 30) & 1) == 0;
}

bool ReadsVectorConfig(uint32_t instruction)
{
    if (IsVset(instruction))
    {
        const bool vsetivli = ((instruction >> 30) & 0b11) == 0b11;
        const uint8_t rd = (instruction >> 7) & 0x1F;
        const uint8_t rs1 = (instruction >> 15) & 0x1F;
        return !vsetivli && rd == 0 && rs1 == 0; // keep current vl
    }
    return IsVectorInstruction(instruction);
}

uint32_t VregsRead(uint32_t instruction, uint64_t vtype)
{
    if (IsVset(instruction) || !IsVectorInstruction(instruction))
        return 0;

    const bool masked = ((instruction >> 25) & 1) == 0;
    uint32_t mask = masked ? 1u : 0u; // v0 holds the mask
    const int lmul_shift = LmulShift(vtype);

    if (IsVectorMemory(instruction))
    {
        // vd of a load (old values kept for inactive elements) or vs3 of a store,
        // sized by EMUL = (EEW / SEW) * LMUL.
        const int sew_log2 = 3 + static_cast<int>((vtype >> 3) & 0b111);
        const int emul_shift = EewLog2(instruction) - sew_log2 + lmul_shift;
        return mask | GroupMask((instruction >> 7) & 0x1F, GroupSize(emul_shift));
    }

    const unsigned group = GroupSize(lmul_shift);
    const uint8_t funct3 = (instruction >> 12) & 0b111;
    mask |= GroupMask((instruction >> 20) & 0x1F, group); // vs2
    mask |= GroupMask((instruction >> 7) & 0x1F, group);  // vd (old values)
    if (funct3 == kOpIVV || funct3 == kOpMVV)
        mask |= GroupMask((instruction >> 15) & 0x1F, group); // vs1
    return mask;
}

uint32_t VregsWritten(const PipelineData &data)
{
    using Kind = PipelineData::Kind;
    if (data.kind == Kind::kArith)
        return GroupMask(data.arith.vd, static_cast<unsigned>(data.arith.group.size()));
    if (data.kind == Kind::kLoad)
        return GroupMask(data.mem.load.vd, static_cast<unsigned>(data.mem.load.group.size()));
    return 0;
}

void AccessMemory(PipelineData &data, MemoryController &memory,
                  std::vector<MemoryChange> &memory_changes)
{
    using Kind = PipelineData::Kind;
    MemoryPlan &plan = data.mem;
    const uint64_t bytes = plan.eew / 8;

    if (data.kind == Kind::kLoad)
    {
        const size_t elems_per_reg = architecture::VLEN_BITS / plan.eew;
        for (uint32_t i : plan.elements)
        {
            const uint64_t addr = plan.base + i * bytes;
            uint64_t value = 0;
            switch (plan.eew)
            {
            case 8: value = memory.readByte(addr); break;
            case 16: value = memory.readHalfWord(addr); break;
            case 32: value = memory.readWord(addr); break;
            default: value = memory.readDoubleWord(addr); break;
            }
            SetVectorElement(plan.load.group[i / elems_per_reg], i % elems_per_reg, plan.eew,
                             value);
        }
        // From here on a load is just a register-group write, committed in WB.
        data.arith = std::move(plan.load);
        data.kind = Kind::kArith;
        return;
    }

    if (data.kind == Kind::kStore)
    {
        for (size_t k = 0; k < plan.elements.size(); ++k)
        {
            const uint64_t addr = plan.base + plan.elements[k] * bytes;
            const uint64_t value = plan.store_data[k];

            // Undo bookkeeping reads use peekByte so they do not show up as cache accesses.
            std::vector<uint8_t> old_bytes(bytes);
            for (uint64_t b = 0; b < bytes; ++b)
                old_bytes[b] = memory.peekByte(addr + b);

            switch (plan.eew)
            {
            case 8: memory.writeByte(addr, static_cast<uint8_t>(value)); break;
            case 16: memory.writeHalfWord(addr, static_cast<uint16_t>(value)); break;
            case 32: memory.writeWord(addr, static_cast<uint32_t>(value)); break;
            default: memory.writeDoubleWord(addr, value); break;
            }

            std::vector<uint8_t> new_bytes(bytes);
            for (uint64_t b = 0; b < bytes; ++b)
                new_bytes[b] = memory.peekByte(addr + b);
            if (old_bytes != new_bytes)
                memory_changes.push_back({addr, std::move(old_bytes), std::move(new_bytes)});
        }
    }
}

} // namespace Kites::vector_unit
