/**
 * @file rvss_vm.cpp
 * @brief RVSS VM implementation
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#include "processor/rvss/rvss_processor.h"

#include "common/instructions.h"
#include "config/config.h"
#include "common/globals.h"
#include "ui/processor_tab/processor_designs/rvss_processor_circuit_scene.h"
#include "utils/utils.h"
#include <QDebug>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>
#include <tuple>

namespace Kites
{
static inline uint64_t sign_extend(uint64_t value, unsigned bits)
{
    if (bits == 0 || bits >= 64)
    {
        return value;
    }

    const uint64_t sign_bit = UINT64_C(1) << (bits - 1);
    const uint64_t mask = (UINT64_C(1) << bits) - 1U;
    return (value & sign_bit) ? (value | ~mask) : value;
}

RVSSProcessor::RVSSProcessor() : ProcessorBase()
{
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
#ifndef DISABLE_GUI
    circuit_scene_ = std::make_unique<Kites::RVSSCircuitScene>();
    connect(this, &ProcessorBase::updateCircuitStateSignal, circuit_scene_.get(),
            &Kites::RVSSCircuitScene::updateCircuitState, Qt::QueuedConnection);
#endif

    active_wires_.append("IM_to_PC_pc");
    active_wires_.append("PC_to_IM_instruction");
    active_wires_.append("IM_to_RF_rs1_whole");
    active_wires_.append("RF_to_ALUMUX");
    active_wires_.append("PCMux_to_PC");
    active_wires_.append("Imm_to_ALUMUX");
    active_wires_.append("BranchALU_to_PCMUX");
    active_wires_.append("PCALU_to_PCMUX");
    active_wires_.append("Control_ALUOp");
    active_wires_.append("PC_to_InstrMem");
    active_wires_.append("ALUOp_to_ALU_alusignal");
    active_wires_.append("ALU_to_DM");

    always_active_wires_count_ = active_wires_.size();
}

RVSSProcessor::~RVSSProcessor() = default;

void RVSSProcessor::SetActiveWireNames()
{
    // first clear the current list
    // TODO * we dont have to clear wire that will always be active
    //  so make a list of those wires and only clear the rest
    //  for now this will
    // active_wires_.clear();

    // Map control signals to canonical wire names used in the scene JSON.
    // maybe sending integers would be better?
    // but for now, this is fine.
    // also the preformace boost well get of that will that really improve the user experience that
    // much?

    // first add the always active wires
    // this is not ideal but for now it will do

    // if(branch_flag_)
    // {

    // }

    if (control_unit_.GetMemWrite())
    {
        active_wires_.append("Control_to_MemWrite");
    }
    if (control_unit_.GetMemRead())
    {
        active_wires_.append("Control_to_MemRead");
    }
    if (control_unit_.GetRegWrite())
    {
        active_wires_.append("Control_to_Regwrite");
        active_wires_.append("WBMux_to_RF_lowest");
    }
    if (control_unit_.GetBranch())
    {
        active_wires_.append("Control_to_BranchAND");
    }
    if (control_unit_.GetAluSrc())
    {
        active_wires_.append("Control_to_ALUSrc_control");
    }
    if (control_unit_.GetMemToReg())
    {
        active_wires_.append("Control_to_MemtoRegMux");
    }

}

void RVSSProcessor::setProcessorState()
{
    processor_state_.programCounters = 
    {program_counter_,INVALID_PC,INVALID_PC,INVALID_PC,INVALID_PC};
    processor_state_.lastExecutedPC = last_executed_pc_;
}

void RVSSProcessor::Run()
{
    ClearStop();
    while (!stop_requested_ && program_counter_ < program_size_)
    {
        if (last_breakpoint_pc_ != UINT64_MAX && program_counter_ != last_breakpoint_pc_)
        {
            last_breakpoint_pc_ = UINT64_MAX;
        }

        if (std::find(breakpoints_.begin(), breakpoints_.end(), program_counter_) !=
                breakpoints_.end() &&
            program_counter_ != last_breakpoint_pc_)
        {
            pause_requested_ = true;
            last_breakpoint_pc_ = program_counter_;
            emit processorPausedAtBreakpointSignal();
        }
        {
            QMutexLocker locker(&pause_mutex_);
            while (pause_requested_ && !stop_requested_)
            {
                pause_wait_condition_.wait(&pause_mutex_);
            }
            if (stop_requested_)
            {
                break;
            }
        }
        StepAndNotify();
        // handling the delay
        {
            QMutexLocker locker(&pause_mutex_);
            if (stop_requested_ || pause_requested_)
                continue;
            pause_wait_condition_.wait(&pause_mutex_, step_delay_);
            // we wait for step_delay_ milliseconds or until notified to wake up
        }
    }
    if (stop_requested_)
    {
        emit processorClockedSignal(processor_state_);
    }
    if (program_counter_ >= program_size_)
    {
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSProcessor::Fetch()
{
    current_instruction_ = memory_controller_.readInstruction(program_counter_);
    UpdateProgramCounter(4);
}

void RVSSProcessor::Decode()
{
    control_unit_.SetControlSignals(current_instruction_);
}

void RVSSProcessor::Execute()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    if (opcode == 0b1110011 && funct3 == 0b000)
    {
        HandleSyscall();
        return;
    }

    if (instruction_set::isFInstruction(current_instruction_))
    { // RV64 F
        ExecuteFloat();
        return;
    }
    else if (instruction_set::isDInstruction(current_instruction_))
    {
        ExecuteDouble();
        return;
    }
    else if (opcode == 0b1110011)
    {
        ExecuteCsr();
        return;
    }
    else if (opcode == 0b1010111){
        // RV64 V
        ExecuteVectorArithmetic();
        return;
    }

    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

    int32_t imm = ImmGenerator(current_instruction_);

    uint64_t reg1_value = registers_.ReadGpr(rs1);
    uint64_t reg2_value = registers_.ReadGpr(rs2);

    bool overflow = false;

    if (control_unit_.GetAluSrc())
    {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation =
        control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());
    std::tie(execution_result_, overflow) = alu_.execute(aluOperation, reg1_value, reg2_value);

    if (control_unit_.GetBranch())
    {
        if (opcode == 0b1100111 || opcode == 0b1101111)
        {                                                      // JALR or JAL
            next_pc_ = static_cast<int64_t>(program_counter_); // PC was already updated in Fetch()
            UpdateProgramCounter(-4);
            return_address_ = program_counter_ + 4;
            if (opcode == 0b1100111)
            { // JALR
                UpdateProgramCounter(-program_counter_ + (execution_result_));
            }
            else
            { // JAL
                UpdateProgramCounter(imm);
            }
        }
        else if (opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbeq)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbne)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kblt)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbge)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbltu)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbgeu)
                               .opcode)
        {
            switch (funct3)
            {
            case 0b000:
            { // BEQ
                branch_flag_ = (execution_result_ == 0);
                break;
            }
            case 0b001:
            { // BNE
                branch_flag_ = (execution_result_ != 0);
                break;
            }
            case 0b100:
            { // BLT
                branch_flag_ = (execution_result_ == 1);
                break;
            }
            case 0b101:
            { // BGE
                branch_flag_ = (execution_result_ == 0);
                break;
            }
            case 0b110:
            { // BLTU
                branch_flag_ = (execution_result_ == 1);
                break;
            }
            case 0b111:
            { // BGEU
                branch_flag_ = (execution_result_ == 0);
                break;
            }
            }
        }
    }

    if (branch_flag_ &&
        (opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbeq)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbne)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kblt)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbge)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbltu)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbgeu)
                       .opcode))
    {
        UpdateProgramCounter(-4);
        UpdateProgramCounter(imm);
        // we have jumped so the branch alu wire will send signal to the pc mux
        active_wires_.append("ALUzero_to_ANDGATElower");
        active_wires_.append("BranchAND_to_PCMUX");
    }

    if (opcode == 0b0010111)
    { // AUIPC
        execution_result_ = static_cast<int64_t>(program_counter_) - 4 + (imm << 12);
    }
}

void RVSSProcessor::ExecuteFloat()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rm = funct3;
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
    uint8_t rs3 = (current_instruction_ >> 27) & 0b11111;

    uint8_t fcsr_status = 0;

    int32_t imm = ImmGenerator(current_instruction_);

    if (rm == 0b111)
    {
        rm = registers_.ReadCsr(0x002);
    }

    uint64_t reg1_value = registers_.ReadFpr(rs1);
    uint64_t reg2_value = registers_.ReadFpr(rs2);
    uint64_t reg3_value = registers_.ReadFpr(rs3);

    if (funct7 == 0b1101000 || funct7 == 0b1111000 || opcode == 0b0000111 || opcode == 0b0100111)
    {
        reg1_value = registers_.ReadGpr(rs1);
    }

    if (control_unit_.GetAluSrc())
    {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation =
        control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());
    std::tie(execution_result_, fcsr_status) =
        alu::Alu::fpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);

    registers_.WriteCsr(0x003, fcsr_status);
}

void RVSSProcessor::ExecuteDouble()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rm = funct3;
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
    uint8_t rs3 = (current_instruction_ >> 27) & 0b11111;

    uint8_t fcsr_status = 0;

    int32_t imm = ImmGenerator(current_instruction_);

    uint64_t reg1_value = registers_.ReadFpr(rs1);
    uint64_t reg2_value = registers_.ReadFpr(rs2);
    uint64_t reg3_value = registers_.ReadFpr(rs3);

    if (funct7 == 0b1101001 || funct7 == 0b1111001 || opcode == 0b0000111 || opcode == 0b0100111)
    {
        reg1_value = registers_.ReadGpr(rs1);
    }

    if (control_unit_.GetAluSrc())
    {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation =
        control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());
    std::tie(execution_result_, fcsr_status) =
        alu::Alu::dfpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);
}

void RVSSProcessor::ExecuteCsr()
{
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint16_t csr = (current_instruction_ >> 20) & 0xFFF;
    uint64_t csr_val = registers_.ReadCsr(csr);

    csr_target_address_ = csr;
    csr_old_value_ = csr_val;
    csr_write_val_ = registers_.ReadGpr(rs1);
    csr_uimm_ = rs1;
}

uint64_t vset_new_vl_{};
uint64_t vset_new_vtype_{};

void RVSSProcessor::ExecuteVsetType()
{
    uint8_t rd  = (current_instruction_ >> 7)  & 0x1F;
    uint8_t rs1 = (current_instruction_ >> 15) & 0x1F;
    uint64_t vtype;
    uint64_t avl;

    if (((current_instruction_ >> 31) & 1) == 0) {               // vsetvli
        vtype = (current_instruction_ >> 20) & 0x7FF;
        avl = rs1 ? registers_.ReadGpr(rs1) : (rd ? UINT64_MAX : registers_.ReadCsr(0xC20));
    } else if (((current_instruction_ >> 30) & 0b11) == 0b11) {  // vsetivli
        vtype = (current_instruction_ >> 20) & 0x3FF;
        avl = rs1;                                                // uimm[4:0]
    } else {                                                      // vsetvl
        vtype = registers_.ReadGpr((current_instruction_ >> 20) & 0x1F);
        avl = rs1 ? registers_.ReadGpr(rs1) : (rd ? UINT64_MAX : registers_.ReadCsr(0xC20));
    }

    unsigned vsew = (vtype >> 3) & 0b111;
    int lmul_shift = static_cast<int>(vtype & 0b111);
    if (lmul_shift >= 4) lmul_shift -= 8;                         // sign-extend: mf8=-3 ... m8=3
    unsigned sew = 8u << vsew;

    bool ill = vsew > 3 || sew > architecture::ELEN_BITS
            || lmul_shift == -4                                   // vlmul=100 is reserved
            || (lmul_shift < 0 && sew > (architecture::ELEN_BITS >> -lmul_shift))
            || (vtype >> 8) != 0;                                // reserved bits must be 0

    if (ill) {
        vset_new_vtype_ = 1ULL << 63;
        vset_new_vl_ = 0;
        return;
    }

    uint64_t base = architecture::VLEN_BITS / sew;
    uint64_t vlmax = lmul_shift >= 0 ? base << lmul_shift : base >> -lmul_shift;
    vset_new_vtype_ = vtype;
    vset_new_vl_ = std::min(avl, vlmax);
}

//multi-cycle simd
namespace
{
// funct3 values of OP-V (opcode 1010111): they pick the operand kind, not the operation.
constexpr uint8_t kOpIVV = 0b000; // integer, vector-vector
constexpr uint8_t kOpMVV = 0b010; // mul/div, vector-vector
constexpr uint8_t kOpIVI = 0b011; // integer, vector-immediate
constexpr uint8_t kOpIVX = 0b100; // integer, vector-scalar
constexpr uint8_t kOpMVX = 0b110; // mul/div, vector-scalar

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
        case 0b000000: return {true, K::kAlu, AluOp::ADD};                         // vadd
        case 0b000010: if (!vi) return {true, K::kAlu, AluOp::SUB}; break;         // vsub (no .vi)
        case 0b000011: if (!vv) return {true, K::kRsub, AluOp::SUB}; break;        // vrsub (no .vv)
        case 0b000100: if (!vi) return {true, K::kMin, AluOp::SLTU}; break;        // vminu
        case 0b000101: if (!vi) return {true, K::kMin, AluOp::SLT, true, true}; break; // vmin
        case 0b000110: if (!vi) return {true, K::kMax, AluOp::SLTU}; break;        // vmaxu
        case 0b000111: if (!vi) return {true, K::kMax, AluOp::SLT, true, true}; break; // vmax
        case 0b001001: return {true, K::kAlu, AluOp::AND};                         // vand
        case 0b001010: return {true, K::kAlu, AluOp::OR};                          // vor
        case 0b001011: return {true, K::kAlu, AluOp::XOR};                         // vxor
        case 0b100101: return {true, K::kAlu, AluOp::SLL, false, false, true};     // vsll
        case 0b101000: return {true, K::kAlu, AluOp::SRL, false, false, true};     // vsrl
        case 0b101001: return {true, K::kAlu, AluOp::SRA, true, false, true};      // vsra
        default: break;
        }
    }
    else if (funct3 == kOpMVV || funct3 == kOpMVX)
    {
        switch (funct6)
        {
        case 0b100101: return {true, K::kAlu, AluOp::MUL};                         // vmul
        case 0b100100: return {true, K::kMulHigh, AluOp::MULHU};                   // vmulhu
        case 0b100110: return {true, K::kMulHigh, AluOp::MULHSU, true, false};     // vmulhsu
        case 0b100111: return {true, K::kMulHigh, AluOp::MULH, true, true};        // vmulh
        case 0b100000: return {true, K::kAlu, AluOp::DIVU};                        // vdivu
        case 0b100001: return {true, K::kAlu, AluOp::DIV, true, true};             // vdiv
        case 0b100010: return {true, K::kAlu, AluOp::REMU};                        // vremu
        case 0b100011: return {true, K::kAlu, AluOp::REM, true, true};             // vrem
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
} // namespace

void RVSSProcessor::ExecuteVectorArithmetic()
{
    vector_result_regs_ = 0; // nothing to write back unless we finish successfully

    const uint8_t funct6 = (current_instruction_ >> 26) & 0b111111;
    const uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    if (funct3 == 0b111)
    { // vsetvli / vsetivli / vsetvl
        ExecuteVsetType();
        return;
    }

    const uint8_t vd = (current_instruction_ >> 7) & 0b11111;
    const uint8_t vs1 = (current_instruction_ >> 15) & 0b11111; // also rs1 / imm5
    const uint8_t vs2 = (current_instruction_ >> 20) & 0b11111;
    const bool masked = ((current_instruction_ >> 25) & 1) == 0; // vm=0 -> use v0 as mask

    const uint64_t vl = registers_.ReadCsr(0xC20);
    const uint64_t vtype = registers_.ReadCsr(0xC21);

    if ((vtype >> 63) & 1)
    {
        std::cerr << "Illegal vector instruction: vtype.vill is set (run vsetvli first)"
                  << std::endl;
        return;
    }

    const unsigned sew = 8u << ((vtype >> 3) & 0b111);
    int lmul_shift = static_cast<int>(vtype & 0b111);
    if (lmul_shift >= 4)
        lmul_shift -= 8; // mf8=-3 ... m8=3
    // A register group spans 2^lmul_shift registers; fractional LMUL still uses one register.
    const unsigned group = lmul_shift > 0 ? (1u << lmul_shift) : 1u;
    const size_t elems_per_reg = architecture::VLEN_BITS / sew;

    const VectorOp vop = DecodeVectorOp(funct3, funct6);
    if (!vop.valid)
    {
        std::cerr << "Unsupported vector operation: funct6=" << static_cast<int>(funct6)
                  << " funct3=" << static_cast<int>(funct3) << std::endl;
        return;
    }

    // Register groups must be aligned to the group size, and v0 can't be the
    // destination of a masked instruction (it holds the mask).
    const bool vv = funct3 == kOpIVV || funct3 == kOpMVV;
    if (vd % group || vs2 % group || (vv && vs1 % group) || (masked && vd == 0))
    {
        std::cerr << "Illegal vector instruction: misaligned register group or vd=v0 when masked"
                  << std::endl;
        return;
    }

    // The scalar operand for .vx / .vi forms.
    uint64_t scalar = 0;
    if (funct3 == kOpIVX || funct3 == kOpMVX)
        scalar = registers_.ReadGpr(vs1);
    else if (funct3 == kOpIVI)
        scalar = vop.shift ? vs1                         // shifts use uimm5
                           : ExtendBits(vs1, 5, true);   // others use simm5

    // Start from the current destination values: masked-off and tail elements stay unchanged.
    for (unsigned r = 0; r < group; ++r)
        vector_result_[r] = registers_.ReadVreg(vd + r);

    for (size_t i = 0; i < vl; ++i)
    {
        // Mask bit i lives in bit i of v0.
        if (masked && ((registers_.ReadVectorElement(0, i / 8, 8) >> (i % 8)) & 1) == 0)
            continue;

        const size_t reg_off = i / elems_per_reg; // which register in the group
        const size_t idx = i % elems_per_reg;     // which element in that register

        uint64_t a = ExtendBits(registers_.ReadVectorElement(vs2 + reg_off, idx, sew), sew,
                                vop.sign_a);
        uint64_t b = vv ? registers_.ReadVectorElement(vs1 + reg_off, idx, sew) : scalar;
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

        SetVectorElement(vector_result_[reg_off], idx, sew, res); // truncates to SEW bits
    }

    vector_result_regs_ = static_cast<uint8_t>(group);
}

// TODO: implement writeback for syscalls
void RVSSProcessor::HandleSyscall()
{
    uint64_t syscall_number = registers_.ReadGpr(17);
    switch (syscall_number)
    {
    case SYSCALL_PRINT_INT:
    {
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        else
        {
            std::cout << "VM_STDOUT_START";
        }
        std::cout << static_cast<int64_t>(registers_.ReadGpr(10)); // Print signed integer
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        else
        {
            std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_FLOAT:
    { // print float
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        else
        {
            std::cout << "VM_STDOUT_START";
        }
        float float_value;
        uint64_t raw = registers_.ReadGpr(10);
        std::memcpy(&float_value, &raw, sizeof(float_value));
        std::cout << std::setprecision(std::numeric_limits<float>::max_digits10) << float_value;
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        else
        {
            std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_DOUBLE:
    { // print double
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        else
        {
            std::cout << "VM_STDOUT_START";
        }
        double double_value;
        uint64_t raw = registers_.ReadGpr(10);
        std::memcpy(&double_value, &raw, sizeof(double_value));
        std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << double_value;
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        else
        {
            std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_STRING:
    {
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        PrintString(registers_.ReadGpr(10)); // Print string
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        break;
    }
    case SYSCALL_EXIT:
    {
        stop_requested_ = true; // Stop the VM
        if (!globals::vm_as_backend)
        {
            std::cout << "VM_EXIT" << std::endl;
        }
        output_status_ = "VM_EXIT";
        std::cout << "Exited with exit code: " << registers_.ReadGpr(10) << std::endl;
        exit(0); // Exit the program
        break;
    }
    case SYSCALL_READ:
    { // Read
        uint64_t file_descriptor = registers_.ReadGpr(10);
        uint64_t buffer_address = registers_.ReadGpr(11);
        uint64_t length = registers_.ReadGpr(12);

        if (file_descriptor == 0)
        {
            // Read from stdin
            std::string input;
            {
                std::cout << "VM_STDIN_START" << std::endl;
                output_status_ = "VM_STDIN_START";
                std::unique_lock<std::mutex> lock(input_mutex_);
                input_cv_.wait(lock, [this]() { return !input_queue_.empty(); });
                output_status_ = "VM_STDIN_END";
                std::cout << "VM_STDIN_END" << std::endl;

                input = input_queue_.front();
                input_queue_.pop();
            }

            std::vector<uint8_t> old_bytes_vec(length, 0);
            std::vector<uint8_t> new_bytes_vec(length, 0);

            for (size_t i = 0; i < length; ++i)
            {
                old_bytes_vec[i] = memory_controller_.peekByte(buffer_address + i);
            }

            for (size_t i = 0; i < input.size() && i < length; ++i)
            {
                memory_controller_.writeByte(buffer_address + i, static_cast<uint8_t>(input[i]));
            }
            if (input.size() < length)
            {
                memory_controller_.writeByte(buffer_address + input.size(), '\0');
            }

            for (size_t i = 0; i < length; ++i)
            {
                new_bytes_vec[i] = memory_controller_.peekByte(buffer_address + i);
            }

            current_delta_.memory_changes.push_back({buffer_address, old_bytes_vec, new_bytes_vec});

            uint64_t old_reg = registers_.ReadGpr(10);
            unsigned int reg_index = 10;
            unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
            uint64_t new_reg =
                std::min(static_cast<uint64_t>(length), static_cast<uint64_t>(input.size()));
            registers_.WriteGpr(10, new_reg);
            if (old_reg != new_reg)
            {
                current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
            }
        }
        else
        {
            std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
        }
        break;
    }
    case SYSCALL_WRITE:
    { // Write
        uint64_t file_descriptor = registers_.ReadGpr(10);
        uint64_t buffer_address = registers_.ReadGpr(11);
        uint64_t length = registers_.ReadGpr(12);

        if (file_descriptor == 1)
        { // stdout
            std::cout << "VM_STDOUT_START";
            output_status_ = "VM_STDOUT_START";
            uint64_t bytes_printed = 0;
            for (uint64_t i = 0; i < length; ++i)
            {
                char c = memory_controller_.readByte(buffer_address + i);
                // if (c == '\0') {
                //     break;
                // }
                std::cout << c;
                bytes_printed++;
            }
            std::cout << std::flush;
            output_status_ = "VM_STDOUT_END";
            std::cout << "VM_STDOUT_END" << std::endl;

            uint64_t old_reg = registers_.ReadGpr(10);
            unsigned int reg_index = 10;
            unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
            uint64_t new_reg = std::min(static_cast<uint64_t>(length), bytes_printed);
            registers_.WriteGpr(10, new_reg);
            if (old_reg != new_reg)
            {
                current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
            }
        }
        else
        {
            std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
        }
        break;
    }
    default:
    {
        std::cerr << "Unknown syscall number: " << syscall_number << std::endl;
        break;
    }
    }
}

void RVSSProcessor::WriteMemory()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    if (opcode == 0b1110011 && funct3 == 0b000)
    {
        return;
    }

    if (instruction_set::isFInstruction(current_instruction_))
    { // RV64 F
        WriteMemoryFloat();
        return;
    }
    else if (instruction_set::isDInstruction(current_instruction_))
    {
        WriteMemoryDouble();
        return;
    }

    if (control_unit_.GetMemRead())
    {
        switch (funct3)
        {
        case 0b000:
        { // LB
            memory_result_ = static_cast<int8_t>(memory_controller_.readByte(execution_result_));
            break;
        }
        case 0b001:
        { // LH
            memory_result_ =
                static_cast<int16_t>(memory_controller_.readHalfWord(execution_result_));
            break;
        }
        case 0b010:
        { // LW
            memory_result_ = static_cast<int32_t>(memory_controller_.readWord(execution_result_));
            break;
        }
        case 0b011:
        { // LD
            memory_result_ = memory_controller_.readDoubleWord(execution_result_);
            break;
        }
        case 0b100:
        { // LBU
            memory_result_ = static_cast<uint8_t>(memory_controller_.readByte(execution_result_));
            break;
        }
        case 0b101:
        { // LHU
            memory_result_ =
                static_cast<uint16_t>(memory_controller_.readHalfWord(execution_result_));
            break;
        }
        case 0b110:
        { // LWU
            memory_result_ = static_cast<uint32_t>(memory_controller_.readWord(execution_result_));
            break;
        }
        }
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    // Undo bookkeeping reads use peekByte so they do not show up as cache accesses.

    if (control_unit_.GetMemWrite())
    {
        switch (funct3)
        {
        case 0b000:
        { // SB
            addr = execution_result_;
            old_bytes_vec.push_back(memory_controller_.peekByte(addr));
            memory_controller_.writeByte(execution_result_, registers_.ReadGpr(rs2) & 0xFF);
            new_bytes_vec.push_back(memory_controller_.peekByte(addr));
            break;
        }
        case 0b001:
        { // SH
            addr = execution_result_;
            for (size_t i = 0; i < 2; ++i)
            {
                old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            memory_controller_.writeHalfWord(execution_result_, registers_.ReadGpr(rs2) & 0xFFFF);
            for (size_t i = 0; i < 2; ++i)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            break;
        }
        case 0b010:
        { // SW
            addr = execution_result_;
            for (size_t i = 0; i < 4; ++i)
            {
                old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            memory_controller_.writeWord(execution_result_, registers_.ReadGpr(rs2) & 0xFFFFFFFF);
            for (size_t i = 0; i < 4; ++i)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            break;
        }
        case 0b011:
        { // SD
            addr = execution_result_;
            for (size_t i = 0; i < 8; ++i)
            {
                old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            memory_controller_.writeDoubleWord(execution_result_,
                                               registers_.ReadGpr(rs2) & 0xFFFFFFFFFFFFFFFF);
            for (size_t i = 0; i < 8; ++i)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            break;
        }
        }
    }

    if (old_bytes_vec != new_bytes_vec)
    {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }
}

void RVSSProcessor::WriteMemoryFloat()
{
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

    if (control_unit_.GetMemRead())
    { // FLW
        memory_result_ = memory_controller_.readWord(execution_result_);
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    if (control_unit_.GetMemWrite())
    { // FSW
        addr = execution_result_;
        for (size_t i = 0; i < 4; ++i)
        {
            old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
        uint32_t val = registers_.ReadFpr(rs2) & 0xFFFFFFFF;
        memory_controller_.writeWord(execution_result_, val);
        // new_bytes_vec.push_back(memory_controller_.ReadByte(addr));
        for (size_t i = 0; i < 4; ++i)
        {
            new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
    }

    if (old_bytes_vec != new_bytes_vec)
    {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }
}

void RVSSProcessor::WriteMemoryDouble()
{
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

    if (control_unit_.GetMemRead())
    { // FLD
        memory_result_ = memory_controller_.readDoubleWord(execution_result_);
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    if (control_unit_.GetMemWrite())
    { // FSD
        addr = execution_result_;
        for (size_t i = 0; i < 8; ++i)
        {
            old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
        memory_controller_.writeDoubleWord(execution_result_, registers_.ReadFpr(rs2));
        for (size_t i = 0; i < 8; ++i)
        {
            new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
    }

    if (old_bytes_vec != new_bytes_vec)
    {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }
}

void RVSSProcessor::WriteBack()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;
    int32_t imm = ImmGenerator(current_instruction_);

    if (opcode == 0b1110011 && funct3 == 0b000)
    {
        return;
    }

    if (instruction_set::isFInstruction(current_instruction_))
    { // RV64 F
        WriteBackFloat();
        return;
    }
    else if (instruction_set::isDInstruction(current_instruction_))
    {
        WriteBackDouble();
        return;
    }
    else if (opcode == 0b1110011)
    { // CSR opcode
        WriteBackCsr();
        return;
    }
    else if (opcode == 0b1010111)
    { // RV64 V
        WriteBackVector();
        return;
    }

    uint64_t old_reg = registers_.ReadGpr(rd);
    unsigned int reg_index = rd;
    unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR

    if (control_unit_.GetRegWrite())
    {
        switch (opcode)
        {
        case 0b0110011: // R-Type
        case 0b0010011: // I-Type
        case 0b0010111: // AUIPC
        case 0b0111011: // R4-Type / OP-32 (ADDW, SUBW, MULW, DIVW, DIVUW, REMW, REMUW, SLLW, SRLW, SRAW)
        case 0b0011011: // I-type OP-32 (ADDIW, SLLIW, SRLIW, SRAIW)
        {
            registers_.WriteGpr(rd, execution_result_);
            break;
        }
        case 0b0000011:
        { // Load
            registers_.WriteGpr(rd, memory_result_);
            break;
        }
        case 0b1100111: // JALR
        case 0b1101111:
        { // JAL
            registers_.WriteGpr(rd, next_pc_);
            break;
        }
        case 0b0110111:
        { // LUI
            registers_.WriteGpr(rd, (imm << 12));
            break;
        }
        default:
            break;
        }
    }

    if (opcode == 0b1101111)
    { // JAL
      // Updated in Execute()
    }
    if (opcode == 0b1100111)
    { // JALR
      // registers_.WriteGpr(rd, return_address_); // Write back to rs1
      // Updated in Execute()
    }

    uint64_t new_reg = registers_.ReadGpr(rd);
    if (old_reg != new_reg)
    {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }
}

void RVSSProcessor::WriteBackFloat()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;

    uint64_t old_reg = 0;
    unsigned int reg_index = rd;
    unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t new_reg = 0;

    if (control_unit_.GetRegWrite())
    {
        // write to GPR
        if (funct7 == 0b1010000 || funct7 == 0b1100000 || funct7 == 0b1110000)
        { // f(eq|lt|le).s, fcvt.(w|wu|l|lu).s
            old_reg = registers_.ReadGpr(rd);
            registers_.WriteGpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 0; // GPR
        }
        // write to FPR
        else if (opcode == 0b0000111)
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, memory_result_);
            new_reg = memory_result_;
            reg_type = 2; // FPR
        }
        else
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 2; // FPR
        }
    }

    if (old_reg != new_reg)
    {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }
}

void RVSSProcessor::WriteBackDouble()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;

    uint64_t old_reg = 0;
    unsigned int reg_index = rd;
    unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t new_reg = 0;

    if (control_unit_.GetRegWrite())
    {
        // write to GPR
        if (funct7 == 0b1010001 || funct7 == 0b1100001 || funct7 == 0b1110001)
        { // f(eq|lt|le).d, fcvt.(w|wu|l|lu).d
            old_reg = registers_.ReadGpr(rd);
            registers_.WriteGpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 0; // GPR
        }
        // write to FPR
        else if (opcode == 0b0000111)
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, memory_result_);
            new_reg = memory_result_;
            reg_type = 2; // FPR
        }
        else
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 2; // FPR
        }
    }

    if (old_reg != new_reg)
    {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }

    return;
}

void RVSSProcessor::WriteBackCsr()
{
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    uint64_t old_gpr = registers_.ReadGpr(rd);
    uint64_t old_csr = registers_.ReadCsr(csr_target_address_);

    switch (funct3)
    {
    case 0b001:
    { // CSRRW
        registers_.WriteGpr(rd, csr_old_value_);
        registers_.WriteCsr(csr_target_address_, csr_write_val_);
        break;
    }
    case 0b010:
    { // CSRRS
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_write_val_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_write_val_);
        }
        break;
    }
    case 0b011:
    { // CSRRC
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_write_val_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_write_val_);
        }
        break;
    }
    case 0b101:
    { // CSRRWI
        registers_.WriteGpr(rd, csr_old_value_);
        registers_.WriteCsr(csr_target_address_, csr_uimm_);
        break;
    }
    case 0b110:
    { // CSRRSI
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_uimm_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_uimm_);
        }
        break;
    }
    case 0b111:
    { // CSRRCI
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_uimm_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_uimm_);
        }
        break;
    }
    }

    uint64_t new_gpr = registers_.ReadGpr(rd);
    uint64_t new_csr = registers_.ReadCsr(csr_target_address_);

    if (old_gpr != new_gpr)
    {
        current_delta_.register_changes.push_back({rd, 0, old_gpr, new_gpr});
    }
    if (old_csr != new_csr)
    {
        current_delta_.register_changes.push_back({csr_target_address_, 1, old_csr, new_csr});
    }
}

void RVSSProcessor::WriteBackVsetType()
{
    // vtype
    uint64_t old_vtype = registers_.ReadCsr(0xC21);
    registers_.WriteCsr(0xC21, vset_new_vtype_);
    if (old_vtype != vset_new_vtype_)
        current_delta_.register_changes.push_back({0xC21, 1, old_vtype, vset_new_vtype_});

    // vl
    uint64_t old_vl = registers_.ReadCsr(0xC20);
    registers_.WriteCsr(0xC20, vset_new_vl_);
    if (old_vl != vset_new_vl_)
        current_delta_.register_changes.push_back({0xC20, 1, old_vl, vset_new_vl_});

    // rd = new vl  (writes to x0 are ignored by WriteGpr, if it follows the usual rule)
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;
    uint64_t old_gpr = registers_.ReadGpr(rd);
    registers_.WriteGpr(rd, vset_new_vl_);
    uint64_t new_gpr = registers_.ReadGpr(rd);
    if (old_gpr != new_gpr)
        current_delta_.register_changes.push_back({rd, 0, old_gpr, new_gpr});
}

void RVSSProcessor::WriteBackVector() 
{

    uint8_t vd = (current_instruction_ >> 7) & 0b11111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    if(funct3 == 0b111){
        WriteBackVsetType();
        return;
    }

    // Write the whole destination group. For undo/redo, record each changed
    // 64-bit word as reg_type 3 with reg_index = vreg * sizeOfVector + word.
    for (unsigned r = 0; r < vector_result_regs_; ++r)
    {
        const size_t reg = vd + r;
        const VectorRegister old_value = registers_.ReadVreg(reg);
        registers_.WriteVreg(reg, vector_result_[r]);
        for (size_t w = 0; w < sizeOfVector; ++w)
        {
            if (old_value[w] != vector_result_[r][w])
            {
                current_delta_.register_changes.push_back(
                    {static_cast<unsigned int>(reg * sizeOfVector + w), 3, old_value[w],
                     vector_result_[r][w]});
            }
        }
    }
    vector_result_regs_ = 0;
}

void RVSSProcessor::DebugRun()
{
    ClearStop();
    while (!stop_requested_ && program_counter_ < program_size_)
    {
        current_delta_.old_pc = program_counter_;
        if (std::find(breakpoints_.begin(), breakpoints_.end(), program_counter_) ==
            breakpoints_.end())
        {
            Fetch();
            Decode();
            Execute();
            WriteMemory();
            WriteBack();
            instructions_retired_++;
            cycle_s_++;
            std::cout << "Program Counter: " << program_counter_ << std::endl;

            current_delta_.new_pc = program_counter_;
            // history_.push(current_delta_);
            undo_stack_.push(current_delta_);
            while (!redo_stack_.empty())
            {
                redo_stack_.pop();
            }
            current_delta_ = RVSingleStageStepDelta();
            if (program_counter_ < program_size_)
            {
                std::cout << "VM_STEP_COMPLETED" << std::endl;
                output_status_ = "VM_STEP_COMPLETED";
            }
            else if (program_counter_ >= program_size_)
            {
                std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
                output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
            }
            DumpRegisters(globals::registers_dump_file_path, registers_);
            DumpState(globals::vm_state_dump_file_path);
            // update circuit UI after the debug step
            SetActiveWireNames();
            emit updateCircuitStateSignal(active_wires_);

            //unsigned int delay_ms = vm_config::config.getRunStepDelay();
            // std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        else
        {
            std::cout << "VM_BREAKPOINT_HIT " << program_counter_ << std::endl;
            output_status_ = "VM_BREAKPOINT_HIT";
            break;
        }
    }
    if (program_counter_ >= program_size_)
    {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSProcessor::Step()
{
    current_delta_.old_pc = program_counter_;
    if (program_counter_ < program_size_)
    {
        Fetch();
        Decode();
        Execute();
        WriteMemory();
        WriteBack();
        instructions_retired_++;
        cycle_s_++;
        std::cout << "Program Counter: " << std::hex << program_counter_ << std::dec << std::endl;

        current_delta_.new_pc = program_counter_;

        // history_.push(current_delta_);

        undo_stack_.push(current_delta_);
        while (!redo_stack_.empty())
        {
            redo_stack_.pop();
        }

        current_delta_ = RVSingleStageStepDelta();

        if (program_counter_ < program_size_)
        {
            std::cout << "VM_STEP_COMPLETED" << std::endl;
            output_status_ = "VM_STEP_COMPLETED";
        }
        else if (program_counter_ >= program_size_)
        {
            std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
            output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
        }
    }
    else if (program_counter_ >= program_size_)
    {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);

}

bool RVSSProcessor::StepAndNotify()
{
    QMutexLocker locker(&step_mutex_);
    if (program_counter_ >= program_size_)
    {
        return false;
    }
    last_executed_pc_ = program_counter_;
    Step();
    SetActiveWireNames();
    setProcessorState();
    emit processorClockedSignal(processor_state_);
    emit updateCircuitStateSignal(active_wires_);
    active_wires_.erase(active_wires_.begin() + always_active_wires_count_,
                        active_wires_.end());
    // clear active wires after emitting signal except always active wires
    return program_counter_ < program_size_;
}

void RVSSProcessor::Undo()
{
    qInfo() << "Attempting to undo last step in rvss";
    if (undo_stack_.empty())
    {
        std::cout << "VM_NO_MORE_UNDO" << std::endl;
        output_status_ = "VM_NO_MORE_UNDO";
        return;
    }

    StepDelta last = undo_stack_.top();
    undo_stack_.pop();

    for (const auto &change : last.register_changes)
    {
        switch (change.reg_type)
        {
        case 0:
        { // GPR
            registers_.WriteGpr(change.reg_index, change.old_value);
            break;
        }
        case 1:
        { // CSR
            registers_.WriteCsr(change.reg_index, change.old_value);
            break;
        }
        case 2:
        { // FPR
            registers_.WriteFpr(change.reg_index, change.old_value);
            break;
        }
        case 3:
        { // Vector register word: reg_index = vreg * sizeOfVector + word
            VectorRegister v = registers_.ReadVreg(change.reg_index / sizeOfVector);
            v[change.reg_index % sizeOfVector] = change.old_value;
            registers_.WriteVreg(change.reg_index / sizeOfVector, v);
            break;
        }
        default:
            std::cerr << "Invalid register type: " << change.reg_type << std::endl;
            break;
        }
    }

    for (const auto &change : last.memory_changes)
    {
        for (size_t i = 0; i < change.old_bytes_vec.size(); ++i)
        {
            memory_controller_.writeByte(change.address + i, change.old_bytes_vec[i]);
        }
    }

    program_counter_ = last.old_pc;
    instructions_retired_--;
    cycle_s_--;
    std::cout << "Program Counter: " << program_counter_ << std::endl;

    redo_stack_.push(last);

    output_status_ = "VM_UNDO_COMPLETED";
    std::cout << "VM_UNDO_COMPLETED" << std::endl;

    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);

    SetActiveWireNames();
    setProcessorState();
    emit updateCircuitStateSignal(active_wires_);
    emit processorClockedSignal(processor_state_);
    qInfo() << "Undo completed in rvss";
}

void RVSSProcessor::Redo()
{
    qInfo() << "Attempting to redo last undone step in rvss";
    if (redo_stack_.empty())
    {
        std::cout << "VM_NO_MORE_REDO" << std::endl;
        return;
    }

    StepDelta next = redo_stack_.top();
    redo_stack_.pop();

    // if (!history_.can_redo()) {
    //       std::cout << "Nothing to redo.\n";
    //       return;
    //   }

    //   StepDelta next = history_.redo();

    for (const auto &change : next.register_changes)
    {
        switch (change.reg_type)
        {
        case 0:
        { // GPR
            registers_.WriteGpr(change.reg_index, change.new_value);
            break;
        }
        case 1:
        { // CSR
            registers_.WriteCsr(change.reg_index, change.new_value);
            break;
        }
        case 2:
        { // FPR
            registers_.WriteFpr(change.reg_index, change.new_value);
            break;
        }
        case 3:
        { // Vector register word: reg_index = vreg * sizeOfVector + word
            VectorRegister v = registers_.ReadVreg(change.reg_index / sizeOfVector);
            v[change.reg_index % sizeOfVector] = change.new_value;
            registers_.WriteVreg(change.reg_index / sizeOfVector, v);
            break;
        }
        default:
            std::cerr << "Invalid register type: " << change.reg_type << std::endl;
            break;
        }
    }

    for (const auto &change : next.memory_changes)
    {
        for (size_t i = 0; i < change.new_bytes_vec.size(); ++i)
        {
            memory_controller_.writeByte(change.address + i, change.new_bytes_vec[i]);
        }
    }

    program_counter_ = next.new_pc;
    instructions_retired_++;
    cycle_s_++;
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
    std::cout << "Program Counter: " << program_counter_ << std::endl;
    undo_stack_.push(next);

    SetActiveWireNames();
    setProcessorState();
    emit updateCircuitStateSignal(active_wires_);
    emit processorClockedSignal(processor_state_);
}

void RVSSProcessor::Reset()
{
    program_counter_ = 0;
    instructions_retired_ = 0;
    cycle_s_ = 0;
    last_breakpoint_pc_ = UINT64_MAX; // Clear breakpoint tracking on reset
    registers_.Reset();
    memory_controller_.reset();
    control_unit_.Reset();
    branch_flag_ = false;
    next_pc_ = 0;
    execution_result_ = 0;
    memory_result_ = 0;

    return_address_ = 0;
    csr_target_address_ = 0;
    csr_old_value_ = 0;
    csr_write_val_ = 0;
    csr_uimm_ = 0;
    current_delta_.register_changes.clear();
    current_delta_.memory_changes.clear();
    current_delta_.old_pc = 0;
    current_delta_.new_pc = 0;
    undo_stack_ = std::stack<StepDelta>();
    redo_stack_ = std::stack<StepDelta>();
}
}//namespaace Kites
