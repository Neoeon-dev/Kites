/**
 * @file vector_unit.h
 * @brief Processor-independent RVV execution logic, shared by every processor model.
 *
 * Everything here is a pure computation or a small commit helper:
 *  - ComputeVset / ExecuteArithmetic only *read* state and return a result.
 *  - CommitVset / CommitGroup write a result back and record undo entries.
 *
 * Each processor decides *when* these run (RVSS: Execute then WriteBack in one step;
 * RV5S: EX computes, the result rides EX/MEM and MEM/WB, WB commits).
 */
#pragma once

#include "processor/registers.h"

#include <cstdint>
#include <string>
#include <vector>

namespace Kites
{
struct RegisterChange; // processor_base.h (not included here to keep this header light)

namespace vector_unit
{
inline constexpr uint8_t kOpcodeOpV = 0b1010111;
inline constexpr size_t kVlCsr = 0xC20;
inline constexpr size_t kVtypeCsr = 0xC21;

/// RegisterChange::reg_type for one 64-bit word of a vector register.
/// reg_index = vreg * sizeOfVector + word.
inline constexpr unsigned kRegTypeVectorWord = 3;

/// True for any OP-V instruction (vector arithmetic and vset*).
bool IsOpV(uint32_t instruction);

/// True for vsetvli / vsetivli / vsetvl (OP-V with funct3 = 111).
bool IsVset(uint32_t instruction);

struct VsetResult
{
    uint64_t vtype = 0; ///< new vtype (vill set if the requested config is illegal)
    uint64_t vl = 0;    ///< new vl (also written to rd)
};

/**
 * @brief Computes the new vtype/vl of a vset* instruction.
 * @param rs1_value  GPR value of the rs1 field (AVL for vsetvli/vsetvl).
 * @param rs2_value  GPR value of the rs2 field (vtype for vsetvl, unused otherwise).
 * @param current_vl Current vl, used when rs1 = rd = x0 (keep vl).
 */
VsetResult ComputeVset(uint32_t instruction, uint64_t rs1_value, uint64_t rs2_value,
                       uint64_t current_vl);

struct ArithResult
{
    bool ok = false;
    std::string error;                 ///< why the instruction was rejected, when !ok
    uint8_t vd = 0;                    ///< first register of the destination group
    std::vector<VectorRegister> group; ///< new values of vd .. vd + group.size() - 1
};

/**
 * @brief Executes one vector arithmetic instruction (OPIVV/OPIVX/OPIVI/OPMVV/OPMVX).
 *
 * Reads vl, vtype and the source/destination vector registers from @p registers but
 * writes nothing. Masked-off and tail elements keep the old destination values.
 *
 * @param scalar GPR value of rs1, used by the .vx forms.
 */
ArithResult ExecuteArithmetic(uint32_t instruction, const RegisterFile &registers,
                              uint64_t scalar);

/// Writes vtype, vl and rd (= new vl), recording each change for undo.
void CommitVset(RegisterFile &registers, uint8_t rd, const VsetResult &result,
                std::vector<RegisterChange> &changes);

/// Writes a destination group, recording each changed 64-bit word for undo.
void CommitGroup(RegisterFile &registers, const ArithResult &result,
                 std::vector<RegisterChange> &changes);

/// Applies one recorded kRegTypeVectorWord change (used by Undo and Redo).
void ApplyVectorWord(RegisterFile &registers, unsigned reg_index, uint64_t value);

/**
 * @brief What a vector instruction carries down a pipeline until writeback.
 * Empty (kind == kNone) for every non-vector instruction, so it costs nothing there.
 */
struct PipelineData
{
    enum class Kind : uint8_t
    {
        kNone,  ///< not a vector instruction (or rejected): nothing to commit
        kVset,  ///< commit vset (rd, vl, vtype)
        kArith, ///< commit arith.group to arith.vd ..
    };

    Kind kind = Kind::kNone;
    VsetResult vset;
    ArithResult arith;

    void reset() { *this = PipelineData{}; }
};

} // namespace vector_unit
} // namespace Kites
