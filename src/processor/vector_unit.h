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
struct MemoryChange;   // processor_base.h
class MemoryController;

namespace vector_unit
{
inline constexpr uint8_t kOpcodeOpV = 0b1010111;
inline constexpr uint8_t kOpcodeLoadFp = 0b0000111;  ///< vle* share LOAD-FP with flw/fld
inline constexpr uint8_t kOpcodeStoreFp = 0b0100111; ///< vse* share STORE-FP with fsw/fsd
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

// ---------------------------------------------------------------------------------------
// Vector loads / stores (unit-stride only: vle8/16/32/64.v, vse8/16/32/64.v)
// ---------------------------------------------------------------------------------------

/// True for any vector load/store: LOAD-FP/STORE-FP with a vector element width
/// (000, 101, 110, 111). flw/fld/fsw/fsd use 010/011, so the two never overlap.
bool IsVectorMemory(uint32_t instruction);
bool IsVectorLoad(uint32_t instruction);
bool IsVectorStore(uint32_t instruction);

/**
 * @brief Everything the MEM stage needs to perform a unit-stride vector load/store.
 *
 * Built in EX by PlanUnitStride: it checks the instruction is legal and records which
 * elements to move. Like the arithmetic path, vector registers are read in EX: the old
 * destination group of a load (so masked-off/tail elements stay undisturbed) and the
 * data of a store.
 */
struct MemoryPlan
{
    bool ok = false;
    std::string error;                 ///< why the instruction was rejected, when !ok
    bool is_load = false;
    unsigned eew = 8;                  ///< element width in bits, encoded in the instruction
    uint64_t base = 0;                 ///< address of element 0 (GPR rs1)
    std::vector<uint32_t> elements;    ///< indices of the active elements (< vl, mask set)
    std::vector<uint64_t> store_data;  ///< store: value of each active element, same order
    ArithResult load;                  ///< load: vd and its current group; MEM fills elements in
};

/**
 * @brief EX: validates a unit-stride vle/vse and gathers its operands.
 * @param base GPR value of rs1 (the base address), read in ID like any scalar source.
 */
MemoryPlan PlanUnitStride(uint32_t instruction, const RegisterFile &registers, uint64_t base);

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
        kLoad,  ///< MEM reads memory into mem.load, then it commits like kArith
        kStore, ///< MEM writes memory; nothing to commit in WB
    };

    Kind kind = Kind::kNone;
    VsetResult vset;
    ArithResult arith;
    MemoryPlan mem;

    void reset() { *this = PipelineData{}; }
};

// ---------------------------------------------------------------------------------------
// Hazard detection helpers (used by the RV5S Hazard Detection Unit)
//
// Pipeline timing of a vector instruction: GPR operands are read in ID, vector registers,
// vl and vtype are read in EX, and everything (vector group, vl/vtype, vset's rd) is
// written in WB. WB runs before EX within a cycle, so a vector consumer only collides
// with the producer directly ahead of it (1 stall); a GPR consumer of vset's rd behaves
// like any other GPR dependency.
// ---------------------------------------------------------------------------------------

/// True for any instruction executed by the vector unit (OP-V or vle/vse).
bool IsVectorInstruction(uint32_t instruction);

/// Whether the rs1 / rs2 fields of a vector instruction name a GPR it reads.
/// (For .vv/.vi forms the rs1 field is vs1/imm5, for vle/vse rs2 is lumop/sumop, ...)
bool ReadsGprRs1(uint32_t instruction);
bool ReadsGprRs2(uint32_t instruction);

/// True if the instruction reads vl/vtype in EX: every vector instruction except
/// vset*, which only reads vl in the "keep vl" form (rs1 = rd = x0).
bool ReadsVectorConfig(uint32_t instruction);

/**
 * @brief Bitmask (bit i = v<i>) of the vector registers an instruction reads in EX.
 * @param vtype The vtype the instruction will execute under (sizes the register groups).
 * Includes the old destination group (kept for masked-off/tail elements) and v0 if masked.
 */
uint32_t VregsRead(uint32_t instruction, uint64_t vtype);

/// Bitmask of the vector registers an in-flight result will write in WB.
uint32_t VregsWritten(const PipelineData &data);

/**
 * @brief MEM: performs the memory access of a kLoad/kStore (no-op for other kinds).
 *
 * A load reads its active elements into the destination group and turns @p data into
 * kArith, so WB commits it with CommitGroup. A store writes its elements and records
 * every changed byte range in @p memory_changes for undo.
 */
void AccessMemory(PipelineData &data, MemoryController &memory,
                  std::vector<MemoryChange> &memory_changes);

} // namespace vector_unit
} // namespace Kites
