/**
 * @file rv5s_hdu.h
 * @brief Header for the universal Hazard Detection Unit (HDU) logic.
 * * This unit checks for data hazards in the pipeline, supporting both
 * No-Forwarding (H_NF) and With-Forwarding (H_F) modes for GPR/FPR data.
 */
#pragma once

#include <cstdint>
// Assuming the path to pipeline_registers.h is correct relative to the HDU file
#include "processor/pipeline_registers.h"

namespace Kites
{
// Define stall count constants (useful for client code)
constexpr int STALL_NONE = 0;
constexpr int STALL_ONE_CYCLE = 1;
constexpr int STALL_TWO_CYCLES = 2;

/**
 * @brief Checks for data hazards between the instruction in ID/EX (source) and
 * the instruction in IF/ID (dependent).
 * * This function handles all GPR and FPR dependencies.
 * * @param if_id_reg The IF/ID register (holds the dependent instruction's data).
 * @param id_ex_reg The ID/EX register (holds the hazard-producing instruction's data).
 * @param is_forwarding_enabled Flag to determine the stall logic:
 * - true: Only stall for the Load-Use hazard (STALL_ONE_CYCLE).
 * - false: Stall for ALL dependencies (STALL_TWO_CYCLES).
 * @return The number of cycles the pipeline must stall (0, 1, or 2).
 */
int check_data_hazard(const IF_ID_Register &if_id_reg, const ID_EX_Register &id_ex_reg,
                      const EX_MEM_Register &ex_mem_reg, bool is_forwarding_enabled);

/**
 * @brief Checks for vector (RVV) data hazards on the instruction in IF/ID.
 *
 * Vector registers, vl and vtype are read in EX and written in WB, and there is no
 * forwarding for them, so a vector instruction stalls one cycle when the instruction
 * directly ahead of it (now in EX/MEM):
 *  - is a vset* and the consumer reads vl/vtype (RAW on vl/vtype), or
 *  - writes a vector register group overlapping a group the consumer reads (RAW on vregs).
 *
 * GPR dependencies (vset* writing rd, .vx / vle / vset* reading rs1) are handled by
 * check_data_hazard like any scalar dependency.
 *
 * @param consumer_vtype vtype the IF/ID instruction will execute under; sizes its
 *        register groups. (If a vset* is directly ahead, the consumer stalls anyway.)
 * @return STALL_NONE or STALL_ONE_CYCLE.
 */
int check_vector_hazard(const IF_ID_Register &if_id_reg, const EX_MEM_Register &ex_mem_reg,
                        uint64_t consumer_vtype);
}//namespace Kites
