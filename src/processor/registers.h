/**
 * @file registers.h
 * @brief Contains the definition of the RegisterFile class for managing registers in the simulator.
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#ifndef REGISTERS_H
#define REGISTERS_H

#include <QObject>
#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>
#include "processor/processor_types.h"

namespace Kites
{

  using VectorElementSize = uint64_t;

  const size_t sizeOfVector = architecture::VLEN_BYTES/sizeof(VectorElementSize);

  using VectorRegister = std::array<VectorElementSize, sizeOfVector>; ///< Type for vector register size, using 64-bit unsigned integer.
/**
 * @brief Represents a register file containing integer, floating-point, and vector registers.
 */
class RegisterFile : public QObject
{
    Q_OBJECT
  private:
    static constexpr size_t NUM_GPR = 32; ///< Number of General-Purpose Registers (GPR).
    static constexpr size_t NUM_FPR = 32; ///< Number of Floating-Point Registers (FPR).
    static constexpr size_t NUM_VREG = 32; ///< Number of Vector Registers.

    std::array<uint64_t, NUM_GPR> gpr_ = {}; ///< Array for storing GPR values.
    std::array<uint64_t, NUM_FPR> fpr_ = {}; ///< Array for storing FPR values.
    std::array<VectorRegister, NUM_VREG> vreg_ = {}; ///< Array for storing vector register values.

    static constexpr size_t NUM_CSR = 4096; ///< Number of Control and Status Registers (CSR).

    std::array<uint64_t, NUM_CSR> csr_ = {}; ///< Array for storing CSR values.

  public:
    /**
     * @brief Enum representing the type of a register.
     */
    enum class RegisterType
    {
        INVALID,        ///< Invalid register type.
        INTEGER,        ///< General-purpose integer register.
        FLOATING_POINT, ///< Floating-point register.
        VECTOR,         ///< Vector register.
        CSR             ///< Control and Status Register (CSR).
    };

    explicit RegisterFile(QObject *parent = nullptr);
    virtual ~RegisterFile() = default;

    void Reset();

    /**
     * @brief Reads the value of a General-Purpose Register (GPR).
     * @param reg The index of the GPR to read.
     * @return The value of the GPR at the specified index.
     */
    [[nodiscard]] uint64_t ReadGpr(size_t reg) const;

    /**
     * @brief Writes a value to a General-Purpose Register (GPR).
     * @param reg The index of the GPR to write.
     * @param value The value to write.
     */
    void WriteGpr(size_t reg, uint64_t value);

    /**
     * @brief Reads the value of a Floating-Point Register (FPR).
     * @param reg The index of the FPR to read.
     * @return The value of the FPR at the specified index.
     */
    [[nodiscard]] uint64_t ReadFpr(size_t reg) const;

    /**
     * @brief Writes a value to a Floating-Point Register (FPR).
     * @param reg The index of the FPR to write.
     * @param value The value to write.
     */
    void WriteFpr(size_t reg, uint64_t value);

    [[nodiscard]] uint64_t ReadCsr(size_t reg) const;

    void WriteCsr(size_t reg, uint64_t value);

    
    /**
     * @brief Reads the value of a Vector register (Vreg).
     * @param reg The index of the FPR to read.
     * @return The value of the FPR at the specified index.
     */

    [[nodiscard]] VectorRegister ReadVreg(size_t reg) const;

    /**
     * @brief Writes a value to a Vector Register (Vreg)
     * @param reg The index of the Vreg to write
     * @param value The value to write
     */

    void WriteVreg(size_t reg, VectorRegister value);

    [[nodiscard]] uint64_t ReadVectorElement(
      size_t reg,
      size_t elementIndex,
      size_t sewBits
    ) const;

    void WriteVectorElement(
      size_t reg,
      size_t elementIndex,
      size_t sewBits,
      uint64_t value
    );

    [[nodiscard]] std::vector<VectorRegister> GetVectorRegisterValue() const;

    /**
     * @brief Retrieves the values of all General-Purpose Registers (GPR).
     * @return A vector containing the values of all GPRs.
     */
    [[nodiscard]] std::vector<uint64_t> GetGprValues() const;

    /**
     * @brief Retrieves the values of all Floating-Point Registers (FPR).
     * @return A vector containing the values of all FPRs.
     */
    [[nodiscard]] std::vector<uint64_t> GetFprValues() const;

    void ModifyRegister(const std::string &reg_name, uint64_t value);

  signals:
    void updateRegister(size_t regIndex, uint64_t value);
    void updateFRegister(size_t regIndex, uint64_t value);
    void registerResetSignal();
};

extern const std::unordered_set<std::string> valid_general_purpose_registers;

extern const std::unordered_set<std::string> valid_floating_point_registers;

extern const std::unordered_set<std::string> valid_csr_registers;

extern const std::unordered_map<std::string, int> csr_to_address;

/**
 * @brief Map of register aliases to their actual names.
 */
extern const std::unordered_map<std::string, std::string> reg_alias_to_name;

bool IsValidGeneralPurposeRegister(const std::string &reg);

bool IsValidFloatingPointRegister(const std::string &reg);

bool IsValidCsr(const std::string &reg);

bool IsValidVectorRegister(const std::string &reg);

}//namespace Kites
#endif // REGISTERS_H