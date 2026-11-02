#include <gtest/gtest.h>

#include "processor/registers.h"

using namespace Kites;

TEST(VectorRegisterFile, initializesRegistersToZero)
{
    RegisterFile registers;

    for (size_t reg = 0; reg < 32; ++reg)
    {
        const auto value = registers.ReadVreg(reg);

        for (uint8_t byte : value)
        {
            EXPECT_EQ(byte, 0);
        }
    }
}

TEST(VectorRegisterFile, readsAndWrites8BitElements)
{
    RegisterFile registers;

    registers.WriteVectorElement(0, 0, 8, 0xAB);
    registers.WriteVectorElement(0, 1, 8, 0xCD);
    registers.WriteVectorElement(0, 31, 8, 0xEF);
    registers.WriteVectorElement(1, 2, 8, 0x56);

    VectorRegister v1{}, v2{};
    v1[0] |= (uint64_t{0xAB} << 0);
    v1[0] |= (uint64_t{0xCD} << 8);
    v1[3] |= (uint64_t{0xEF} << 56);

    EXPECT_EQ(registers.ReadVectorElement(0, 0, 8), 0xAB);
    EXPECT_EQ(registers.ReadVectorElement(0, 1, 8), 0xCD);
    EXPECT_EQ(registers.ReadVectorElement(0, 31, 8), 0xEF);
    EXPECT_EQ(registers.ReadVreg(0), v1);
    
}

TEST(VectorRegisterFile, readsAndWrites32BitElements)
{
    RegisterFile registers;

    registers.WriteVectorElement(1, 0, 32, 0x12345678);
    registers.WriteVectorElement(1, 7, 32, 0xABCDEF12);

    EXPECT_EQ(
        registers.ReadVectorElement(1, 0, 32),
        0x12345678
    );

    EXPECT_EQ(
        registers.ReadVectorElement(1, 7, 32),
        0xABCDEF12
    );
}

TEST(VectorRegisterFile, supports64BitElements)
{
    RegisterFile registers;

    registers.WriteVectorElement(
        2, 3, 64, 0x123456789ABCDEF0ULL
    );

    EXPECT_EQ(
        registers.ReadVectorElement(2, 3, 64),
        0x123456789ABCDEF0ULL
    );
}

TEST(VectorRegisterFile, rejectsInvalidRegisterIndex)
{
    RegisterFile registers;

    EXPECT_THROW(
        registers.ReadVreg(32),
        std::out_of_range
    );
}

TEST(VectorRegisterFile, rejectsInvalidElementIndex)
{
    RegisterFile registers;

    // 256 / 32 = 8 elements.
    EXPECT_THROW(
        registers.ReadVectorElement(0, 8, 32),
        std::out_of_range
    );
}

TEST(VectorRegisterFile, rejectsUnsupportedSEW)
{
    RegisterFile registers;

    EXPECT_THROW(
        registers.ReadVectorElement(0, 0, 128),
        std::invalid_argument
    );
}

TEST(VectorRegisterFile, resetClearsVectorRegisters)
{
    RegisterFile registers;

    registers.WriteVectorElement(5, 0, 64, 12345);

    registers.Reset();

    EXPECT_EQ(registers.ReadVectorElement(5, 0, 64), 0);
}

TEST(VectorRegisterFile, ReadsMultipleRegisters)
{
    RegisterFile registers;

    registers.WriteVectorElement(0, 0, 8, 0xAB);
    registers.WriteVectorElement(0, 1, 8, 0xCD);
    registers.WriteVectorElement(0, 31, 8, 0xEF);
    registers.WriteVectorElement(1, 2, 8, 0x56);

    EXPECT_EQ(registers.ReadVectorElement(0, 0, 8), 0xAB);
    EXPECT_EQ(registers.ReadVectorElement(0, 1, 8), 0xCD);
    EXPECT_EQ(registers.ReadVectorElement(0, 31, 8), 0xEF);
}