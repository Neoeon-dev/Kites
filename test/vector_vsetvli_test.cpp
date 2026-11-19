#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "assembler/assembler.h"
#include "common/instructions.h"

namespace
{

Kites::AssembledProgram assembleVsetvli()
{
    std::istringstream source(
        ".text\n"
        "vsetvli t0, a0, e32, m1, ta, ma\n"
    );

    return Kites::assemble(source);
}

} // namespace


// Test 1: Verify generated machine code.
TEST(VsetvliAssembler, GeneratesExpectedMachineCode)
{
    auto program = assembleVsetvli();

    ASSERT_EQ(program.text_buffer.size(), 1u);

    EXPECT_EQ(program.text_buffer[0], 0x0D0572D7u);
}


// Test 2: Verify the intermediate representation.
TEST(VsetvliAssembler, CreatesCorrectICUnit)
{
    auto program = assembleVsetvli();

    ASSERT_EQ(program.intermediate_code.size(), 1u);

    const auto &ic = program.intermediate_code[0].first;

    EXPECT_EQ(ic.getOpcode(), "vsetvli");
    EXPECT_EQ(ic.getRd(), "x5");
    EXPECT_EQ(ic.getRs1(), "x10");
    EXPECT_EQ(ic.getImm(), "208");
}


// Test 3: Verify disassembly.
TEST(VsetvliAssembler, DisassemblesCorrectly)
{
    auto program = assembleVsetvli();

    ASSERT_EQ(program.text_buffer.size(), 1u);

    const std::string decoded =
        Kites::instruction_set::disassemble(program.text_buffer[0]);

    EXPECT_EQ(
        decoded,
        "vsetvli x5, x10, e32, m1, ta, ma"
    );
}

// TEST(VsetvliAssembler, HandlesDifferentVectorConfiguration)
// {
//     std::istringstream source(
//         ".text\n"
//         "vsetvli t1, s0, e16, m2, tu, mu\n"
//     );

//     auto program = Kites::assemble(source);

//     ASSERT_EQ(program.text_buffer.size(), 1u);

//     // Verify machine code
//     EXPECT_EQ(program.text_buffer[0], 0x00947357u);

//     // Verify ICUnit
//     const auto &ic = program.intermediate_code[0].first;

//     EXPECT_EQ(ic.getOpcode(), "vsetvli");
//     EXPECT_EQ(ic.getRd(), "x6");
//     EXPECT_EQ(ic.getRs1(), "x8");
//     EXPECT_EQ(ic.getImm(), "9");

//     // Verify disassembly
//     EXPECT_EQ(
//         Kites::instruction_set::disassemble(program.text_buffer[0]),
//         "vsetvli x6, x8, e16, m2, tu, mu"
//     );
// }