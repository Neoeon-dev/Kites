#include <gtest/gtest.h>

#include "assembler/lexer.h"
#include "assembler/parser.h"

#include <sstream>

using namespace Kites;

TEST(VectorRegisterParser, parsesVectorAdd)
{
    std::istringstream source(
        ".text\n"
        "vadd.vv v0, v1, v2\n"
    );

    Lexer lexer(source);
    const auto tokens = lexer.getTokenList();

    Parser parser("<vector-test>", tokens);
    parser.parse();

    EXPECT_EQ(parser.getErrorCount(), 0);

    const auto &code = parser.getIntermediateCode();

    ASSERT_EQ(code.size(), 1);

    EXPECT_EQ(code[0].first.getOpcode(), "vadd.vv");
    EXPECT_EQ(code[0].first.getRd(), "v0");
    EXPECT_EQ(code[0].first.getRs1(), "v1");
    EXPECT_EQ(code[0].first.getRs2(), "v2");
}


TEST(VectorRegisterParser, generatesVaddVvMachineCode)
{
    std::istringstream source(
        ".text\n"
        "vadd.vv v3, v5, v7\n"
    );

    Lexer lexer(source);
    const auto tokens = lexer.getTokenList();

    Parser parser("<vector-test>", tokens);
    parser.parse();

    ASSERT_EQ(parser.getErrorCount(), 0);

    const auto machineCode =
        generateMachineCode(parser.getIntermediateCode());

    ASSERT_EQ(machineCode.size(), 1);

    EXPECT_EQ(machineCode[0], 0x025381D7u);
}

