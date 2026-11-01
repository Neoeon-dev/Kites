#include <gtest/gtest.h>

#include "assembler/lexer.h"

#include <sstream>

#include <QApplication>

#include "processor/registers.h"

#include <memory>

namespace
{

class QtTestEnvironment : public testing::Environment
{
public:
    void SetUp() override
    {
        qputenv("QT_QPA_PLATFORM", "offscreen");

        m_application = std::make_unique<QApplication>(
            m_argc, m_argv
        );
    }

    void TearDown() override
    {
        m_application.reset();
    }

private:
    int m_argc = 1;
    char m_arg0[16] = "kites_test";
    char *m_argv[2] = {m_arg0, nullptr};

    std::unique_ptr<QApplication> m_application;
};

[[maybe_unused]] const auto *qtTestEnvironment =
    testing::AddGlobalTestEnvironment(new QtTestEnvironment);

}

using namespace Kites;

TEST(VectorRegisterLexer, recognizesValidVectorRegisters)
{
    std::istringstream source("v0, v17, v31\n");
    Lexer lexer(source);

    const auto tokens = lexer.getTokenList();

    ASSERT_EQ(tokens.size(), 6);

    EXPECT_EQ(tokens[0].type, TokenType::VEC_REGISTER);
    EXPECT_EQ(tokens[0].value, "v0");

    EXPECT_EQ(tokens[2].type, TokenType::VEC_REGISTER);
    EXPECT_EQ(tokens[2].value, "v17");

    EXPECT_EQ(tokens[4].type, TokenType::VEC_REGISTER);
    EXPECT_EQ(tokens[4].value, "v31");
}

TEST(VectorRegisterLexer, rejectsOutOfRangeRegister)
{
    std::istringstream source("v32\n");
    Lexer lexer(source);

    const auto tokens = lexer.getTokenList();

    ASSERT_GE(tokens.size(), 1);
    EXPECT_EQ(tokens[0].type, TokenType::INVALID);
}