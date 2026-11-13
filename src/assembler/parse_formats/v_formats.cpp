/** File Name: v_formats.cpp
 * Author: Navaneeth.V
 * Github:  https://github.com/Neoeon-dev
 */

#include "assembler/parser.h"
#include "common/instructions.h"
#include "utils/utils.h"
#include "processor/registers.h"

#include <string>
namespace Kites
{
bool Parser::parse_O_GPR_C_GPR_C_VTYPE()
{
    if (peekToken(1).line_number == curentToken().line_number&&
        peekToken(1).type == TokenType::GP_REGISTER &&
        peekToken(2).line_number == currentToken().line_number &&
        peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == currentToken().line_number &&
        peekToken(3).type == TokenType::GP_REGISTER &&
        peekToken(4).line_number == currentToken().line_number &&
        peekToken(4).type == TokenType::COMMA &&
        peekToken(5).line_number == currentToken().line_number &&
        peekToken(5).type == TokenType::SEW &&
        peekToken(6).line_number == currentToken().line_number &&
        peekToken(6).type == TokenType::COMMA &&
        peekToken(7).line_number == currentToken().line_number &&
        peekToken(7).type == TokenType::LMUL &&
        peekToken(8).line_number == currentToken().line_number &&
        peekToken(8).type == TokenType::COMMA &&
        peekToken(9).line_number == currentToken().line_number &&
        peekToken(9).type == TokenType::TAIL &&
        peekToken(10).line_number == currentToken().line_number &&
        peekToken(10).type == TokenType::COMMA &&
        peekToken(11).line_number == currentToken().line_number &&
        peekToken(11).type == TokenType::MASK &&
        (peekToken(12).type == TokenType::EOF_ ||
         peekToken(12).line_number != currentToken().line_number)
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;
            reg = reg_alias_to_name.at(peekToken(1).value);
        }
)

    {
        ICUnit 
    }
}
}