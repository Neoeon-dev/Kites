#include "assembler/parser.h"
#include "common/instructions.h"
#include "utils/utils.h"
#include "processor/registers.h"
#include "common/vector_config.h"

#include <string>
namespace Kites
{
bool Parser::parse_O_GPR_C_GPR_C_SEW_C_LMUL_C_TAIL_C_MASK()
{
    if (peekToken(1).line_number == currentToken().line_number &&
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
         peekToken(12).line_number != currentToken().line_number))
    {
        ICUnit block;
        block.setOpcode(currentToken().value);
        block.setLineNumber(currentToken().line_number);
        block.setInstructionIndex(instruction_index_);

        std::string reg;
        reg = reg_alias_to_name.at(peekToken(1).value);
        block.setRd(reg);
        reg = reg_alias_to_name.at(peekToken(3).value);
        block.setRs1(reg);

        uint16_t vtypei = encodeVtypeImmediate(peekToken(5).value, peekToken(7).value, peekToken(9).value, peekToken(11).value);
        
        block.setImm(std::to_string(vtypei));

        skipCurrentLine();
        intermediate_code_.emplace_back(block, true);
        instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
        instruction_index_++;
        return true;
    }
    return false;
}

bool Parser::parse_O_GPR_C_UIMM_C_SEW_C_LMUL_C_TAIL_C_MASK()
{
    if (peekToken(1).line_number == currentToken().line_number &&
        peekToken(1).type == TokenType::GP_REGISTER &&
        peekToken(2).line_number == currentToken().line_number &&
        peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == currentToken().line_number &&
        peekToken(3).type == TokenType::NUM &&
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
         peekToken(12).line_number != currentToken().line_number))
    {
        ICUnit block;
        block.setOpcode(currentToken().value);
        block.setLineNumber(currentToken().line_number);
        block.setInstructionIndex(instruction_index_);

        std::string reg;
        reg = reg_alias_to_name.at(peekToken(1).value);
        block.setRd(reg);

        // AVL is a 5-bit unsigned immediate (it is encoded in the rs1 field)
        int64_t uimm = std::stoll(peekToken(3).value);
        if (0 <= uimm && uimm <= 31)
        {
            block.setRs1(std::to_string(uimm)); /// Rs1 holds the uimm (a number, not a register) in this format.
        }
        else
        {
            recordError(ParseError(peekToken(3).line_number, "Immediate value out of range"));
            errors_.all_errors.emplace_back(errors::ImmediateOutOfRangeError(
                "Immediate value out of range", "Expected: 0 <= uimm <= 31", filename_,
                peekToken(3).line_number, peekToken(3).column_number,
                GetLineFromFile(filename_, peekToken(3).line_number)));
            skipCurrentLine();
            return true;
        }

        uint16_t vtypei = encodeVtypeImmediate(peekToken(5).value, peekToken(7).value, peekToken(9).value, peekToken(11).value);
        block.setImm(std::to_string(vtypei));

        skipCurrentLine();
        intermediate_code_.emplace_back(block, true);
        instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
        instruction_index_++;
        return true;
    }
    return false;
}

bool Parser::parse_O_VREG_C_VREG_C_VREG()
{
    if (peekToken(1).line_number == currentToken().line_number &&
        peekToken(1).type == TokenType::VEC_REGISTER &&
        peekToken(2).line_number == currentToken().line_number &&
        peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == currentToken().line_number &&
        peekToken(3).type == TokenType::VEC_REGISTER &&
        peekToken(4).line_number == currentToken().line_number &&
        peekToken(4).type == TokenType::COMMA &&
        peekToken(5).line_number == currentToken().line_number &&
        peekToken(5).type == TokenType::VEC_REGISTER)
    {   
        if(peekToken(6).type == TokenType::EOF_ ||  
           peekToken(6).line_number != currentToken().line_number)
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;
            reg = reg_alias_to_name.at(peekToken(1).value);
            block.setVd(reg);
            reg = reg_alias_to_name.at(peekToken(3).value);
            block.setVs2(reg);
            reg = reg_alias_to_name.at(peekToken(5).value);
            block.setVs1(reg);

            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;

            return true;
        }
    else if(peekToken(6).line_number == currentToken().line_number &&
            peekToken(6).type == TokenType::COMMA &&
            peekToken(7).line_number == currentToken().line_number &&
            peekToken(7).type == TokenType:: VM &&
            (peekToken(8).type == TokenType::EOF_ ||  
             peekToken(8).line_number != currentToken().line_number))
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;
            reg = reg_alias_to_name.at(peekToken(1).value);
            block.setVd(reg);
            reg = reg_alias_to_name.at(peekToken(3).value);
            block.setVs2(reg);
            reg = reg_alias_to_name.at(peekToken(5).value);
            block.setVs1(reg);
            block.setVm(0); // Assuming the 4th vector register is used for masking, set vm to 0

            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;
            
            return true;
        }
    }
    return false;
}

bool Parser::parse_O_VREG_C_VREG_C_GPR()
{
    if (peekToken(1).line_number == currentToken().line_number &&
        peekToken(1).type == TokenType::VEC_REGISTER &&
        peekToken(2).line_number == currentToken().line_number &&
        peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == currentToken().line_number &&
        peekToken(3).type == TokenType::VEC_REGISTER &&
        peekToken(4).line_number == currentToken().line_number &&
        peekToken(4).type == TokenType::COMMA &&
        peekToken(5).line_number == currentToken().line_number &&
        peekToken(5).type == TokenType::GP_REGISTER)
    {   
        if((peekToken(6).type == TokenType::EOF_ ||  
         peekToken(6).line_number != currentToken().line_number))
        {
        ICUnit block;
        block.setOpcode(currentToken().value);
        block.setLineNumber(currentToken().line_number);
        block.setInstructionIndex(instruction_index_);

        std::string reg;
        reg = reg_alias_to_name.at(peekToken(1).value);
        block.setVd(reg);
        reg = reg_alias_to_name.at(peekToken(3).value);
        block.setVs2(reg);
        reg = reg_alias_to_name.at(peekToken(5).value);
        block.setRs1(reg);

        skipCurrentLine();
        intermediate_code_.emplace_back(block, true);
        instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
        instruction_index_++;
        return true;
        }

        else if(peekToken(6).line_number == currentToken().line_number &&
                peekToken(6).type == TokenType::COMMA &&
                peekToken(7).line_number == currentToken().line_number &&
                peekToken(7).type == TokenType:: VM &&
                (peekToken(8).type == TokenType::EOF_ ||  
                 peekToken(8).line_number != currentToken().line_number))
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;
            reg = reg_alias_to_name.at(peekToken(1).value);
            block.setVd(reg);
            reg = reg_alias_to_name.at(peekToken(3).value);
            block.setVs2(reg);
            reg = reg_alias_to_name.at(peekToken(5).value);
            block.setRs1(reg);
            block.setVm(0); // Assuming the 4th vector register is used for masking, set vm to 0

            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;
            
            return true;
        }
    }
    return false;
}

bool Parser::parse_O_VREG_C_VREG_C_IMM()
{
    if (peekToken(1).line_number == currentToken().line_number &&
        peekToken(1).type == TokenType::VEC_REGISTER &&
        peekToken(2).line_number == currentToken().line_number &&
        peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == currentToken().line_number &&
        peekToken(3).type == TokenType::VEC_REGISTER &&
        peekToken(4).line_number == currentToken().line_number &&
        peekToken(4).type == TokenType::COMMA &&
        peekToken(5).line_number == currentToken().line_number &&
        peekToken(5).type == TokenType::NUM)
    {   
        if((peekToken(6).type == TokenType::EOF_ ||  
         peekToken(6).line_number != currentToken().line_number))
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;
            reg = reg_alias_to_name.at(peekToken(1).value);
            block.setVd(reg);
            reg = reg_alias_to_name.at(peekToken(3).value);
            block.setVs2(reg);
            
            int64_t imm = std::stoll(peekToken(5).value);
            if (-16 <= imm && imm <= 15)
            {
                block.setImm(std::to_string(imm));
            }
            else
            {
                recordError(ParseError(peekToken(5).line_number, "Immediate value out of range"));
                errors_.all_errors.emplace_back(errors::ImmediateOutOfRangeError(
                    "Immediate value out of range", "Expected: -16 <= imm <= 15", filename_,
                    peekToken(5).line_number, peekToken(5).column_number,
                    GetLineFromFile(filename_, peekToken(5).line_number)));
                skipCurrentLine();
                return true;
            }
            
            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;
            return true;
        }

        else if(peekToken(6).line_number == currentToken().line_number &&
                peekToken(6).type == TokenType::COMMA &&
                peekToken(7).line_number == currentToken().line_number &&
                peekToken(7).type == TokenType:: VM &&
                (peekToken(8).type == TokenType::EOF_ ||  
                 peekToken(8).line_number != currentToken().line_number))
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;
            reg = reg_alias_to_name.at(peekToken(1).value);
            block.setVd(reg);
            reg = reg_alias_to_name.at(peekToken(3).value);
            block.setVs2(reg);

            int64_t imm = std::stoll(peekToken(5).value);
            if (-16 <= imm && imm <= 15)
            {
                block.setImm(std::to_string(imm));
            }
            else
            {
                recordError(ParseError(peekToken(5).line_number, "Immediate value out of range"));
                errors_.all_errors.emplace_back(errors::ImmediateOutOfRangeError(
                    "Immediate value out of range", "Expected: -16 <= imm <= 15", filename_,
                    peekToken(5).line_number, peekToken(5).column_number,
                    GetLineFromFile(filename_, peekToken(5).line_number)));
                skipCurrentLine();
                return true;
            }

            block.setVm(0); // Assuming the 4th vector register is used for masking, set vm to 0

            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;
            
            return true;
        }
    }
    return false;
} 

bool Parser::parse_O_VREG_C_LP_GPR_RP()
{
    if (peekToken(1).line_number == currentToken().line_number &&
        peekToken(1).type == TokenType::VEC_REGISTER &&
        peekToken(2).line_number == currentToken().line_number &&
        peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == currentToken().line_number &&
        peekToken(3).type == TokenType::LPAREN &&
        peekToken(4).line_number == currentToken().line_number &&
        peekToken(4).type == TokenType::GP_REGISTER &&
        peekToken(5).line_number == currentToken().line_number &&
        peekToken(5).type == TokenType::RPAREN)
    {
        if((peekToken(6).type == TokenType::EOF_ ||  
         peekToken(6).line_number != currentToken().line_number))
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;

            if(instruction_set::isValidVLTypeInstruction(block.getOpcode()))
            {
                reg = reg_alias_to_name.at(peekToken(1).value);
                block.setVd(reg);
                reg = reg_alias_to_name.at(peekToken(4).value);
                block.setRs1(reg);
            }
            else
            {
                reg = reg_alias_to_name.at(peekToken(1).value);
                block.setVs3(reg);
                reg = reg_alias_to_name.at(peekToken(4).value);
                block.setRs1(reg);
            }

            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;
            
            return true;
        }

        else if(peekToken(6).line_number == currentToken().line_number &&
                peekToken(6).type == TokenType::COMMA &&
                peekToken(7).line_number == currentToken().line_number &&
                peekToken(7).type == TokenType:: VM &&
                (peekToken(8).type == TokenType::EOF_ ||  
                 peekToken(8).line_number != currentToken().line_number))
        {
            ICUnit block;
            block.setOpcode(currentToken().value);
            block.setLineNumber(currentToken().line_number);
            block.setInstructionIndex(instruction_index_);

            std::string reg;

            if(instruction_set::isValidVLTypeInstruction(block.getOpcode()))
            {
                reg = reg_alias_to_name.at(peekToken(1).value);

                // A masked load may not overwrite the mask register v0
                if (reg == "v0")
                {
                    recordError(ParseError(peekToken(1).line_number, "Masked vector load cannot write v0"));
                    errors_.all_errors.emplace_back(errors::InvalidRegisterError(
                        "Invalid destination register", "vd cannot be v0 when the load is masked with v0.t", filename_,
                        peekToken(1).line_number, peekToken(1).column_number,
                        GetLineFromFile(filename_, peekToken(1).line_number)));
                    skipCurrentLine();
                    return true;
                }

                block.setVd(reg);
                reg = reg_alias_to_name.at(peekToken(4).value);
                block.setRs1(reg);
            }
            else
            {
                reg = reg_alias_to_name.at(peekToken(1).value);
                block.setVs3(reg);
                reg = reg_alias_to_name.at(peekToken(4).value);
                block.setRs1(reg);
            }

            block.setVm(0); // v0.t is present, so the instruction is masked: vm = 0

            skipCurrentLine();
            intermediate_code_.emplace_back(block, true);
            instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
            instruction_index_++;
            
            return true;
        }
    }
    return false;
}

}//namespace Kites