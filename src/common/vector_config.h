#ifndef VECTOR_CONFIG_H
#define VECTOR_CONFIG_H

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace Kites
{
enum class SEW
{
    E8 = 0b000,
    E16 = 0b001,
    E32 = 0b010,
    E64 = 0b011,
};

enum class LMUL
{
    M1 = 0b000,
    M2 = 0b001,
    M4 = 0b010,
    M8 = 0b011,
    MF2 = 0b111,
    MF4 = 0b110,
    MF8 = 0b101,
};

enum class Tail
{
    TU = 0b0,   // tail undisturbed
    TA= 0b1,    // tail agnostic
};

enum class Mask
{
    MU = 0b0,   // mask undisturbed
    MA = 0b1,   // mask agnostic
};

inline const std::unordered_map<std::string, SEW> stringToSEW = {
    {"e8", SEW::E8}, {"e16", SEW::E16}, {"e32", SEW::E32}, {"e64", SEW::E64}};

inline const std::unordered_map<std::string, LMUL> stringToLMUL = {
    {"m1", LMUL::M1}, {"m2", LMUL::M2}, {"m4", LMUL::M4}, {"m8", LMUL::M8},
    {"mf8", LMUL::MF8}, {"mf4", LMUL::MF4}, {"mf2", LMUL::MF2}};

inline const std::unordered_map<std::string, Tail> stringToTail = {
    {"tu", Tail::TU}, {"ta", Tail::TA}};

inline const std::unordered_map<std::string, Mask> stringToMask = {
    {"mu", Mask::MU}, {"ma", Mask::MA}};


inline bool isValidSEW(const std::string &value){
    return stringToSEW.find(value) != stringToSEW.end();
}
inline bool isValidLMUL(const std::string &value){
    return stringToLMUL.find(value) != stringToLMUL.end();
}
inline bool isValidTail(const std::string &value){
    return stringToTail.find(value) != stringToTail.end();
}
inline bool isValidMask(const std::string &value){
    return stringToMask.find(value) != stringToMask.end();
}


inline uint16_t encodeVtypeImmediate(
    const std::string &sew,
    const std::string &lmul,
    const std::string &tail,
    const std::string &mask)
{
    if(!isValidSEW(sew) || !isValidLMUL(lmul) || !isValidTail(tail) || !isValidMask(mask))
    {
        throw std::invalid_argument("Invalid vector configuration parameters.");
    }

    uint16_t vsew = static_cast<uint8_t>(stringToSEW.at(sew));
    uint16_t vlmul = static_cast<uint8_t>(stringToLMUL.at(lmul));
    uint16_t vta = static_cast<uint8_t>(stringToTail.at(tail));
    uint16_t vma = static_cast<uint8_t>(stringToMask.at(mask));

    return vlmul | (vsew << 3) | (vta << 6) | (vma << 7);
}

}//namespace Kites
#endif // VECTOR_CONFIG_H