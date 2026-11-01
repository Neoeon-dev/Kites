#pragma once

#include <cstddef>
#include <cstdint>

namespace Kites
{

namespace architecture
{
    inline constexpr std::size_t XLEN_BITS = 64;
    inline constexpr std::size_t VLEN_BITS = 256;
    inline constexpr std::size_t ELEN_BITS = 64;

    inline constexpr std::size_t VECTOR_REGISTER_COUNT = 32;
    inline constexpr std::size_t VLEN_BYTES = VLEN_BITS / 8;

}

enum class ProcessorType
{
    RVSS,
    RV5Stage_NH_NF,
    RV5Stage_H_NF,
    RV5Stage_NH_F,
    RV5Stage_H_F,

    ProcessorTypeCount
};

}  // namespace Kites