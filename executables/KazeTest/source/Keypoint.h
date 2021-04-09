#pragma once

#include <cstdint>

struct Keypoint {
    std::uint16_t x;
    std::uint16_t y;
    std::uint8_t strength;
};


