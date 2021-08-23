#pragma once

#include <cstdint>

struct Keypoint {
    std::uint16_t x;
    std::uint16_t y;
    std::int16_t x_undistorted_times4;
    std::int16_t y_undistorted_times4;
    std::uint8_t strength;
};


