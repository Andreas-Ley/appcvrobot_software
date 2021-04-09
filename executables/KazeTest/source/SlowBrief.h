#pragma once

#include <cstdint>
#include <vector>
#include <tuple>

class Image;
class Keypoint;

class SlowBrief {
    public:
        SlowBrief(unsigned dimension = 256, unsigned patternExtend = 24);

        void operator()(const Image &img,
                const std::vector<Keypoint> &keypoints,
                std::vector<std::uint8_t> &descriptors,
                std::vector<bool> &valid) const;

        unsigned getDimension() const { return m_descriptorPattern.size(); }
        unsigned getPatternExtend() const { return m_patternExtend; }
    protected:
        unsigned m_patternExtend;
        std::vector<std::tuple<int, int, int, int>> m_descriptorPattern;
};
