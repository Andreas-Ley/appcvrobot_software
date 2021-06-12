#include "SlowBrief.h"

#include "Image.h"
#include "Keypoint.h"

#include <random>

#include <string.h>


SlowBrief::SlowBrief(unsigned dimension, unsigned patternExtend, unsigned seed):
    m_patternExtend(patternExtend){

    std::mt19937 mt(seed);
    std::normal_distribution<float> dist(0.0f, 8.0f);
    m_descriptorPattern.resize(dimension);

    for (unsigned i = 0; i < m_descriptorPattern.size(); i++) {
        int sx = std::min<int>(std::max<int>(dist(mt), -patternExtend), patternExtend);
        int sy = std::min<int>(std::max<int>(dist(mt), -patternExtend), patternExtend);
        int dx = std::min<int>(std::max<int>(dist(mt), -patternExtend), patternExtend);
        int dy = std::min<int>(std::max<int>(dist(mt), -patternExtend), patternExtend);
        m_descriptorPattern[i] = {sx, sy, dx, dy};
    }
}

void SlowBrief::operator()(const Image &img,
        const std::vector<Keypoint> &keypoints,
        std::vector<std::uint8_t> &descriptors,
        std::vector<bool> &valid) const {

    unsigned descriptorWords = m_descriptorPattern.size() / 8;
    descriptors.resize(keypoints.size() * descriptorWords);
    valid.resize(keypoints.size());
    memset(descriptors.data(), 0, descriptors.size());

    for (unsigned i = 0; i < keypoints.size(); i++) {
        unsigned descOffset = i * descriptorWords;

        auto cx = keypoints[i].x;
        auto cy = keypoints[i].y;
        //auto strength = keypoints[i].strength;
        if (cx < m_patternExtend || cx+m_patternExtend >= img.width() ||
            cy < m_patternExtend || cy+m_patternExtend >= img.height()) {
            valid[i] = false;
            continue;
        }
        valid[i] = true;

        for (unsigned j = 0; j < m_descriptorPattern.size(); j++) {
            auto [sx, sy, dx, dy] = m_descriptorPattern[j];
            bool bit = img(cx+sx, cy+sy) > img(cx+dx, cy+dy);
            if (bit)
                descriptors[descOffset+j/8] |= 1 << (j % 8);
        }
    }
        }