#pragma once

#include "FrameKPGrid.h"

class SlowBrief;
class Image;

struct InternalCalibration;

struct RawMatch {
    unsigned dstIdx;
    std::uint16_t bestDistance;
    std::uint16_t secondBestDistance;
};

class Frame {
    public:
        enum  {
            DESCRIPTOR_DIMENSION = 256,
            DESCRIPTOR_BYTES = DESCRIPTOR_DIMENSION/8
        };

        void extractKeypoints(const Image &img, const SlowBrief &brief);
        void buildKPGrid(const InternalCalibration &internalCalib);

        void matchWith(const Frame &other, std::vector<RawMatch> &dst) const;


        unsigned getWidth() const { return m_width; }
        unsigned getHeight() const { return m_height; }
        const std::vector<Keypoint> &getKeypoints() const { return m_keypoints; }
        const std::vector<std::uint8_t> &getDescriptors() const { return m_descriptors; }

        const FrameKPGrid &getKPGrid() const { return m_kpGrid; }
    protected:
        unsigned m_width = 0;
        unsigned m_height = 0;
        std::vector<Keypoint> m_keypoints;
        std::vector<std::uint8_t> m_descriptors;

        FrameKPGrid m_kpGrid;
};
