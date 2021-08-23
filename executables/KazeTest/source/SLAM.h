#pragma once

#include "Map.h"
#include "SlowBrief.h"

#include <boost/rational.hpp>

struct Settings {
    float ransacThresh = 0.0075f;
    unsigned minKeypointsFirstFrame = 1000;
    unsigned minEInliersSecondFrame = 500;
    float maxEHInliersSecondFrameFraction = 0.8f;
    unsigned minInitialTracks = 400;
    unsigned minPnPInliers = 100;

    unsigned matchMaxDistance = 80;
    boost::rational<unsigned> maxDistanceRatio = {2,3};

    unsigned numPastFramesMatch = 3;
};


class SLAM {
    public:
        SLAM(InternalCalibration internalCalib);

        bool tryIngestImage(Image image);

        CameraPose getCurrentCameraEstimate() { return {}; }

        Map &getMap() { return m_map; }
    protected:
        Settings m_settings;

        SlowBrief m_brief;

        Map m_map;

        InternalCalibration m_internalCalib;

        Frame processImage(Image image);

        void firstFrame(Frame frame);
        void secondFrame(Frame frame);
        void nextFrame(Frame frame);
};