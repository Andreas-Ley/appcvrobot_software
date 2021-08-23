#pragma once

#include <random>
#include <vector>

#include <Eigen/Dense>


class RansacPnP {
    public:
        RansacPnP &setMaxDistance(float maxD) { m_maxDistance = maxD; return *this; }
        RansacPnP &setNumIters(unsigned numIters) { m_numIters = numIters; return *this; }

        unsigned process(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector4f>> &pairs,
                     std::vector<float> &sqrDistances, Eigen::Matrix3f &R, Eigen::Vector3f &t);
    protected:
        std::mt19937 m_rng;
        float m_maxDistance = 0.015f;
        unsigned m_numIters = 5000;
};