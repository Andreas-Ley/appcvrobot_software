#pragma once

#include <random>
#include <vector>

#include <Eigen/Dense>


class RansacE {
    public:
        RansacE &setMaxDistance(float maxD) { m_maxDistance = maxD; return *this; }
        RansacE &setNumIters(unsigned numIters) { m_numIters = numIters; return *this; }

        unsigned process(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> &pairs,
                     std::vector<float> &distances, Eigen::Matrix3f &E);
    protected:
        std::mt19937 m_rng;
        float m_maxDistance = 0.015f;
        unsigned m_numIters = 5000;
};