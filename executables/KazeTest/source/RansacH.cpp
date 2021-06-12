#include "RansacH.h"

unsigned RansacH::process(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> &pairs,
    Eigen::Matrix3f &bestH)
{
    std::uniform_int_distribution<unsigned> randomPair(0, pairs.size()-1);

    Eigen::Matrix<float, 8, 9> D;
    D.setZero();

    unsigned bestScore = 0;

    for (unsigned iter = 0; iter < m_numIters; iter++) {
        for (unsigned i = 0; i < 4; i++) {
            unsigned idx = randomPair(m_rng);

            D(i*2+0, 0) = pairs[idx].first[0];
            D(i*2+0, 1) = pairs[idx].first[1];
            D(i*2+0, 2) = 1.0f;

            D(i*2+0, 6) = -pairs[idx].first[0] * pairs[idx].second[0];
            D(i*2+0, 7) = -pairs[idx].first[1] * pairs[idx].second[0];
            D(i*2+0, 8) = -1.0f                * pairs[idx].second[0];

            D(i*2+1, 3) = pairs[idx].first[0];
            D(i*2+1, 4) = pairs[idx].first[1];
            D(i*2+1, 5) = 1.0f;

            D(i*2+1, 6) = -pairs[idx].first[0] * pairs[idx].second[1];
            D(i*2+1, 7) = -pairs[idx].first[1] * pairs[idx].second[1];
            D(i*2+1, 8) = -1.0f                * pairs[idx].second[1];
        }

        Eigen::JacobiSVD<Eigen::Matrix<float, 8, 9>> svd(D, Eigen::ComputeFullV);

        auto svdV = svd.matrixV();

        Eigen::Matrix3f H;
        H <<
                svdV(0, 8),
                svdV(1, 8),
                svdV(2, 8),
                svdV(3, 8),
                svdV(4, 8),
                svdV(5, 8),
                svdV(6, 8),
                svdV(7, 8),
                svdV(8, 8);

        unsigned score = 0;
        for (const auto &p : pairs) {
            Eigen::Vector3f proj = H.block<3,2>(0,0) * p.first + H.block<3,1>(0,2);
            Eigen::Vector2f scaled = p.second * proj[2];

            float sqrDistance = (proj.head<2>() - scaled).squaredNorm();
            float thresh = m_maxDistance*m_maxDistance * (proj[2]*proj[2]);

            if (sqrDistance < thresh)
                score++;
        }

        if (score > bestScore) {
            bestScore = score;
            bestH = H;
        }
    }

    return bestScore;
}
