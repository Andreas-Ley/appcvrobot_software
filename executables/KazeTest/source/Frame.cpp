#include "Frame.h"

#include "Image.h"
#include "SlowBrief.h"
#include "Fast.h"

#include "Map.h"

void Frame::extractKeypoints(const Image &img, const SlowBrief &brief)
{
    m_width = img.width();
    m_height = img.height();

    m_keypoints.reserve(2000);

    Image fastDetections;
    fast(img, fastDetections);
    nonMaxSuppress<false>(fastDetections, brief.getPatternExtend(), m_keypoints);

    std::vector<bool> valid;
    brief(img, m_keypoints, m_descriptors, valid);
}

void Frame::buildKPGrid(const InternalCalibration &internalCalib)
{
    internalCalib.undistortKeypoints(m_keypoints.data(), m_keypoints.size());

    m_kpGrid.resize(std::max(1u, m_height / 64), std::max(1u, m_width / 64));
    //m_kpGrid.resize(3, 3);
    m_kpGrid.build(
        -(int)m_width/2,
        -(int)m_height/2,
        m_width+(int)m_width/2,
        m_height+(int)m_height/2,
        m_keypoints);
}

void Frame::matchWith(const Frame &other, std::vector<RawMatch> &dst) const
{
    const auto &otherKP = other.getKPGrid();

    dst.resize(m_keypoints.size());
    for (unsigned r = 0; r < m_kpGrid.getRows(); r++)
        for (unsigned c = 0; c < m_kpGrid.getCols(); c++) {
            auto cell = m_kpGrid(r, c);

            for (auto srcIdx : cell) {
                const auto srcKp = m_keypoints[srcIdx];
                /*
                unsigned dstColx2 = srcKp.x * other.getKPGrid().getCols()*2 / other.getWidth();
                unsigned dstRowx2 = srcKp.y * other.getKPGrid().getRows()*2 / other.getHeight();
                */


                int width_times2 = (otherKP.getEndX() - otherKP.getStartX()) * 2;
                int height_times2 = (otherKP.getEndY() - otherKP.getStartY()) * 2;

                int dstRowx2, dstColx2;
                dstRowx2 = (srcKp.y_undistorted_times4 - otherKP.getStartX()*4) * otherKP.getRows() / height_times2;
                dstColx2 = (srcKp.x_undistorted_times4 - otherKP.getStartY()*4) * otherKP.getCols() / width_times2;

                int dstCol = dstColx2 / 2;
                int dstRow = dstRowx2 / 2;
                bool rightQuadrant = dstColx2 % 2;
                bool bottomQuadrant = dstRowx2 % 2;

                int startCol;
                int endCol;
                if (rightQuadrant) {
                    startCol = (int)dstCol;
                    endCol = (int)dstCol+1;
                } else {
                    startCol = (int)dstCol-1;
                    endCol = (int)dstCol;
                }

                startCol = std::max(startCol, 0);
                endCol = std::min(endCol, (int)other.getKPGrid().getCols()-1);

                int startRow;
                int endRow;
                if (bottomQuadrant) {
                    startRow = (int)dstRow;
                    endRow = (int)dstRow+1;
                } else {
                    startRow = (int)dstRow-1;
                    endRow = (int)dstRow;
                }

                startRow = std::max(startRow, 0);
                endRow = std::min(endRow, (int) other.getKPGrid().getRows()-1);


                const std::uint32_t * __restrict__ srcPtr = (const std::uint32_t *) &m_descriptors[srcIdx * DESCRIPTOR_BYTES];

                unsigned closestMatch = 0;
                unsigned closestDistance = 10000;
                unsigned secondClosestDistance = 10000;

                const unsigned descWords = DESCRIPTOR_BYTES / 4;

                for (unsigned r2 = startRow; r2 <= endRow; r2++)
                    for (unsigned c2 = startCol; c2 <= endCol; c2++) {
                        auto dstCell = other.getKPGrid()(r2, c2);

                        for (auto dstIdx : dstCell) {
                            const std::uint32_t * __restrict__ dstPtr = (const std::uint32_t *) &other.m_descriptors[dstIdx * DESCRIPTOR_BYTES];

                            unsigned hammingDistance = 0;
                            for (unsigned k = 0; k < descWords; k++)
                                hammingDistance += __builtin_popcount(srcPtr[k] ^ dstPtr[k]);

                            if (hammingDistance < closestDistance) {
                                secondClosestDistance = closestDistance;
                                closestDistance = hammingDistance;
                                closestMatch = dstIdx;
                            } else if (hammingDistance < secondClosestDistance) {
                                secondClosestDistance = hammingDistance;
                            }
                        }
                    }

                dst[srcIdx].dstIdx = closestMatch;
                dst[srcIdx].bestDistance = closestDistance;
                dst[srcIdx].secondBestDistance = secondClosestDistance;
            }
        }
}

void Frame::matchWith(const Frame &other, const Eigen::Matrix3f &F, float maxDistance, std::vector<RawMatch> &dst) const
{

    dst.resize(m_keypoints.size());

    for (unsigned srcIdx = 0; srcIdx < m_keypoints.size(); srcIdx++) {

        const auto &kp = m_keypoints[srcIdx];

        Eigen::Vector3f epiLine = F * Eigen::Vector3f(kp.x_undistorted_times4, kp.y_undistorted_times4, 4.0f);
        epiLine /= epiLine.head<2>().norm();

        const std::uint32_t * __restrict__ srcPtr = (const std::uint32_t *) &m_descriptors[srcIdx * DESCRIPTOR_BYTES];
        unsigned closestMatch = 0;
        unsigned closestDistance = 10000;
        unsigned secondClosestDistance = 10000;

        for (unsigned dstIdx = 0; dstIdx < other.m_keypoints.size(); dstIdx++) {

            const auto &dstKp = other.m_keypoints[dstIdx];

            float distance = (
                epiLine[0] * dstKp.x_undistorted_times4 +
                epiLine[1] * dstKp.y_undistorted_times4 +
                epiLine[2] * 4.0f
            ) / 4.0f;

            if (std::abs(distance) < maxDistance) {

                const unsigned descWords = DESCRIPTOR_BYTES / 4;

                const std::uint32_t * __restrict__ dstPtr = (const std::uint32_t *) &other.m_descriptors[dstIdx * DESCRIPTOR_BYTES];

                unsigned hammingDistance = 0;
                for (unsigned k = 0; k < descWords; k++)
                    hammingDistance += __builtin_popcount(srcPtr[k] ^ dstPtr[k]);

                if (hammingDistance < closestDistance) {
                    secondClosestDistance = closestDistance;
                    closestDistance = hammingDistance;
                    closestMatch = dstIdx;
                } else if (hammingDistance < secondClosestDistance) {
                    secondClosestDistance = hammingDistance;
                }
            }
        }

        dst[srcIdx].dstIdx = closestMatch;
        dst[srcIdx].bestDistance = closestDistance;
        dst[srcIdx].secondBestDistance = secondClosestDistance;
    }

}