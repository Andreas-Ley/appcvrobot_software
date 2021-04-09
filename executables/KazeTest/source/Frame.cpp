#include "Frame.h"

#include "Image.h"
#include "SlowBrief.h"


void fast(const Image &img, Image &dst);
void nonMaxSuppressNWB(Image &img, unsigned borderSize, std::vector<Keypoint> &coords);

void Frame::extractKeypoints(const Image &img, const SlowBrief &brief)
{
    m_width = img.width();
    m_height = img.height();

    m_keypoints.reserve(2000);

    Image fastDetections;
    fast(img, fastDetections);
    nonMaxSuppressNWB(fastDetections, brief.getPatternExtend(), m_keypoints);

    std::vector<bool> valid;
    brief(img, m_keypoints, m_descriptors, valid);
}

void Frame::buildKPGrid()
{
    m_kpGrid.resize(m_height / 40, m_width / 40);
    m_kpGrid.build(m_width, m_height, m_keypoints);
}

void Frame::matchWith(const Frame &other, std::vector<RawMatch> &dst)
{
    dst.resize(m_keypoints.size());
    for (unsigned r = 0; r < m_kpGrid.getRows(); r++)
        for (unsigned c = 0; c < m_kpGrid.getCols(); c++) {
            auto cell = m_kpGrid(r, c);

            for (auto srcIdx : cell) {
                const auto srcKp = m_keypoints[srcIdx];
                unsigned dstColx2 = srcKp.x * other.getKPGrid().getCols()*2 / other.getWidth();
                unsigned dstRowx2 = srcKp.y * other.getKPGrid().getRows()*2 / other.getHeight();

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

                for (unsigned r2 = startRow; r2 < endRow; r2++)
                    for (unsigned c2 = startCol; c2 < endCol; c2++) {
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
/*
                const unsigned ratio_num = 2;
                const unsigned ratio_denom = 3;

                if (closestDistance < 100 && closestDistance * ratio_denom < secondClosestDistance * ratio_num) {

                }
*/
            }
        }
}
