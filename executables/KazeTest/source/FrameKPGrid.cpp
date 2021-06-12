#include "FrameKPGrid.h"


void FrameKPGrid::resize(unsigned rows, unsigned cols)
{
    m_rows = rows;
    m_cols = cols;
    m_cells.clear();
    m_cells.resize(m_rows * m_cols);
}

void FrameKPGrid::build(unsigned width, unsigned height, const std::vector<Keypoint> &keypoints)
{
    m_cells.clear();
    m_cells.resize(m_rows * m_cols);
    m_keypointIndices.clear();

    std::vector<std::pair<unsigned,unsigned>> kpRowCol;
    kpRowCol.resize(keypoints.size());

    for (unsigned i = 0; i < keypoints.size(); i++) {
        unsigned r, c;
        kpRowCol[i] = { r = keypoints[i].y * m_rows / height, c = keypoints[i].x * m_cols / width };
        m_cells[r * m_cols + c].idxCount++;
    }

    {
        unsigned cumulativeOffset = 0;
        for (unsigned i = 0; i < m_cells.size(); i++) {
            m_cells[i].idxStart = cumulativeOffset;
            cumulativeOffset += m_cells[i].idxCount;
        }
    }

    m_keypointIndices.resize(keypoints.size());
    std::vector<unsigned> cellFillState(m_cells.size(), 0);
    for (unsigned i = 0; i < keypoints.size(); i++) {
        auto [row, col] = kpRowCol[i];
        auto cellIdx = row * m_cols + col;

        m_keypointIndices[m_cells[cellIdx].idxStart + cellFillState[cellIdx]] = i;
        cellFillState[cellIdx]++;
    }
/*
    for (unsigned r = 0; r < m_rows; r++)
        for (unsigned c = 0; c < m_cols; c++) {
            unsigned xMin = c * width / m_cols;
            unsigned yMin = r * height / m_rows;
            unsigned xMax = (c + 1) * width / m_cols;
            unsigned yMax = (r + 1) * height / m_rows;

            for (auto kpIdx : (*this)(r, c)) {
                auto &kp = keypoints[kpIdx];
                if (kp.x < xMin || kp.x > xMax || kp.y < yMin || kp.y > yMax)
                    throw "huhu";
            }
        }
*/
}
