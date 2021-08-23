#include "FrameKPGrid.h"


void FrameKPGrid::resize(unsigned rows, unsigned cols)
{
    m_rows = rows;
    m_cols = cols;
    m_cells.clear();
    m_cells.resize(m_rows * m_cols);
}

void FrameKPGrid::build(int startX, int startY, int endX, int endY, const std::vector<Keypoint> &keypoints)
{
    m_startX = startX;
    m_startY = startY;
    m_endX = endX;
    m_endY = endY;

    m_cells.clear();
    m_cells.resize(m_rows * m_cols);
    m_keypointIndices.clear();

    std::vector<std::pair<unsigned,unsigned>> kpRowCol;
    kpRowCol.resize(keypoints.size());

    int width_times4 = (endX - startX) * 4;
    int height_times4 = (endY - startY) * 4;

    for (unsigned i = 0; i < keypoints.size(); i++) {
        int r, c;
        r = (keypoints[i].y_undistorted_times4 - startX*4) * m_rows / height_times4;
        c = (keypoints[i].x_undistorted_times4 - startY*4) * m_cols / width_times4;

        r = std::min<int>(std::max(r, 0), m_rows-1);
        c = std::min<int>(std::max(c, 0), m_cols-1);

        kpRowCol[i] = {r, c};
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
