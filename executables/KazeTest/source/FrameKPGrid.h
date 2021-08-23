#pragma once

#include <span>
#include <vector>

#include "Keypoint.h"

class FrameKPGrid {
    public:
        void resize(unsigned rows, unsigned cols);

        std::span<const unsigned> operator()(unsigned r, unsigned c) const {
            const auto &cell = m_cells[r*m_cols+c];
            return std::span<const unsigned>(m_keypointIndices.data()+cell.idxStart, cell.idxCount);
        }

        void build(int startX, int startY, int endX, int endY, const std::vector<Keypoint> &keypoints);

        inline unsigned getRows() const { return m_rows; }
        inline unsigned getCols() const { return m_cols; }

        inline int getStartX() const { return m_startX; }
        inline int getStartY() const { return m_startY; }
        inline int getEndX() const { return m_endX; }
        inline int getEndY() const { return m_endY; }
    protected:
        unsigned m_rows = 0;
        unsigned m_cols = 0;

        int m_startX;
        int m_startY;
        int m_endX;
        int m_endY;

        struct Cell {
            unsigned idxStart = 0;
            unsigned idxCount = 0;
        };

        std::vector<Cell> m_cells;
        std::vector<unsigned> m_keypointIndices;
};

