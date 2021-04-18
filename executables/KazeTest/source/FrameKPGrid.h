#pragma once

#include <span>
#include <vector>

#include "Keypoint.h"

class FrameKPGrid {
    public:
        void resize(unsigned rows, unsigned cols);

        std::span<const unsigned> operator()(unsigned r, unsigned c) const {
            const auto &cell = m_cells[r*m_cols+c];
            return std::span<const unsigned>(m_keypointIndices.data()+cell.idxStart, cell.idxStart+cell.idxCount);
        }

        void build(unsigned width, unsigned height, const std::vector<Keypoint> &keypoints);

        inline unsigned getRows() const { return m_rows; }
        inline unsigned getCols() const { return m_cols; }
    protected:
        unsigned m_rows = 0;
        unsigned m_cols = 0;

        struct Cell {
            unsigned idxStart = 0;
            unsigned idxCount = 0;
        };

        std::vector<Cell> m_cells;
        std::vector<unsigned> m_keypointIndices;
};

