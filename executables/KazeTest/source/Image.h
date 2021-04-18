#pragma once

#include <cstdint>
#include <vector>

class Image {
    public:
        unsigned width() const { return m_width; }
        unsigned height() const { return m_height; }

        void allocate(unsigned w, unsigned h);

        std::uint8_t &operator()(unsigned x, unsigned y) { return m_ptr[x+y*m_stride]; }
        const std::uint8_t &operator()(unsigned x, unsigned y) const { return m_ptr[x+y*m_stride]; }
    protected:
        std::vector<std::uint8_t> m_data;
        std::uint8_t *m_ptr = nullptr;
        unsigned m_width = 0;
        unsigned m_stride = 0;
        unsigned m_height = 0;
};