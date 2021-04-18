#include "Image.h"


void Image::allocate(unsigned w, unsigned h) {
    m_width = w;
    m_stride = (w+15) & ~15;
    m_height = h;
    m_data.resize(m_stride*m_height+15);
    m_ptr = m_data.data();
    m_ptr = (std::uint8_t *) (((size_t&)m_ptr + 15ull) & ~15ull);
}
