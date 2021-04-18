#pragma once

#include "Keypoint.h"
#include "Image.h"
#include "simd.h"

#include <vector>


void fast(const Image &img, Image &dst);

template<unsigned i>
void emitCoords(unsigned x, unsigned y, const Vuint8x16 &centralPixel, std::vector<Keypoint> &coords)
{
    auto fourPixels = centralPixel.extract32B<i>();
    if (fourPixels != 0) {
        for (unsigned j = 0; j < 4; j++) {
            std::uint8_t strength = (fourPixels >> (j*8)) & 0xFF;
            if (strength) {
                unsigned locX = x + i*4 + j;
                coords.push_back({locX, y, strength});
            }
        }
    }
}

template<bool writeBackImg>
void nonMaxSuppress(Image &img, unsigned borderSize, std::vector<Keypoint> &coords)
{
    borderSize = std::max<unsigned>(borderSize, 1);

    coords.clear();
    for (unsigned y = borderSize; y+borderSize < img.height(); y++)
        for (unsigned x = borderSize; x+borderSize < img.width(); x+=16) {
            Vuint8x16 centralPixel;
            centralPixel.load(&img(x, y));

            if (!any(centralPixel)) continue;

            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                    if ((i != 0) || (j != 0)) {
                        Vuint8x16 neighbor;
                        neighbor.load(&img((int)x+j, (int)y+i));

                        Vuint8x16 larger = centralPixel > neighbor;
                        centralPixel = centralPixel & larger;
                    }

            if (writeBackImg)
                centralPixel.store(&img(x, y));

            if (any(centralPixel)) {
                emitCoords<0>(x, y, centralPixel, coords);
                emitCoords<1>(x, y, centralPixel, coords);
                emitCoords<2>(x, y, centralPixel, coords);
                emitCoords<3>(x, y, centralPixel, coords);
            }
        }
}