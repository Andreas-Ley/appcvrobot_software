#include "Fast.h"


void fast(const Image &img, Image &dst)
{
    const int taps[16][2] = {
        {0, -3},
        {1, -3},
        {2, -2},
        {3, -1},
        {3, 0},
        {3, 1},
        {2, 2},
        {1, 3},
        {0, 3},
        {-1, 3},
        {-2, 2},
        {-3, 1},
        {-3, 0},
        {-3, -1},
        {-2, -2},
        {-1, -3},
    };

    const unsigned t = 5;

    for (unsigned y = 3; y+3 < img.height(); y++)
        for (unsigned x = 3; x+3 < img.width(); x+=16) {
            Vuint8x16 centralPixel;
            centralPixel.load(&img(x, y));

            Vuint8x16 centralPixelPlusThresh = saturatingAdd(centralPixel, t);
            Vuint8x16 centralPixelMinusThresh = saturatingSub(centralPixel, t);


            Vuint8x16 halfCircleNotBigger_0 = 0;
            Vuint8x16 halfCircleNotSmaller_0 = 0;
            for (unsigned tap = 0; tap < 8; tap++) {
                Vuint8x16 tapPixel;
                tapPixel.load(&img(x+taps[tap][0], y+taps[tap][1]));

                Vuint8x16 notBigger = tapPixel <= centralPixelPlusThresh;
                Vuint8x16 notSmaller = tapPixel >= centralPixelMinusThresh;

                #ifdef BUILD_FOR_ARM
                    halfCircleNotBigger_0.shiftLeftInsert(notBigger, tap);
                    halfCircleNotSmaller_0.shiftLeftInsert(notSmaller, tap);
                #else
                    halfCircleNotBigger_0 |= notBigger & Vuint8x16(1 << tap);
                    halfCircleNotSmaller_0 |= notSmaller & Vuint8x16(1 << tap);
                #endif
            }
            Vuint8x16 halfCircleNotBigger_1 = 0;
            Vuint8x16 halfCircleNotSmaller_1 = 0;
            for (unsigned tap = 8; tap < 16; tap++) {
                Vuint8x16 tapPixel;
                tapPixel.load(&img(x+taps[tap][0], y+taps[tap][1]));

                Vuint8x16 notBigger = tapPixel <= centralPixelPlusThresh;
                Vuint8x16 notSmaller = tapPixel >= centralPixelMinusThresh;

                #ifdef BUILD_FOR_ARM
                    halfCircleNotBigger_1.shiftLeftInsert(notBigger, tap-8);
                    halfCircleNotSmaller_1.shiftLeftInsert(notSmaller, tap-8);
                #else
                    halfCircleNotBigger_1 |= notBigger & Vuint8x16(1 << (tap-8));
                    halfCircleNotSmaller_1 |= notSmaller & Vuint8x16(1 << (tap-8));
                #endif
            }

            Vuint16x8 lowerWarpCircleNotBigger;
            Vuint16x8 upperWarpCircleNotBigger;
            zip(halfCircleNotBigger_0, halfCircleNotBigger_1, lowerWarpCircleNotBigger, upperWarpCircleNotBigger);

            Vuint16x8 lowerWarpCircleNotSmaller;
            Vuint16x8 upperWarpCircleNotSmaller;
            zip(halfCircleNotSmaller_0, halfCircleNotSmaller_1, lowerWarpCircleNotSmaller, upperWarpCircleNotSmaller);

            const std::uint16_t bitMasks[] = {
                0b0000111111111111,
                0b0001111111111110,
                0b0011111111111100,
                0b0111111111111000,
                0b1111111111110000,
                0b1111111111100001,
                0b1111111111000011,
                0b1111111110000111,
                0b1111111100001111,
                0b1111111000011111,
                0b1111110000111111,
                0b1111100001111111,
                0b1111000011111111,
                0b1110000111111111,
                0b1100001111111111,
                0b1000011111111111,
            };

            Vuint16x8 lowerWarpNoneBigger = true;
            Vuint16x8 lowerWarpNoneSmaller = true;
            Vuint16x8 upperWarpNoneBigger = true;
            Vuint16x8 upperWarpNoneSmaller = true;
            for (unsigned startIdx = 0; startIdx < 16; startIdx++) {
                Vuint16x8 mask = bitMasks[startIdx];
                lowerWarpNoneBigger &= lowerWarpCircleNotBigger.bitTest(mask);
                lowerWarpNoneSmaller &= lowerWarpCircleNotSmaller.bitTest(mask);
                upperWarpNoneBigger &= upperWarpCircleNotBigger.bitTest(mask);
                upperWarpNoneSmaller &= upperWarpCircleNotSmaller.bitTest(mask);
            }

            Vuint8x16 noneBigger = unzipLower(lowerWarpNoneBigger, upperWarpNoneBigger);
            Vuint8x16 noneSmaller = unzipLower(lowerWarpNoneSmaller, upperWarpNoneSmaller);

            Vuint8x16 isKeypoint = ~(noneBigger & noneSmaller);

            if (any(isKeypoint)) {

                Vuint8x16 strength = 0;
                for (unsigned tap = 0; tap < 16; tap++) {
                    Vuint8x16 tapPixel;
                    tapPixel.load(&img(x+taps[tap][0], y+taps[tap][1]));

                    Vuint8x16 v = absDiff(centralPixel, tapPixel);
                    v >>= 2;

                    strength = saturatingAdd(strength, v);
                }
                isKeypoint = isKeypoint & strength;
            }

            isKeypoint.store(&dst(x, y));
        }

}