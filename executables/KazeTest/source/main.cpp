#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <time.h>
#include <array>
#include <stdint.h>

class CPUStopWatch
{
    public:
        CPUStopWatch();

        void start();
        uint64_t getNanoseconds();
    protected:
        timespec m_start;
};

CPUStopWatch::CPUStopWatch()
{
    start();
}


void CPUStopWatch::start()
{
    clock_gettime(CLOCK_MONOTONIC_RAW, &m_start);
}

uint64_t CPUStopWatch::getNanoseconds()
{
    timespec stop;
    clock_gettime(CLOCK_MONOTONIC_RAW, &stop);

    if (stop.tv_sec > m_start.tv_sec) {
        return (int64_t)(stop.tv_sec - m_start.tv_sec) * (int64_t)1000000000l
                    + ((int64_t) stop.tv_nsec - (int64_t)m_start.tv_nsec);
    } else {
        return stop.tv_nsec - m_start.tv_nsec;
    }
}


#if 0

template<typename type, unsigned dim>
struct Vuint {
    Vuint() = default;
    Vuint(int v) { operator=(v); }
    Vuint(bool v) { operator=(v); }

    void load(const type *src) {
        for (unsigned i = 0; i < dim; i++)
            values[i] = src[i];
    }
    void store(type *dst) const {
        for (unsigned i = 0; i < dim; i++)
            dst[i] = values[i];
    }

    void operator=(int v) {
        for (unsigned i = 0; i < dim; i++)
            values[i] = v;
    }

    void operator=(bool v) {
        for (unsigned i = 0; i < dim; i++)
            values[i] = v?~0ull:0ull;
    }

    Vuint<type, dim> operator+(int i) const {
        auto res = *this;
        for (auto &v : res.values) v += i;
        return res;
    }
    Vuint<type, dim> operator-(int i) const {
        auto res = *this;
        for (auto &v : res.values) v -= i;
        return res;
    }
    Vuint<type, dim> operator+(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] + rhs.values[i];
        return res;
    }
    Vuint<type, dim> operator-(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] - rhs.values[i];
        return res;
    }

    Vuint<type, dim> operator&(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] & rhs.values[i];
        return res;
    }

    Vuint<type, dim> operator|(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] | rhs.values[i];
        return res;
    }

    void operator&=(const Vuint<type, dim> &rhs) {
        for (unsigned i = 0; i < dim; i++)
            values[i] &= rhs.values[i];
    }

    Vuint<type, dim> operator~() const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = ~values[i];
        return res;
    }

    Vuint<type, dim> operator<=(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] <= rhs.values[i])?~0ull:0ull;
        return res;
    }
    Vuint<type, dim> operator>=(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] >= rhs.values[i])?~0ull:0ull;
        return res;
    }

    void shiftLeftInsert(const Vuint<type, dim> &insert, unsigned shiftAmount) {
        for (unsigned i = 0; i < dim; i++) {
            type shifted = insert.values[i] << shiftAmount;
            type mask = ~(~0ull << shiftAmount);
            values[i] = (values[i] & mask) | shifted;
        }
    }

    Vuint<type, dim> bitTest(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] & rhs.values[i])?~0ull:0ull;
        return res;
    }


    std::array<type, dim> values;
};


template<typename type, unsigned dim>
Vuint<type, dim> saturatingAdd(const Vuint<type, dim> &lhs, const Vuint<type, dim> &rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::min<std::int64_t>((std::int64_t) lhs.values[i] + (std::int64_t)rhs.values[i], std::numeric_limits<type>::max());
    return res;
}

template<typename type, unsigned dim>
Vuint<type, dim> saturatingSub(const Vuint<type, dim> &lhs, const Vuint<type, dim> &rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::max<std::int64_t>((std::int64_t) lhs.values[i] - (std::int64_t)rhs.values[i], std::numeric_limits<type>::min());
    return res;
}

template<typename type, unsigned dim>
Vuint<type, dim> saturatingAdd(const Vuint<type, dim> &lhs, int rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::min<std::int64_t>((std::int64_t) lhs.values[i] + rhs, std::numeric_limits<type>::max());
    return res;
}

template<typename type, unsigned dim>
Vuint<type, dim> saturatingSub(const Vuint<type, dim> &lhs, int rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::max<std::int64_t>((std::int64_t) lhs.values[i] - rhs, std::numeric_limits<type>::min());
    return res;
}




using Vuint8x16 = Vuint<std::uint8_t, 16>;
using Vuint16x8 = Vuint<std::uint16_t, 8>;


Vuint16x8 zipLower(const Vuint8x16 &a, const Vuint8x16 &b) {
    Vuint16x8 res;

    for (unsigned i = 0; i < 8; i++)
        res.values[i] = (((std::uint16_t) a.values[i]) << 8) | b.values[i];

    return res;
}

Vuint16x8 zipUpper(const Vuint8x16 &a, const Vuint8x16 &b) {
    Vuint16x8 res;

    for (unsigned i = 0; i < 8; i++)
        res.values[i] = (((std::uint16_t) a.values[8+i]) << 8) | b.values[8+i];

    return res;
}


Vuint8x16 unzipLower(const Vuint16x8 &a, const Vuint16x8 &b) {
    Vuint8x16 res;

    for (unsigned i = 0; i < 8; i++)
        res.values[i] = a.values[i] & 0xFF;

    for (unsigned i = 0; i < 8; i++)
        res.values[8+i] = b.values[i] & 0xFF;

    return res;
}




bool any(const Vuint8x16 &v) {
    bool res = false;
    for (unsigned i = 0; i < 16; i++)
        res |= v.values[i];
    return res;
}


Vuint8x16 sel(const Vuint8x16 &sel, const Vuint8x16 &a, const Vuint8x16 &b);
bool all(const Vuint8x16 &v);


#else

#include <arm_neon.h>


struct Vuint8x16 {
    Vuint8x16() = default;
    Vuint8x16(int v) { operator=(v); }
    Vuint8x16(bool v) { operator=(v); }

    void load(const void *src) {
        values = vld1q_s8((const std::uint8_t*)src);
    }
    void store(void *dst) const {
        vst1q_s8((std::uint8_t*)dst, values);
    }

    void operator=(int v) {
        values = vdupq_n_s8(v);
    }

    void operator=(bool v) {
        values = vdupq_n_s8(v?~0ull:0ull);
    }

    Vuint8x16 operator&(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vandq_s8(values, rhs.values);
        return res;
    }

    void operator&=(const Vuint8x16 &rhs) {
        values = vandq_s8(values, rhs.values);
    }

    Vuint8x16 operator~() const {
        Vuint8x16 res;
        res.values = vmvnq_s8(values);
        return res;
    }

    Vuint8x16 operator<=(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vcleq_u8(values, rhs.values);
        return res;
    }
    Vuint8x16 operator>=(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vcgeq_u8(values, rhs.values);
        return res;
    }

    void shiftLeftInsert(const Vuint8x16 &insert, unsigned shiftAmount) {
        values = vsliq_n_s8(values, insert.values, shiftAmount);
    }

    uint8x16_t values;
};

struct Vuint16x8 {
    Vuint8x16() = default;
    Vuint8x16(int v) { operator=(v); }
    Vuint8x16(bool v) { operator=(v); }

    void operator=(int v) {
        values = vdupq_n_s16(v);
    }

    void operator=(bool v) {
        values = vdupq_n_s16(v?~0ull:0ull);
    }

    void operator&=(const Vuint8x16 &rhs) {
        values = vandq_s16(values, rhs.values);
    }

    Vuint8x16 bitTest(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vtstq_s16(values, rhs.values);
        return res;
    }


    uint16x8_t values;
};


Vuint8x16 saturatingAdd(const Vuint8x16 &lhs, int rhs) {
    Vuint8x16 res;
    res.values = vqaddq_s8(lhs.values, vdupq_n_s8(rhs));
    return res;
}

Vuint8x16 saturatingSub(const Vuint8x16 &lhs, int rhs) {
    Vuint8x16 res;
    res.values = vqsubq_s8(lhs.values, vdupq_n_s8(rhs));
    return res;
}


Vuint16x8 zipLower(const Vuint8x16 &a, const Vuint8x16 &b) {
    Vuint16x8 res;
    res.values = vzip1q_s8(a.values, b.values);
    return res;
}

Vuint16x8 zipUpper(const Vuint8x16 &a, const Vuint8x16 &b) {
    Vuint16x8 res;
    res.values = vzip2q_s8(a.values, b.values);
    return res;
}


Vuint8x16 unzipLower(const Vuint16x8 &a, const Vuint16x8 &b) {
    Vuint8x16 res;
    res.values = vuzp1q_s8(a.values, b.values);
    return res;
}


#endif





class Image {
    public:
        unsigned width() const { return m_width; }
        unsigned height() const { return m_height; }

        void allocate(unsigned w, unsigned h) {
            m_width = w;
            m_stride = (w+15) & ~15;
            m_height = h;
            m_data.resize(m_stride*m_height+15);
            m_ptr = m_data.data();
            m_ptr = (std::uint8_t *) (((size_t&)m_ptr + 15ull) & ~15ull);
        }

        std::uint8_t &operator()(unsigned x, unsigned y) { return m_ptr[x+y*m_stride]; }
        const std::uint8_t &operator()(unsigned x, unsigned y) const { return m_ptr[x+y*m_stride]; }
    protected:
        std::vector<std::uint8_t> m_data;
        std::uint8_t *m_ptr = nullptr;
        unsigned m_width = 0;
        unsigned m_stride = 0;
        unsigned m_height = 0;
};

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

    const unsigned t = 10;

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

                halfCircleNotBigger_0.shiftLeftInsert(notBigger, tap);
                halfCircleNotSmaller_0.shiftLeftInsert(notSmaller, tap);
            }
            Vuint8x16 halfCircleNotBigger_1 = 0;
            Vuint8x16 halfCircleNotSmaller_1 = 0;
            for (unsigned tap = 8; tap < 16; tap++) {
                Vuint8x16 tapPixel;
                tapPixel.load(&img(x+taps[tap][0], y+taps[tap][1]));

                Vuint8x16 notBigger = tapPixel <= centralPixelPlusThresh;
                Vuint8x16 notSmaller = tapPixel >= centralPixelMinusThresh;

                halfCircleNotBigger_1.shiftLeftInsert(notBigger, tap-8);
                halfCircleNotSmaller_1.shiftLeftInsert(notSmaller, tap-8);
            }

            Vuint16x8 lowerWarpCircleNotBigger = zipLower(halfCircleNotBigger_0, halfCircleNotBigger_1);
            Vuint16x8 upperWarpCircleNotBigger = zipUpper(halfCircleNotBigger_0, halfCircleNotBigger_1);

            Vuint16x8 lowerWarpCircleNotSmaller = zipLower(halfCircleNotSmaller_0, halfCircleNotSmaller_1);
            Vuint16x8 upperWarpCircleNotSmaller = zipUpper(halfCircleNotSmaller_0, halfCircleNotSmaller_1);

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

            isKeypoint.store(&dst(x, y));
        }

}



void slow(const Image &img, Image &dst)
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

    const unsigned t = 10;

    for (unsigned y = 3; y+3 < img.height(); y++)
        for (unsigned x = 3; x+3 < img.width(); x++) {
            int centralPixel = img(x, y);

            int centralPixelPlusThresh = std::min<int>(centralPixel + t, 255);
            int centralPixelMinusThresh = std::max<int>(centralPixel - t, 0);


            bool bigger[16];
            bool smaller[16];
            for (unsigned tap = 0; tap < 16; tap++) {
                int tapPixel = img(x+taps[tap][0], y+taps[tap][1]);

                bigger[tap] = tapPixel > centralPixelPlusThresh;
                smaller[tap] = tapPixel < centralPixelMinusThresh;
            }

            unsigned countBigger = 0;
            unsigned countSmaller = 0;
            bool isKeypoint = false;
            for (unsigned tap_ = 0; tap_ < 16+12; tap_++) {
                unsigned tap = tap_ % 16;
                if (bigger[tap]) {
                    countSmaller = 0;
                    countBigger++;
                    if (countBigger >= 12) {
                        isKeypoint = true;
                        break;
                    }
                } else if (smaller[tap]) {
                    countSmaller++;
                    countBigger = 0;
                    if (countSmaller >= 12) {
                        isKeypoint = true;
                        break;
                    }
                } else {
                    countSmaller = 0;
                    countBigger = 0;
                }
            }

            dst(x, y) = isKeypoint?255:0;
        }

}


/*

saturating add:
vqaddq_s8
vqsubq_s8

comparison:
vcgeq_u8

shift-insert bits:
vsliq_n_s8

2x8bit -> 16bit
vzip1q_s8
vzip1q_s8

(a & b != 0)
vtstq_s16

vuzp1q_s8

*/


int main() {

    auto imgA = cv::imread("/home/andy/Documents/CAD/AppCVRobot/data/fullHDTest_autoIso_640_480_frames/frame0126.png", cv::IMREAD_GRAYSCALE);
    auto imgB = cv::imread("/home/andy/Documents/CAD/AppCVRobot/data/fullHDTest_autoIso_640_480_frames/frame0128.png");

    //cv::cvtColor(imgA, cv::COLOR_BGR2GRAY);

    Image inputImg;
    inputImg.allocate(imgA.cols, imgA.rows);
    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            inputImg(x, y) = imgA.at<std::uint8_t>(y, x);
        }

    Image outputImg;
    outputImg.allocate(imgA.cols, imgA.rows);

    fast(inputImg, outputImg);

    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            imgA.at<std::uint8_t>(y, x) = outputImg(x, y);
        }

    cv::imwrite("fast.png", imgA);

    cv::imshow("image", imgA);
    cv::waitKey(0);


    slow(inputImg, outputImg);

    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            imgA.at<std::uint8_t>(y, x) = outputImg(x, y);
        }

    cv::imwrite("slow.png", imgA);

    cv::imshow("image", imgA);
    cv::waitKey(0);

#if 0

    cv::Mat descsA;
    std::vector<cv::KeyPoint> keypointsA;
    cv::Mat descsB;
    std::vector<cv::KeyPoint> keypointsB;

    //auto akaze = cv::AKAZE::create();
    auto akaze = cv::ORB::create(5000);

    auto fast = cv::FastFeatureDetector::create();
//    auto brief = cv::BriefDescriptorExtractor::create();

    CPUStopWatch timer;
#if 1
    for (unsigned i = 0; i < 10; i++) {
        akaze->detectAndCompute(imgA, cv::noArray(), keypointsA, descsA);
        akaze->detectAndCompute(imgB, cv::noArray(), keypointsB, descsB);
    }
#else
    for (unsigned i = 0; i < 10; i++) {
        fast->detect(imgA, keypointsA);
     //   brief->compute(imgA, keypointsA, descsA);
        fast->detect(imgB, keypointsB);
     //   brief->compute(imgB, keypointsB, descsB);
        akaze->compute(imgA, keypointsA, descsA);
        akaze->compute(imgB, keypointsB, descsB);
    }
#endif
    std::cout << timer.getNanoseconds() * 1e-9f / 20 << " seconds/image" << std::endl;

    cv::Mat outImageA;
    cv::drawKeypoints(imgA, keypointsA, outImageA);
    cv::Mat outImageB;
    cv::drawKeypoints(imgB, keypointsB, outImageB);

    cv::imshow("outImageA", outImageA);
    cv::imshow("outImageB", outImageB);
    cv::waitKey(0);


    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(descsA, descsB, matches, 2);

    float thresh = 0.7f;
    std::vector<cv::DMatch> filteredMatches;
    for (auto &v : matches) {
        auto &first = v[0];
        auto &second = v[1];
        if (thresh * second.distance > first.distance)
            filteredMatches.push_back(v.front());
    }


    cv::Mat outImage;

    cv::drawMatches (imgA, keypointsA, imgB, keypointsB, filteredMatches, outImage);

    cv::imshow("outImage", outImage);
    cv::waitKey(0);
#endif
    return 0;
}