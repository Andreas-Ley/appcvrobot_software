
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Image.h"
#include "Frame.h"
#include "Keypoint.h"
#include "SlowBrief.h"
#include "Fast.h"
#include "RansacE.h"
#include "RansacH.h"
#include "MVGMath.h"
#include "SLAM.h"

#include <iostream>
#include <time.h>
#include <array>
#include <stdint.h>
#include <vector>
#include <cstdint>
#include <random>
#include <tuple>
#include <fstream>

//#include <span>
#include <algorithm>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "simd.h"

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


void nonMaxSuppressNWB(Image &img, unsigned borderSize, std::vector<Keypoint> &coords)
{
    nonMaxSuppress<false>(img, borderSize, coords);
}


template<unsigned dimension = 256>
void slowMatch(const std::vector<std::uint8_t> &descriptorsA, const std::vector<std::uint8_t> &descriptorsB, std::vector<std::tuple<unsigned, unsigned>> &matches)
{
    unsigned numDescriptorsA = descriptorsA.size() / (dimension/8);
    unsigned numDescriptorsB = descriptorsB.size() / (dimension/8);
    const unsigned descWords = dimension / 32;
    matches.resize(numDescriptorsA);
    for (unsigned i = 0; i < numDescriptorsA; i++) {
        if (i % 100 == 0)
            std::cout << i << " of " << numDescriptorsA << std::endl;
        const std::uint32_t *srcPtr = (const std::uint32_t *) &descriptorsA[i * (dimension/8)];

        unsigned bestMatch = 0;
        unsigned bestDistance = ~0u;

        for (unsigned j = 0; j < numDescriptorsB; j++) {
            const std::uint32_t *dstPtr = (const std::uint32_t *) &descriptorsB[j * (dimension/8)];

            unsigned hammingDistance = 0;
            for (unsigned k = 0; k < descWords; k++)
                hammingDistance += __builtin_popcount(srcPtr[k] ^ dstPtr[k]);

            if (hammingDistance < bestDistance) {
                bestDistance = hammingDistance;
                bestMatch = j;
            }
        }

        matches[i] = {bestMatch, bestDistance};
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

    const unsigned t = 5;

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

            if (isKeypoint) {
                unsigned strength = 0;
                for (unsigned tap = 0; tap < 16; tap++) {
                    int tapPixel = img(x+taps[tap][0], y+taps[tap][1]);
                    unsigned diff = std::abs(centralPixel - tapPixel);
                    strength += diff;
                }
                dst(x, y) = strength;
            } else
                dst(x, y) = 0;
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

/*

Load:
__m128i _mm_load_si128 (__m128i const* mem_addr)

Store:
void _mm_store_si128 (__m128i* mem_addr, __m128i a)

Broadcast assign:
__m128i _mm_set1_epi8 (char a)

Broadcast assign bool:
__m128i _mm_setzero_si128 ()

&:
__m128i _mm_and_si128 (__m128i a, __m128i b)

>>:
v = _mm_srli_epi16(v, shift_amount);
v = _mm_and_si128(v, _mm_set1_epi8(255u >> shift_amount));

&~:
__m128i _mm_andnot_si128 (__m128i a, __m128i b)

>:
__m128i _mm_cmpgt_epi8 (__m128i a, __m128i b)

<:
__m128i _mm_cmplt_epi8 (__m128i a, __m128i b)
==:
__m128i _mm_cmpeq_epi8 (__m128i a, __m128i b)

|:
__m128i _mm_or_si128 (__m128i a, __m128i b)

shiftLeftInsert: no
bitTest: no

extract32B:
int _mm_extract_epi32 (__m128i a, const int imm8)

saturatingAdd:
__m128i _mm_adds_epu8 (__m128i a, __m128i b)

saturatingSub:
__m128i _mm_subs_epu8 (__m128i a, __m128i b)

absDiff:
__m128i abs_sub_epu8(const __m128i a, const __m128i b) {

    const __m128i ab = _mm_subs_epu8(a, b);
    const __m128i ba = _mm_subs_epu8(b, a);

    return _mm_or_si128(ab, ba);
}

zip:

__m128i _mm_unpacklo_epi8 (__m128i a, __m128i b)
__m128i _mm_unpackhi_epi8 (__m128i a, __m128i b)

unzipLower:
__m128i _mm_packus_epi16 (__m128i a, __m128i b)

any:
int _mm_testz_si128 (__m128i a, __m128i b)


 */

int main() {
    SlowBrief slowBrief;

#ifdef BUILD_FOR_ARM
    auto imgA = cv::imread("/home/pi/frame0126.png", cv::IMREAD_GRAYSCALE);
    auto imgB = cv::imread("/home/pi/frame0130.png", cv::IMREAD_GRAYSCALE);
#else
    auto imgA = cv::imread("/home/andy/Documents/CAD/AppCVRobot/data/fullHDTest_autoIso_640_480_frames/frame0126.png", cv::IMREAD_GRAYSCALE);
    auto imgB = cv::imread("/home/andy/Documents/CAD/AppCVRobot/data/fullHDTest_autoIso_640_480_frames/frame0128.png", cv::IMREAD_GRAYSCALE);
    //auto imgB = cv::imread("/home/andy/Documents/CAD/AppCVRobot/data/fullHDTest_autoIso_640_480_frames/frame0130.png", cv::IMREAD_GRAYSCALE);
//    auto imgA = cv::imread("/home/irina/libcompile/software-appcvrobot/data/IMG_3745.JPG", cv::IMREAD_GRAYSCALE);
    //auto imgB = cv::imread("/home/irina/libcompile/software-appcvrobot/data/IMG_3746.JPG", cv::IMREAD_GRAYSCALE);
#endif

#if 0



    Image inputImg;
    inputImg.allocate(imgA.cols, imgA.rows);
    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            inputImg(x, y) = imgA.at<std::uint8_t>(y, x);
        }

    Image outputImg;
    outputImg.allocate(imgA.cols, imgA.rows);

    std::vector<std::tuple<std::uint16_t, std::uint16_t, std::uint8_t>> coordsA;
    coordsA.reserve(10000);

    fast(inputImg, outputImg);
    nonMaxSuppress<false>(outputImg, coordsA);

    std::vector<std::uint8_t> descriptorsA;
    std::vector<bool> validA;
    slowBrief(inputImg, coordsA, descriptorsA, validA);


    inputImg.allocate(imgB.cols, imgB.rows);
    for (unsigned y = 0; y < imgB.rows; y++)
        for (unsigned x = 0; x < imgB.cols; x++) {
            inputImg(x, y) = imgB.at<std::uint8_t>(y, x);
        }

    outputImg.allocate(imgB.cols, imgB.rows);

    std::vector<std::tuple<std::uint16_t, std::uint16_t, std::uint8_t>> coordsB;
    coordsB.reserve(10000);

    slow(inputImg, outputImg);
    nonMaxSuppress<false>(outputImg, coordsB);

    std::vector<std::uint8_t> descriptorsB;
    std::vector<bool> validB;
    slowBrief(inputImg, coordsB, descriptorsB, validB);



    std::vector<std::tuple<unsigned, unsigned>> matches;
    slowMatch(descriptorsA, descriptorsB, matches);

    auto coords2keypoints = [](auto &coords, auto &keypoints) {
        keypoints.resize(coords.size());
        for (unsigned i = 0; i < coords.size(); i++) {
            auto [x, y, s] = coords[i];
            keypoints[i] = cv::KeyPoint({x, y}, 5);
        }
    };

    std::vector<cv::KeyPoint> keypointsA;
    coords2keypoints(coordsA, keypointsA);
    std::vector<cv::KeyPoint> keypointsB;
    coords2keypoints(coordsB, keypointsB);

    std::vector<cv::DMatch> filteredMatches;
    for (unsigned i = 0; i < matches.size(); i++) {
        auto [idx, distance] = matches[i];
        std::cout << idx << " " << distance << std::endl;
        if (distance > 40) continue;
        if (!validA[i]) continue;
        //if (!validB[matches[i]]) continue;

        filteredMatches.push_back(cv::DMatch(i, idx, distance));
    }


    cv::Mat outImage;
    cv::drawMatches (imgA, keypointsA, imgB, keypointsB, filteredMatches, outImage);

    cv::imwrite("matches_ours.png", outImage);
#ifndef BUILD_FOR_ARM
    cv::imshow("outImage", outImage);
    cv::waitKey(0);
#endif

#elif 1

    Eigen::Matrix3f K;
    K.setZero();
    K(0, 0) = 3.1227664660089198e+02;
    K(1, 1) = 3.1227664660089198e+02;

    K(0, 2) = 3.2310723583689497e+02;
    K(1, 2) = 2.5560322207748723e+02;
    K(2, 2) = 1.0f;

    Eigen::Vector3f distortion(0.0f, 0.0f, 0.0f);

    SLAM slam({.internalCalib = K, .distortion_k = distortion});


    Image inputImgA, inputImgB;
    inputImgA.allocate(imgA.cols, imgA.rows);
    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++)
            inputImgA(x, y) = imgA.at<std::uint8_t>(y, x);

    inputImgB.allocate(imgB.cols, imgB.rows);
    for (unsigned y = 0; y < imgB.rows; y++)
        for (unsigned x = 0; x < imgB.cols; x++)
            inputImgB(x, y) = imgB.at<std::uint8_t>(y, x);


    slam.tryIngestImage(inputImgA);
    slam.tryIngestImage(inputImgB);


#elif 1


    Image inputImgA, inputImgB;
    inputImgA.allocate(imgA.cols, imgA.rows);
    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++)
            inputImgA(x, y) = imgA.at<std::uint8_t>(y, x);

    inputImgB.allocate(imgB.cols, imgB.rows);
    for (unsigned y = 0; y < imgB.rows; y++)
        for (unsigned x = 0; x < imgB.cols; x++)
            inputImgB(x, y) = imgB.at<std::uint8_t>(y, x);



    Frame frameA, frameB;
    frameA.extractKeypoints(inputImgA, slowBrief);
    frameB.extractKeypoints(inputImgB, slowBrief);

    frameA.buildKPGrid();
    frameB.buildKPGrid();

    std::vector<RawMatch> rawMatches;
    CPUStopWatch timer;
    unsigned num_iter = 100;
    for (unsigned i = 0; i < num_iter; i++) {
        rawMatches.clear();
        frameA.matchWith(frameB, rawMatches);
    }
    std::cout << timer.getNanoseconds() * 1e-9f / num_iter << " seconds/pair-match" << std::endl;



    auto coords2keypoints = [](auto &coords, auto &keypoints) {
        keypoints.resize(coords.size());
        for (unsigned i = 0; i < coords.size(); i++) {
            keypoints[i] = cv::KeyPoint({coords[i].x, coords[i].y}, 5);
        }
    };

    std::vector<cv::KeyPoint> keypointsA;
    coords2keypoints(frameA.getKeypoints(), keypointsA);
    std::vector<cv::KeyPoint> keypointsB;
    coords2keypoints(frameB.getKeypoints(), keypointsB);

    std::vector<cv::DMatch> filteredMatches;
    for (unsigned i = 0; i < rawMatches.size(); i++) {
        auto &match = rawMatches[i];
        if (match.bestDistance > 80) continue;

        if (match.bestDistance*3 > match.secondBestDistance *2) continue;

        filteredMatches.push_back(cv::DMatch(i, match.dstIdx, match.bestDistance));
    }


    cv::Mat outImage;
    cv::drawMatches (imgA, keypointsA, imgB, keypointsB, filteredMatches, outImage);

    cv::imwrite("matches_ours.png", outImage);


    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> pairs;

    Eigen::Matrix3f K;
    K.setZero();
    K(0, 0) = 3.1227664660089198e+02;
    K(1, 1) = 3.1227664660089198e+02;

    K(0, 2) = 3.2310723583689497e+02;
    K(1, 2) = 2.5560322207748723e+02;
    K(2, 2) = 1.0f;

    Eigen::Matrix3f KInv = K.inverse();

    for (unsigned i = 0; i < rawMatches.size(); i++) {
        auto &match = rawMatches[i];
        if (match.bestDistance > 80) continue;

        if (match.bestDistance*3 > match.secondBestDistance *2) continue;

        const auto &kpA = frameA.getKeypoints()[i];
        const auto &kpB = frameB.getKeypoints()[match.dstIdx];

        Eigen::Vector3f locA = KInv * Eigen::Vector3f(kpA.x, kpA.y, 1.0f);
        Eigen::Vector3f locB = KInv * Eigen::Vector3f(kpB.x, kpB.y, 1.0f);

        pairs.push_back({locA.head<2>()/locA[2], locB.head<2>()/locB[2]});
    }


    std::vector<float> distances;
    RansacE ransac;
    float thresh = 0.01f;
    ransac.setMaxDistance(thresh);

    Eigen::Matrix3f E;
    unsigned numInliers = ransac.process(pairs, distances, E);
    std::cout << "E: " << E << std::endl;

    std::cout << "Inliers: " << numInliers << std::endl;


    {
        std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> Hpairs;
        Hpairs.reserve(pairs.size());
        for (unsigned i = 0; i < pairs.size(); i++)
            if (distances[i] < thresh)
                Hpairs.push_back(pairs[i]);

        RansacH ransacH;
        ransacH.setMaxDistance(thresh);
        Eigen::Matrix3f H;
        unsigned numInliersH = ransacH.process(pairs, H);

        std::cout << "H: " << H << std::endl;

        std::cout << "InliersH: " << numInliersH << " of " << numInliers << std::endl;
    }


    {
        std::vector<cv::DMatch> filteredMatches;
        unsigned j = 0;
        for (unsigned i = 0; i < rawMatches.size(); i++) {
            auto &match = rawMatches[i];
            if (match.bestDistance > 80) continue;

            if (match.bestDistance*3 > match.secondBestDistance *2) continue;

            if (distances[j++] < thresh)
                filteredMatches.push_back(cv::DMatch(i, match.dstIdx, match.bestDistance));
        }


        cv::Mat outImage;
        cv::drawMatches (imgA, keypointsA, imgB, keypointsB, filteredMatches, outImage);

        cv::imwrite("matches_ours_ransaced.png", outImage);
    }


    EDecomposer eDecomposer(E);

    unsigned bestVariation = 0;
    unsigned bestVariationInFront = 0;

    for (unsigned variation = 0; variation < 4; variation++) {

        Eigen::Matrix<float, 3, 4> P1, P2;
        P1.setIdentity();
        P2 = eDecomposer.getP(variation);

        unsigned inFront = 0;
        PointTriangulator triangulator(P1, P2);
        for (unsigned i = 0; i < pairs.size(); i++) {
            if (distances[i] > thresh) continue;
            auto &p = pairs[i];

            Eigen::Vector4f loc3D = triangulator.triangulate(p.first, p.second);

            float proj_z = P2.block<1, 4>(2,0) * loc3D;

            if (loc3D[2] > 0.0f == loc3D[3] > 0.0f &&
                proj_z > 0.0f == loc3D[3] > 0.0f)
                inFront++;

        }
        std::cout << "Variation " << variation << " has " << inFront << " in front." << std::endl;

        if (inFront > bestVariationInFront) {
            bestVariationInFront = inFront;
            bestVariation = variation;
        }
    }

    Eigen::Matrix<float, 3, 4> P1, P2;
    P1.setIdentity();
    P2 = eDecomposer.getP(bestVariation);


    std::vector<Eigen::Vector3f> tracks;
    tracks.reserve(pairs.size());

    PointTriangulator triangulator(P1, P2);
    for (unsigned i = 0; i < pairs.size(); i++) {
        if (distances[i] > thresh) continue;
        auto &p = pairs[i];

        Eigen::Vector4f loc3D = triangulator.triangulate(p.first, p.second);

        float proj_z1 = P1.block<1, 4>(2,0) * loc3D;
        float proj_z2 = P2.block<1, 4>(2,0) * loc3D;

        if (proj_z1 > 0.0f == loc3D[3] > 0.0f &&
            proj_z2 > 0.0f == loc3D[3] > 0.0f) {

            if (std::abs(loc3D[3]) > 1e-20f)
                tracks.push_back(loc3D.head<3>() / loc3D[3]);
        }
    }

    std::fstream plyFile("tracks.ply", std::fstream::out);
    plyFile
        << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << tracks.size() << std::endl
        << "property float x" << std::endl
        << "property float y" << std::endl
        << "property float z" << std::endl
        << "end_header" << std::endl;

    for (auto &t : tracks)
        plyFile << t[0] << ' ' << t[1] << ' ' << t[2] << std::endl;

#elif 1

    //cv::cvtColor(imgA, cv::COLOR_BGR2GRAY);

    Image inputImg;
    inputImg.allocate(imgA.cols, imgA.rows);
    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            inputImg(x, y) = imgA.at<std::uint8_t>(y, x);
        }

    Image outputImg;
    outputImg.allocate(imgA.cols, imgA.rows);

    std::vector<Keypoint> coords;
    coords.reserve(10000);

    CPUStopWatch timer;
    unsigned num_iter = 100;
    for (unsigned i = 0; i < num_iter; i++) {
        fast(inputImg, outputImg);
        nonMaxSuppress<true>(outputImg, slowBrief.getPatternExtend(), coords);
    }
    std::cout << timer.getNanoseconds() * 1e-9f / num_iter << " seconds/image" << std::endl;


//    timer.start();
//    for (unsigned i = 0; i < 50; i++) {
//	    std::vector<std::uint8_t> descriptorsA;
//	    std::vector<bool> validA;
//        slowBrief(inputImg, coords, descriptorsA, validA);
//    }
//    std::cout << timer.getNanoseconds() * 1e-9f / 50 << " seconds/image" << std::endl;


    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            imgA.at<std::uint8_t>(y, x) = outputImg(x, y);
        }

    cv::imwrite("fast.png", imgA);

//    cv::imshow("image", imgA);
//    cv::waitKey(0);



    timer.start();
    for (unsigned i = 0; i < num_iter; i++)
        slow(inputImg, outputImg);
    std::cout << timer.getNanoseconds() * 1e-9f / num_iter << " seconds/image" << std::endl;
/*
    for (unsigned y = 0; y < imgA.rows; y++)
        for (unsigned x = 0; x < imgA.cols; x++) {
            imgA.at<std::uint8_t>(y, x) = outputImg(x, y);
        }

    cv::imwrite("slow.png", imgA);

//    cv::imshow("image", imgA);
//    cv::waitKey(0);
*/
#elif 0
    //auto imgA = cv::imread("/home/pi/frame0126.png", cv::IMREAD_GRAYSCALE);
    auto imgA = cv::imread("/home/andy/Documents/CAD/AppCVRobot/data/fullHDTest_autoIso_640_480_frames/frame0126.png", cv::IMREAD_GRAYSCALE);
    auto fast = cv::FastFeatureDetector::create(21);


    std::vector<cv::KeyPoint> keypointsA;
    CPUStopWatch timer;
    for (unsigned i = 0; i < 1; i++)
        fast->detect(imgA, keypointsA);
    std::cout << timer.getNanoseconds() * 1e-9f / 1 << " seconds/image" << std::endl;

    cv::Mat outImageA;
    cv::drawKeypoints(imgA, keypointsA, outImageA);

    cv::imwrite("opencv_fast.png", outImageA);
#elif 1
    cv::Mat descsA;
    std::vector<cv::KeyPoint> keypointsA;
    cv::Mat descsB;
    std::vector<cv::KeyPoint> keypointsB;

    //auto akaze = cv::AKAZE::create();
    auto orb = cv::ORB::create(5000);

    auto fast = cv::FastFeatureDetector::create(5, true, cv::FastFeatureDetector::TYPE_7_12);
    //auto brief = cv::BriefDescriptorExtractor::create();

    CPUStopWatch timer;
#if 0
    for (unsigned i = 0; i < 10; i++) {
        akaze->detectAndCompute(imgA, cv::noArray(), keypointsA, descsA);
        akaze->detectAndCompute(imgB, cv::noArray(), keypointsB, descsB);
    }
#else
    for (unsigned i = 0; i < 10; i++) {
        fast->detect(imgA, keypointsA);
        orb->compute(imgA, keypointsA, descsA);
        fast->detect(imgB, keypointsB);
        orb->compute(imgB, keypointsB, descsB);
     /*
        akaze->compute(imgA, keypointsA, descsA);
        akaze->compute(imgB, keypointsB, descsB);
    */
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
        #if 0
        if (thresh * second.distance > first.distance)
            filteredMatches.push_back(v.front());
        #else
        if (first.distance < 50)
            filteredMatches.push_back(v.front());
        #endif
    }


    cv::Mat outImage;

    cv::drawMatches (imgA, keypointsA, imgB, keypointsB, filteredMatches, outImage);

    cv::imshow("outImage", outImage);
    cv::waitKey(0);
#endif
    return 0;
}
