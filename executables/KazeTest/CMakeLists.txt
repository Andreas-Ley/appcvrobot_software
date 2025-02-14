cmake_minimum_required (VERSION 3.0)


find_package( OpenCV REQUIRED )
find_package(Eigen3 3.1.0 REQUIRED)

find_package(Boost COMPONENTS system REQUIRED)

option(BUILD_FOR_ARM "Build with all calls to robot functions being empty stubs (for building on desktop PCs)" OFF)
if(BUILD_FOR_ARM)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_FOR_ARM")
endif(BUILD_FOR_ARM)

add_executable(KazeTest
	source/main.cpp
    source/SlowBrief.cpp
    source/SlowBrief.h
    source/Keypoint.cpp
    source/Keypoint.h
    source/Image.cpp
    source/Image.h
    source/FrameKPGrid.cpp
    source/FrameKPGrid.h
    source/Frame.cpp
    source/Frame.h
    source/Fast.cpp
    source/Fast.h
    source/simd.h
    source/simd_neon.h
    source/simd_ref.h
    source/simd_sse.h
    source/Map.cpp
    source/Map.h
    source/RansacE.cpp
    source/RansacE.h
    source/RansacH.cpp
    source/RansacH.h
    source/RansacPnP.cpp
    source/RansacPnP.h
    source/SLAM.cpp
    source/SLAM.h
    source/MVGMath.cpp
    source/MVGMath.h
    source/Track.cpp
    source/Track.h
)

target_include_directories(KazeTest PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(KazeTest
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
