cmake_minimum_required (VERSION 3.0)


find_package( OpenCV REQUIRED )

option(BUILD_FOR_ARM "Build with all calls to robot functions being empty stubs (for building on desktop PCs)" OFF)
if(BUILD_FOR_ARM)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_FOR_ARM")
endif(BUILD_FOR_ARM)

add_executable(KazeTest
	source/main.cpp
)

target_include_directories(KazeTest PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(KazeTest
    ${OpenCV_LIBS}
)

