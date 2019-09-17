cmake_minimum_required (VERSION 3.0)
project (Everything)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

LIST(APPEND CMAKE_CXX_FLAGS "-march=native -ffast-math -Wall -pedantic -Wextra -std=c++17 -Wno-int-in-bool-context -Wno-class-memaccess -Wno-unused-parameter -Wno-deprecated-copy")


set(COMPILATION_HOSTNAME "andynotebook" CACHE STRING "To differentiate in code between computers for e.g. different paths")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCOMPILATION_HOSTNAME_${COMPILATION_HOSTNAME}")


option(BUILD_WITH_ROBOT_STUBS "Build with all calls to robot functions being empty stubs (for building on desktop PCs)" OFF)
if(BUILD_WITH_ROBOT_STUBS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_WITH_ROBOT_STUBS")
endif(BUILD_WITH_ROBOT_STUBS)



option(BUILD_WITH_ORB_SLAM "Build with ORB SLAM (requires OpenCV)" OFF)
if(BUILD_WITH_ORB_SLAM)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_WITH_ORB_SLAM")
endif(BUILD_WITH_ORB_SLAM)


set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -O3 -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Og -g")

if(BUILD_WITH_ORB_SLAM)
add_subdirectory(extern/g2o/)
add_subdirectory(extern/DBoW2/)
add_subdirectory(extern/ORB_SLAM/)
endif(BUILD_WITH_ORB_SLAM)

add_subdirectory(libs/robot/)
add_subdirectory(executables/DebugDashboard/)
add_subdirectory(executables/ORBSLAM_Test/)
