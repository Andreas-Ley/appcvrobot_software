cmake_minimum_required (VERSION 3.0)
   
find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(ORBSLAM_Test
    source/main.cpp
)

target_include_directories(ORBSLAM_Test PUBLIC
    ${Boost_INCLUDE_DIRS}
)


target_link_libraries(ORBSLAM_Test
    robot
    orb_slam
    ${Boost_LIBRARIES}
)

