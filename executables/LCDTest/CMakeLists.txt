cmake_minimum_required (VERSION 3.0)
   
find_package(Boost COMPONENTS system thread REQUIRED)
find_package( OpenCV REQUIRED )

add_executable(LCDTest
    source/main.cpp
)

target_include_directories(LCDTest PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(LCDTest
    robot
    controlSocketProtocol
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS} 
)

