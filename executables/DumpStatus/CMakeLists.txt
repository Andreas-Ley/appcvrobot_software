cmake_minimum_required (VERSION 3.0)
   
find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(DumpStatus
    source/main.cpp
)

target_include_directories(DumpStatus PUBLIC
)


target_link_libraries(DumpStatus
    robot
    controlSocketProtocol
)

