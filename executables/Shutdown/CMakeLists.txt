cmake_minimum_required (VERSION 3.0)
   
find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(Shutdown
    source/main.cpp
)

target_include_directories(Shutdown PUBLIC
)


target_link_libraries(Shutdown
    robot
)

