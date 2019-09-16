cmake_minimum_required (VERSION 3.0)
   
find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(DebugDashboard
    source/main.cpp
)

target_include_directories(DebugDashboard PUBLIC
    ${Boost_INCLUDE_DIRS}
)


target_link_libraries(DebugDashboard
    robot
    ${Boost_LIBRARIES}
)

