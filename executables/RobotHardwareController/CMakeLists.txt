cmake_minimum_required (VERSION 3.0)
   
find_package(Boost COMPONENTS system filesystem REQUIRED)

add_executable(RobotHardwareController
    source/config.cpp
    source/config.h
    source/ControlSocket.cpp
    source/ControlSocket.h
    source/Session.cpp
    source/Session.h
    source/HardwareInterface.cpp
    source/HardwareInterface.h
    source/Logger.cpp
    source/Logger.h
    source/Manager.cpp
    source/Manager.h
    source/main.cpp
)

target_include_directories(RobotHardwareController PUBLIC
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(RobotHardwareController PUBLIC
    ${Boost_LIBRARIES}
    controlSocketProtocol
    pthread
)

if(BUILD_WITH_ROBOT_STUBS)
else(BUILD_WITH_ROBOT_STUBS)
target_link_libraries(RobotHardwareController PUBLIC
    pigpio
)
endif(BUILD_WITH_ROBOT_STUBS)

