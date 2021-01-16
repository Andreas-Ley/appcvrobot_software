cmake_minimum_required (VERSION 3.0)


add_library(controlSocketProtocol STATIC
    source/controlSocketProtocol/ControlSocketProtocol.cpp
    source/controlSocketProtocol/ControlSocketProtocol.h
    source/controlSocketProtocol/RobotHardware.cpp
    source/controlSocketProtocol/RobotHardware.h
)

target_include_directories(controlSocketProtocol INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/source
)
