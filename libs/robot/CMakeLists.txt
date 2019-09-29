cmake_minimum_required (VERSION 3.0)

find_package( OpenCV REQUIRED )
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED) 


if(BUILD_WITH_ORB_SLAM)
set(SLAM_files 
    source/robot/SLAMSystem.cpp
    source/robot/SLAMSystem.h
)
else(BUILD_WITH_ORB_SLAM)
set(SLAM_files 
)
endif(BUILD_WITH_ORB_SLAM)



add_library(robot STATIC
    source/robot/Robot.cpp
    source/robot/Robot.h
    source/robot/Subsystem.cpp
    source/robot/Subsystem.h
    source/robot/DrivePolicy.cpp
    source/robot/DrivePolicy.h
    source/robot/DrivePolicyAccLimited.cpp
    source/robot/DrivePolicyAccLimited.h
    source/robot/DrivePolicyDirect.cpp
    source/robot/DrivePolicyDirect.h
    source/robot/RemoteControlPolicy.cpp
    source/robot/RemoteControlPolicy.h
    source/robot/SystemMonitoring.cpp
    source/robot/SystemMonitoring.h
    source/robot/CameraSystem.cpp
    source/robot/CameraSystem.h
    source/robot/WifiCommunication.cpp
    source/robot/WifiCommunication.h
    source/robot/HardwareInterface.cpp
    source/robot/HardwareInterface.h
    ${SLAM_files}
)

set(data_files 
)

foreach(file ${data_files})
    configure_file(${file} ${PROJECT_BINARY_DIR}/${file} COPYONLY)
endforeach(file)

if (${ImageMagick_LIBRARIES} MATCHES "Q16")
target_compile_definitions(robot PUBLIC "MAGICKCORE_QUANTUM_DEPTH=16")
else()
target_compile_definitions(robot PUBLIC "MAGICKCORE_QUANTUM_DEPTH=8")
endif()

if (${ImageMagick_LIBRARIES} MATCHES "HDRI")
target_compile_definitions(robot PUBLIC "MAGICKCORE_HDRI_ENABLE=true")
else()
target_compile_definitions(robot PUBLIC "MAGICKCORE_HDRI_ENABLE=false")
endif()


target_include_directories(robot PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${ImageMagick_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

target_include_directories(robot INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/source
)

target_link_libraries(robot PUBLIC
    ${OpenCV_LIBS} 
    ${ImageMagick_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
    pthread
)

if(BUILD_WITH_ORB_SLAM)
target_link_libraries(robot PUBLIC
    orb_slam
)
endif(BUILD_WITH_ORB_SLAM)

if(BUILD_WITH_ROBOT_STUBS)
else(BUILD_WITH_ROBOT_STUBS)
target_link_libraries(robot PUBLIC
    pigpio
)
endif(BUILD_WITH_ROBOT_STUBS)


