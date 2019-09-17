cmake_minimum_required (VERSION 3.0)


find_package( OpenCV REQUIRED )
find_package(Eigen3 3.1.0 REQUIRED)



add_library(orb_slam STATIC
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    src/Viewer.cc
)


set(data_files 
    Vocabulary/ORBvoc.txt
)

foreach(file ${data_files})
    configure_file(${file} ${PROJECT_BINARY_DIR}/${file} COPYONLY)
endforeach(file)




target_include_directories(orb_slam PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(orb_slam PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(orb_slam PUBLIC
    ${OpenCV_LIBS} 
    ${EIGEN3_LIBRARIES}
    DBoW2
    g2o
)

