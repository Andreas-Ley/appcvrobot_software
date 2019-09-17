cmake_minimum_required (VERSION 3.0)

find_package( OpenCV REQUIRED )


add_library(DBoW2 STATIC
    DBoW2/BowVector.h
    DBoW2/FORB.h 
    DBoW2/FClass.h       
    DBoW2/FeatureVector.h
    DBoW2/ScoringObject.h   
    DBoW2/TemplatedVocabulary.h
    DBoW2/BowVector.cpp
    DBoW2/FORB.cpp      
    DBoW2/FeatureVector.cpp
    DBoW2/ScoringObject.cpp
    DUtils/Random.h
    DUtils/Timestamp.h
    DUtils/Random.cpp
    DUtils/Timestamp.cpp
)


target_include_directories(DBoW2 PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(DBoW2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2
    ${CMAKE_CURRENT_SOURCE_DIR}/DUtils
)

target_include_directories(DBoW2 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(DBoW2 PUBLIC
    ${OpenCV_LIBS} 
)

