cmake_minimum_required (VERSION 3.0)

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED) 


add_library(robot STATIC
)

set(data_files 
)

foreach(file ${data_files})
    configure_file(${file} ${PROJECT_BINARY_DIR}/${file} COPYONLY)
endforeach(file)

if (${ImageMagick_LIBRARIES} MATCHES "Q16")
target_compile_definitions(convnet PUBLIC "MAGICKCORE_QUANTUM_DEPTH=16")
else()
target_compile_definitions(convnet PUBLIC "MAGICKCORE_QUANTUM_DEPTH=8")
endif()

if (${ImageMagick_LIBRARIES} MATCHES "HDRI")
target_compile_definitions(convnet PUBLIC "MAGICKCORE_HDRI_ENABLE=true")
else()
target_compile_definitions(convnet PUBLIC "MAGICKCORE_HDRI_ENABLE=false")
endif()


target_include_directories(convnet PUBLIC
    ${ImageMagick_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

target_include_directories(convnet INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/source
)

target_link_libraries(convnet PUBLIC
    ${ImageMagick_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
	pthread
)


target_link_libraries(convnet PUBLIC
)



