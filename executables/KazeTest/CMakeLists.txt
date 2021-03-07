cmake_minimum_required (VERSION 3.0)


find_package( OpenCV REQUIRED )

add_executable(KazeTest
	source/main.cpp
)

target_include_directories(KazeTest PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(KazeTest
    ${OpenCV_LIBS}
)

