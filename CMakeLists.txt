cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(${ROOT_SOURCE_DIR})

link_libraries(${OpenCV_LIBS})

add_executable(${PROJECT_NAME} 
	main.cpp
	ExtriCal.cpp
	Param.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
