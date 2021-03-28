cmake_minimum_required(VERSION 3.5)
project(test1)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${ROOT_SOURCE_DIR}
					${PCL_INCLUDE_DIRS}
)

link_libraries(	${PCL_LIBRARY_DIRS} 
				${OpenCV_LIBS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} 
	main.cpp
	ExtriCal.cpp
	Param.cpp
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} 
${OpenCV_LIBS})
