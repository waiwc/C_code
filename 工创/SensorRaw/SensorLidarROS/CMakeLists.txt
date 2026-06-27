cmake_minimum_required(VERSION 3.8.0)

project(SensorLidarProject VERSION 3.0.0 LANGUAGES C CXX)

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

set(SIMONE_SDK_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
include_directories("${SIMONE_SDK_PATH_ROOT}/include")

SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH "./")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	pcl_conversions
)
# pcl related lib
find_package(PCL REQUIRED COMPONENTS io)

include_directories("${CMAKE_CURRENT_LIST_DIR}")
include_directories("${catkin_INCLUDE_DIRS}")

add_executable(SensorLidarROS "")
target_sources(SensorLidarROS PRIVATE "src/SensorLidarROS.cpp")

set_target_properties(SensorLidarROS PROPERTIES CXX_STANDARD 14)
set_target_properties(SensorLidarROS PROPERTIES
			ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
			LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
			RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
		)


target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so)
target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58)
target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58)
target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56)
target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3)
target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5)
target_link_libraries(SensorLidarROS ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.148)
target_link_libraries(SensorLidarROS pthread)
target_link_libraries(SensorLidarROS ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_custom_command(TARGET SensorLidarROS
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so ${CMAKE_BINARY_DIR}/bin/libSimOneStreamingAPI.so
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58 ${CMAKE_BINARY_DIR}/bin/libavcodec.so.58
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58 ${CMAKE_BINARY_DIR}/bin/libavformat.so.58
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56 ${CMAKE_BINARY_DIR}/bin/libavutil.so.56
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3 ${CMAKE_BINARY_DIR}/bin/libswresample.so.3
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5 ${CMAKE_BINARY_DIR}/bin/libswscale.so.5
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.148 ${CMAKE_BINARY_DIR}/bin/libx264.so.148
)
