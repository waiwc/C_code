cmake_minimum_required(VERSION 3.8.0)

project(SensorLidarProject VERSION 3.0.0 LANGUAGES C CXX)

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

# set(THIRDPARTY_LIB_PATH /opt/apollo/sysroot/lib)
# set(THIRDPARTY_INCLUDE_PATH /opt/apollo/sysroot/include)
# set(FASTRTPS_LIB_PATH /usr/local/fast-rtps/lib)
# set(FASTRTPS_INCLUDE_PATH /usr/local/fast-rtps/include)
set(APOLLO_PATH_ROOT /apollo)

# include_directories("${THIRDPARTY_INCLUDE_PATH}")
# include_directories("${THIRDPARTY_INCLUDE_PATH}/pcl-1.10")
# include_directories("${THIRDPARTY_INCLUDE_PATH}/boost")

# find_package(PCL REQUIRED)
# add_definitions(${PCL_DEFINITIONS})
# include_directories(${PCL_INCLUDE_DIRS})

# C API of Python for Launch Apollo
# find_package (Python COMPONENTS Interpreter Development REQUIRED)
# include_directories(${Python_INCLUDE_DIRS})

set(CYBER_BRIDGE_ROOT ${APOLLO_PATH_ROOT}/bazel-bin/CyberBridge)
include_directories("${APOLLO_PATH_ROOT}/CyberBridge")

set(SIMONE_SDK_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
include_directories("${SIMONE_SDK_PATH_ROOT}/include")

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/tasks)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH "./")

add_executable(TransNodeCyber "")

file(GLOB_RECURSE tasks ${CMAKE_CURRENT_LIST_DIR}/src/task/*.cpp)
list(FILTER tasks EXCLUDE REGEX "TaskCyberImage.cpp")
list(FILTER tasks EXCLUDE REGEX "TaskCyberPointCloud.cpp")

file(GLOB_RECURSE glm ${CMAKE_CURRENT_LIST_DIR}/src/glm/*.cpp)
file(GLOB_RECURSE com ${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/*.cpp)
target_sources(TransNodeCyber PRIVATE "src/trans_node_cyber.cpp" "src/Config.cpp" ${tasks} ${glm} ${com})

set_target_properties(TransNodeCyber PROPERTIES CXX_STANDARD 14)
set_target_properties(TransNodeCyber PROPERTIES
			ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../run
			LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../run
			RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../run
			ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../run
			LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../run
			RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../run
		)

	target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libHDMapModule.so)
	target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneAPI.so)
	target_link_libraries(TransNodeCyber /usr/lib/x86_64-linux-gnu/libcurl.so.4)
	target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSSD.so)

	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so)
	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58)
	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58)
	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56)
	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3)
	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5)
	# target_link_libraries(TransNodeCyber ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.152)
	target_link_libraries(TransNodeCyber pthread)
	# target_link_libraries(TransNodeCyber ${THIRDPARTY_LIB_PATH}/libboost_thread.so )
	target_link_libraries(TransNodeCyber ${CYBER_BRIDGE_ROOT}/libApolloCyber.so)
	# target_link_libraries(TransNodeCyber ${FASTRTPS_LIB_PATH}/libfastrtps.so)
	# target_link_libraries(TransNodeCyber ${FASTRTPS_LIB_PATH}/libfastcdr.so)

	# target_link_libraries(TransNodeCyber ${Python_LIBRARIES})

	add_custom_command(TARGET TransNodeCyber
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libHDMapModule.so ${CMAKE_BINARY_DIR}/../run/libHDMapModule.so
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneAPI.so ${CMAKE_BINARY_DIR}/../run/libSimOneAPI.so
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSSD.so ${CMAKE_BINARY_DIR}/../run/libSSD.so
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so ${CMAKE_BINARY_DIR}/../run/libSimOneStreamingAPI.so
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58 ${CMAKE_BINARY_DIR}/../run/libavcodec.so.58
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58 ${CMAKE_BINARY_DIR}/../run/libavformat.so.58
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56 ${CMAKE_BINARY_DIR}/../run/libavutil.so.56
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3 ${CMAKE_BINARY_DIR}/../run/libswresample.so.3
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5 ${CMAKE_BINARY_DIR}/../run/libswscale.so.5
		# COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.152 ${CMAKE_BINARY_DIR}/../run/libx264.so.152
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CYBER_BRIDGE_ROOT}/libApolloCyber.so ${CMAKE_BINARY_DIR}/../run/libApolloCyber.so
	)
