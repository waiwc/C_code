cmake_minimum_required(VERSION 3.8.0)

project(SensorLidarProject VERSION 3.0.0 LANGUAGES C CXX)

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

set(SIMONE_SDK_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
SET(PCL_HOME ${SIMONE_SDK_PATH_ROOT}/SensorRaw/3rdparty/pcl)
include_directories("${SIMONE_SDK_PATH_ROOT}/include")
include_directories("${PCL_HOME}/include")
include_directories("${PCL_HOME}/include/pcl")

SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH "./")

add_executable(SensorLidarView "")
target_sources(SensorLidarView PRIVATE "src/SensorLidar.cpp")

set_target_properties(SensorLidarView PROPERTIES CXX_STANDARD 14)
set_target_properties(SensorLidarView PROPERTIES
			ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
			LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
			RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
			ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
			LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
			RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
		)

IF (WIN32)
    include_directories("${PCL_HOME}/include/vtk-8.0")
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Win64/SimOneStreamingAPI.lib)
	SET(PCL_LIB_PATH ${PCL_HOME}/lib/Win64)
	FILE(GLOB_RECURSE ALL_PCL_LIBS  ${PCL_LIB_PATH}/*.lib)
	target_link_libraries(SensorLidarView ${ALL_PCL_LIBS})
	
	add_custom_command(TARGET SensorLidarView
        POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avcodec-58.dll ${CMAKE_BINARY_DIR}/bin/avcodec-58.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avformat-58.dll ${CMAKE_BINARY_DIR}/bin/avformat-58.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avutil-56.dll ${CMAKE_BINARY_DIR}/bin/avutil-56.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/swresample-3.dll ${CMAKE_BINARY_DIR}/bin/swresample-3.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/swscale-5.dll ${CMAKE_BINARY_DIR}/bin/swscale-5.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/SimOneStreamingAPI.dll ${CMAKE_BINARY_DIR}/bin/SimOneStreamingAPI.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PCL_HOME}/bin/OpenNI2.dll ${CMAKE_BINARY_DIR}/bin/OpenNI2.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PCL_HOME}/bin/pcl_common_release.dll ${CMAKE_BINARY_DIR}/bin/pcl_common_release.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PCL_HOME}/bin/pcl_io_ply_release.dll ${CMAKE_BINARY_DIR}/bin/pcl_io_ply_release.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PCL_HOME}/bin/pcl_io_release.dll ${CMAKE_BINARY_DIR}/bin/pcl_io_release.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PCL_HOME}/bin/pcl_kdtree_release.dll ${CMAKE_BINARY_DIR}/bin/pcl_kdtree_release.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PCL_HOME}/bin/pcl_visualization_release.dll ${CMAKE_BINARY_DIR}/bin/pcl_visualization_release.dll
    )

ELSEIF (UNIX)
    include_directories("${PCL_HOME}/include/vtk-6.2")
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so)
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58)
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58)
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56)
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3)
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5)
	target_link_libraries(SensorLidarView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.148)
	target_link_libraries(SensorLidarView pthread)
	SET(PCL_LIB_PATH ${PCL_HOME}/lib/Linux64)
	FILE(GLOB_RECURSE ALL_PCL_LIBS  ${PCL_LIB_PATH}/*.*)
	target_link_libraries(SensorLidarView ${ALL_PCL_LIBS})

	add_custom_command(TARGET SensorLidarView
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so ${CMAKE_BINARY_DIR}/bin/libSimOneStreamingAPI.so
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58 ${CMAKE_BINARY_DIR}/bin/libavcodec.so.58
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58 ${CMAKE_BINARY_DIR}/bin/libavformat.so.58
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56 ${CMAKE_BINARY_DIR}/bin/libavutil.so.56
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3 ${CMAKE_BINARY_DIR}/bin/libswresample.so.3
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5 ${CMAKE_BINARY_DIR}/bin/libswscale.so.5
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.148 ${CMAKE_BINARY_DIR}/bin/libx264.so.148
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ALL_PCL_LIBS} ${CMAKE_BINARY_DIR}/bin/
	)
ENDIF()
