cmake_minimum_required(VERSION 3.8.0)

project(SensorCameraProject VERSION 3.0.0 LANGUAGES C CXX)

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

set(SIMONE_SDK_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
SET(OPENCV_HOME ${SIMONE_SDK_PATH_ROOT}/SensorRaw/3rdparty/opencv)
SET(OPENCV_LIB_PATH ${OPENCV_HOME}/lib)	
include_directories("${SIMONE_SDK_PATH_ROOT}/include")
include_directories("${SIMONE_SDK_PATH_ROOT}/include/ffmpeg")
include_directories("${OPENCV_HOME}/include")

SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH "./")

add_executable(SensorCameraView "")
target_sources(SensorCameraView PRIVATE "src/SensorCamera.cpp")

set_target_properties(SensorCameraView PROPERTIES CXX_STANDARD 14)
set_target_properties(SensorCameraView PROPERTIES
			ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
			LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
			RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
			ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
			LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
			RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
		)

IF (WIN32)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Win64/SimOneStreamingAPI.lib)
	target_link_libraries(SensorCameraView ${OPENCV_LIB_PATH}/opencv_world346.lib)
	
	add_custom_command(TARGET SensorCameraView
        POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avcodec-58.dll ${CMAKE_BINARY_DIR}/bin/avcodec-58.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avformat-58.dll ${CMAKE_BINARY_DIR}/bin/avformat-58.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avutil-56.dll ${CMAKE_BINARY_DIR}/bin/avutil-56.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/swresample-3.dll ${CMAKE_BINARY_DIR}/bin/swresample-3.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/swscale-5.dll ${CMAKE_BINARY_DIR}/bin/swscale-5.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Win64/SimOneStreamingAPI.dll ${CMAKE_BINARY_DIR}/bin/SimOneStreamingAPI.dll
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENCV_LIB_PATH}/opencv_world346.dll ${CMAKE_BINARY_DIR}/bin/opencv_world346.dll
    )
    target_link_libraries(SensorCameraView  ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avcodec.lib)
    target_link_libraries(SensorCameraView  ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avformat.lib)
    target_link_libraries(SensorCameraView  ${SIMONE_SDK_PATH_ROOT}/lib/Win64/avutil.lib)
    target_link_libraries(SensorCameraView  ${SIMONE_SDK_PATH_ROOT}/lib/Win64/swscale.lib)
    target_link_libraries(SensorCameraView  ${SIMONE_SDK_PATH_ROOT}/lib/Win64/swresample.lib)


ELSEIF (UNIX)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.152)
	target_link_libraries(SensorCameraView ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx265.so.146)
    target_link_libraries(SensorCameraView ${OPENCV_LIB_PATH}/libopencv_highgui.so.3.4)
	target_link_libraries(SensorCameraView ${OPENCV_LIB_PATH}/libopencv_core.so.3.4)
	target_link_libraries(SensorCameraView ${OPENCV_LIB_PATH}/libopencv_imgcodecs.so.3.4)
	target_link_libraries(SensorCameraView ${OPENCV_LIB_PATH}/libopencv_imgproc.so.3.4)
	target_link_libraries(SensorCameraView pthread)

	add_custom_command(TARGET SensorCameraView
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneStreamingAPI.so ${CMAKE_BINARY_DIR}/bin/libSimOneStreamingAPI.so
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavcodec.so.58 ${CMAKE_BINARY_DIR}/bin/libavcodec.so.58
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavformat.so.58 ${CMAKE_BINARY_DIR}/bin/libavformat.so.58
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libavutil.so.56 ${CMAKE_BINARY_DIR}/bin/libavutil.so.56
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswresample.so.3 ${CMAKE_BINARY_DIR}/bin/libswresample.so.3
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libswscale.so.5 ${CMAKE_BINARY_DIR}/bin/libswscale.so.5
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx264.so.152 ${CMAKE_BINARY_DIR}/bin/libx264.so.152
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libx265.so.146 ${CMAKE_BINARY_DIR}/bin/libx265.so.146
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENCV_LIB_PATH}/libopencv_highgui.so.3.4 ${CMAKE_BINARY_DIR}/bin/libopencv_highgui.so.3.4
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENCV_LIB_PATH}/libopencv_core.so.3.4 ${CMAKE_BINARY_DIR}/bin/libopencv_core.so.3.4
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENCV_LIB_PATH}/libopencv_imgcodecs.so.3.4 ${CMAKE_BINARY_DIR}/bin/libopencv_imgcodecs.so.3.4
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENCV_LIB_PATH}/libopencv_imgproc.so.3.4 ${CMAKE_BINARY_DIR}/bin/libopencv_imgproc.so.3.4
	)
ENDIF()
