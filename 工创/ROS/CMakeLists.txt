
project(ProjectName)
cmake_minimum_required(VERSION 3.8.0)

set(project_name trans_ros)
project(${project_name})


add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

add_executable(trans_node_ros "")
# add_library(trans_node_ros SHARED "")

target_sources(trans_node_ros PRIVATE "src/trans_node_ros.cpp" "src/Config.cpp")

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)

source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

set(LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../lib/Linux64)
set(SAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")

include_directories("${SAMPLES_DIR}/include")
include_directories("${INCLUDE_DIR}")
include_directories("${INCLUDE_DIR}/3rdparty")
include_directories("${INCLUDE_DIR}/msg_gen")

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

IF (MSVC)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

function(SetDefaultTargetProperties target)
    set_target_properties(${target} PROPERTIES CXX_STANDARD 14)
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../run)
endfunction()

SetDefaultTargetProperties(trans_node_ros)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		tf
		sensor_msgs
		pcl_msgs
		pcl_conversions
		${extra_ros_packages}
	    )

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(trans_node_ros debug ${LIB_DIR}/libSimOneAPI.so)
target_link_libraries(trans_node_ros optimized ${LIB_DIR}/libSimOneAPI.so -lpthread)
target_link_libraries(trans_node_ros debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
target_link_libraries(trans_node_ros optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)
target_link_libraries(trans_node_ros ${catkin_LIBRARIES})
target_link_libraries(trans_node_ros debug ${LIB_DIR}/libSimOneStreamingAPI.so)
target_link_libraries(trans_node_ros optimized ${LIB_DIR}/libSimOneStreamingAPI.so)
target_link_libraries(trans_node_ros debug ${LIB_DIR}/libSSD.so)
target_link_libraries(trans_node_ros optimized ${LIB_DIR}/libSSD.so -lpthread)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
	file(GLOB APILibFiles "${CMAKE_CURRENT_LIST_DIR}/../../Build/build_debug/lib/*")
        file(GLOB SSDLibFiles "${3RDPARTY_DIR}/SSD/lib/linux/debug/*")

elseif(CMAKE_BUILD_TYPE MATCHES "Release")
	file(GLOB APILibFiles "${CMAKE_CURRENT_LIST_DIR}/../../Build/build_release/lib/*")
        file(GLOB SSDLibFiles "${3RDPARTY_DIR}/SSD/lib/linux/release/*")
endif()

file(GLOB StreamingAPILibFiles "${3RDPARTY_DIR}/streaming/lib/*.so")
file(GLOB ffmpegLibFiles "${3RDPARTY_DIR}/artifactory/ffmpeg/lib/*")

file(COPY ${APILibFiles} DESTINATION ${LIB_DIR})
file(COPY ${SSDLibFiles} DESTINATION ${LIB_DIR})
file(COPY ${StreamingAPILibFiles} DESTINATION ${LIB_DIR})
file(COPY ${ffmpegLibFiles} DESTINATION ${LIB_DIR})

# add_custom_command(TARGET trans_node_ros
#     POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneIOAPI.so ${BRIDGEIOAPI_BUILD}/libSimOneIOAPI.so
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BRIDGEIOAPI_BUILD}/libSSD.so
#     COMMAND
# )
