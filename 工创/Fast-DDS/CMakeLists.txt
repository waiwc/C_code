
cmake_minimum_required(VERSION 2.8.12)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project("SimOneTrans")

set(fastrtps_FOUND TRUE)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

# Check C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(NOT SUPPORTS_CXX11)
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

include_directories("idl")
include_directories("include")

message(STATUS "Configuring SimOneTrans ...")
file(GLOB SIMONETRANS_SOURCES_CPP "src/*.cpp")
file(GLOB SIMONEDATA_SOURCES_CXX "idl/*.cxx")
file(GLOB COMMON_SOURCES_CPP "src/common/*.cpp")


#file(GLOB XML_CONFIG_FILES "*.xml")
#message(STATUS "XML Files: " ${XML_CONFIG_FILES})
# configure_file("BenchmarkSubscriber.xml" "BenchmarkSubscriber.xml" COPYONLY)
# configure_file("BenchmarkPublisher.xml" "BenchmarkPublisher.xml" COPYONLY)

# SimOneAPI
set(SIMONE_SDK_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
include_directories("${SIMONE_SDK_PATH_ROOT}/include")

add_executable(Trans ${SIMONETRANS_SOURCES_CPP} ${SIMONEDATA_SOURCES_CXX} ${COMMON_SOURCES_CPP})
target_compile_definitions(Trans PRIVATE
    $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
    $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
)
target_link_libraries(Trans fastrtps fastcdr foonathan_memory pthread)

set_target_properties(Trans PROPERTIES CXX_STANDARD 14)
set_target_properties(Trans PROPERTIES
			ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../run
			LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../run
			RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../run
			ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../run
			LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../run
			RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../run
		)

# SimOneAPI
target_link_libraries(Trans /usr/lib/x86_64-linux-gnu/libcurl.so.4)
target_link_libraries(Trans ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libHDMapModule.so)
target_link_libraries(Trans ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSimOneAPI.so)
target_link_libraries(Trans ${SIMONE_SDK_PATH_ROOT}/lib/Linux64/libSSD.so)

# install(TARGETS Benchmark
#     RUNTIME DESTINATION examples/C++/Benchmark/${BIN_INSTALL_DIR})
