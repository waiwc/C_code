
# set(CMAKE_C_COMPILER "/usr/local/gcc")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++")

add_executable(PNCSample "")

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp *.py)
file(GLOB_RECURSE headers *.h)

target_sources(PNCSample PRIVATE "src/PNC_Sample.cpp" "src/dumper.cpp" ${headers})

source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

# include_directories("${SIMONE_API}/include/")
include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")
include_directories("${INCLUDE_DIR}")
#include_directories("${3RDPARTY}")

set (LOCAL_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib")

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()
	add_definitions(-DNOMINMAX)
	add_definitions(-DUNICODE)
	add_definitions(-D_UNICODE)

IF (MSVC)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

SetDefaultTargetProperties(PNCSample)

IF (WIN32)
        target_link_libraries(PNCSample  ${LIB_DIR}/SimOneAPI.lib ${LIB_DIR}/SSD.lib ${LIB_DIR}/HDMapModule.lib  ${LIB_DIR}/ProtobufModule.lib ${LIB_DIR}/ProtobufModule.lib)
       
        add_custom_command(TARGET PNCSample
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SimOneAPI.dll ${BINARY_OUT_PATH}/SimOneAPI.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SSD.dll ${BINARY_OUT_PATH}/SSD.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/HDMapModule.dll ${BINARY_OUT_PATH}/HDMapModule.dll
        )    

ELSEIF (UNIX)
        target_link_libraries(PNCSample debug ${LIB_DIR}/libSimOneAPI.so)
        target_link_libraries(PNCSample optimized ${LIB_DIR}/libSimOneAPI.so -lpthread)
        target_link_libraries(PNCSample debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
        target_link_libraries(PNCSample optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)
        target_link_libraries(PNCSample debug ${LIB_DIR}/libSSD.so)
        target_link_libraries(PNCSample optimized ${LIB_DIR}/libSSD.so -lpthread)
        target_link_libraries(PNCSample debug ${LIB_DIR}/libHDMapModule.so)
        target_link_libraries(PNCSample optimized ${LIB_DIR}/libHDMapModule.so)

        add_custom_command(TARGET PNCSample
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneAPI.so ${BINARY_OUT_PATH}/libSimOneAPI.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BINARY_OUT_PATH}/libSSD.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libHDMapModule.so ${BINARY_OUT_PATH}/libHDMapModule.so
        )
ENDIF()
