add_executable(DynamicsTest "")

include_directories("${INCLUDE_DIR}")

IF (WIN32)
    add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

IF (MSVC)
    string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)
target_sources(DynamicsTest PRIVATE ${srcs})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

SetDefaultTargetProperties(DynamicsTest)

IF (WIN32)
	target_link_libraries(DynamicsTest  ${LIB_DIR}/SimOneAPI.lib)
	add_custom_command(TARGET DynamicsTest
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SIMONEAPI.dll ${BINARY_OUT_PATH}/SimOneAPI.dll
    )
ELSEIF(UNIX)
    target_link_libraries(DynamicsTest  ${LIB_DIR}/libSimOneAPI.so)
ENDIF()


