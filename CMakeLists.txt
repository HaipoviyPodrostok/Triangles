enable_testing()

cmake_minimum_required(VERSION 3.11)
project("triangles")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

SET(SRCS 
    main.cpp
    source/triangles.cpp
)

add_library(triangles_lib STATIC ${SRCS})
target_include_directories(triangles_lib PUBLIC "include")

add_executable(triangles.x main.cpp)
target_link_libraries(triangles.x triangles_lib)

file(GLOB testfiles "${CMAKE_SOURCE_DIR}/tests/*.in")

foreach(file ${testfiles})
    get_filename_component(testname "${file}" NAME_WE)

    add_test(
        NAME ${testname}
        COMMAND bash -c "${CMAKE_SOURCE_DIR}/runtest.sh ${file} ${CMAKE_CURRENT_BINARY_DIR}/triangles.x"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )

    set_tests_properties(${testname} PROPERTIES DEPENDS triangles.x)
endforeach()