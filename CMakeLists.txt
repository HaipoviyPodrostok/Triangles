cmake_minimum_required(VERSION 3.11)
project("triangles")

enable_testing()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

SET(SRCS
    source/geometry/vector_3d.cpp
    source/geometry/line.cpp
    source/geometry/section.cpp
    source/geometry/plane.cpp
    source/geometry/triangle.cpp
    source/acceleration/AABB.cpp
    source/acceleration/bvh_tree.cpp
)

add_library(triangles_lib STATIC ${SRCS})
target_include_directories(triangles_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(triangles.x main.cpp)
target_link_libraries(triangles.x PRIVATE triangles_lib)

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()

#file(GLOB testfiles "${CMAKE_SOURCE_DIR}/tests/*.in")

#foreach(file ${testfiles})
#    get_filename_component(testname "${file}" NAME_WE)
#
#    add_test(
#        NAME ${testname}
#        COMMAND bash -c "${CMAKE_SOURCE_DIR}/runtest.sh ${file} ${CMAKE_CURRENT_BINARY_DIR}/triangles.x"
#        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#    )
#
#    set_tests_properties(${testname} PROPERTIES DEPENDS triangles.x)
#endforeach() 