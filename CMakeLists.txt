cmake_minimum_required(VERSION 3.14)
project("triangles")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(TRIANGLES_STANDALONE ON)
else()
    set(TRIANGLES_STANDALONE OFF)
endif()

option(USE_OPENCL "Enable OpenCL GPU acceleration" ON)
option(ENABLE_LOGS "Enable spdlog logging" ON)

if(USE_OPENCL)
    find_package(OpenCL REQUIRED)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRCS
    source/geometry/vector_3d.cpp
    source/geometry/line.cpp
    source/geometry/section.cpp
    source/geometry/plane.cpp
    source/geometry/triangle.cpp
    source/acceleration/AABB.cpp
    source/acceleration/bvh_tree.cpp
)

add_library(triangles_lib STATIC ${SRCS})
target_include_directories(triangles_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(USE_OPENCL)
    target_link_libraries(triangles_lib PUBLIC OpenCL::OpenCL)
    target_compile_definitions(triangles_lib PUBLIC USE_OPENCL)
endif()

if(ENABLE_LOGS)
    find_package(spdlog REQUIRED)
    target_link_libraries(triangles_lib PRIVATE spdlog::spdlog)
    target_compile_definitions(triangles_lib PRIVATE ENABLE_LOGGING)
endif()

if(TRIANGLES_STANDALONE)
    enable_testing()

    add_executable(triangles.x main.cpp)
    target_link_libraries(triangles.x PRIVATE triangles_lib)

    include(CTest)
    if(BUILD_TESTING)
        add_subdirectory(tests/unit)

        find_package(Python3 COMPONENTS Interpreter REQUIRED)
        add_test(NAME end2end
                 COMMAND ${Python3_EXECUTABLE}
                         ${CMAKE_SOURCE_DIR}/tests/end2end/run.py
                         -b $<TARGET_FILE:triangles.x>
        )
        set_tests_properties(end2end PROPERTIES LABELS "e2e"
                             TEST_PREFIX "e2e:"
        )
    endif()
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