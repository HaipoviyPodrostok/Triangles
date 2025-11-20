include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

add_executable(geometry_test.x
    triangle_test.cpp
    plane_test.cpp
    section_test.cpp
)

target_include_directories(geometry_test.x
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(geometry_test.x
    PRIVATE
        triangles_lib
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(geometry_test.x
    PROPERTIES LABELS "unit"
    TEST_PREFIX "unit:"
)