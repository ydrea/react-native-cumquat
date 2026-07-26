cmake_minimum_required(VERSION 3.22)
project(CumquatNativeTests LANGUAGES CXX)

enable_testing()

add_executable(
    cumquat-projection-tests
    cpp/__tests__/ProjectionTest.cpp
    cpp/projection/Projection.cpp
)

target_include_directories(
    cumquat-projection-tests
    PRIVATE
    cpp
)

set_target_properties(
    cumquat-projection-tests
    PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

add_test(
    NAME cumquat-projection-tests
    COMMAND cumquat-projection-tests
)
