cmake_minimum_required(VERSION 3.4.1)
project(Cumquat)

set (CMAKE_VERBOSE_MAKEFILE ON)

add_library(
    react-native-cumquat    STATIC
    ../cpp/CumquatImpl.cpp
)

set_target_properties(
    react-native-cumquat PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(
    react-native-cumquat    PUBLIC
    ../cpp
)

target_link_libraries(
    react-native-cumquat    jsi
    reactnative
    react_codegen_CumquatSpec
)
