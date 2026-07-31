cmake_minimum_required(VERSION 3.22)
project(Cumquat LANGUAGES CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(
    react-native-cumquat
    STATIC
    ../cpp/CumquatImpl.cpp
    ../cpp/android/GameRotationSensor.cpp
    ../cpp/android/RotationAlignment.cpp
    ../cpp/bridge/NativeCumquatModule.cpp
    ../cpp/core/Engine.cpp
    ../cpp/geo/Geodesy.cpp
    ../cpp/projection/Projection.cpp
)

set_target_properties(
    react-native-cumquat
    PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(
    react-native-cumquat
    PUBLIC
    ../cpp
    ../cpp/bridge
)

target_link_libraries(
    react-native-cumquat
    jsi
    reactnative
    react_codegen_CumquatSpec
    android
)
