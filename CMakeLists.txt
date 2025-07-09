cmake_minimum_required(VERSION 3.16)
project(RadarTrackingSystem VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

# Global compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall -Wextra -Wpedantic -Werror
        -Wno-unused-parameter
        -fPIC
    )
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3 -DNDEBUG -march=native)
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-O0 -g -fsanitize=address -fsanitize=undefined)
        add_link_options(-fsanitize=address -fsanitize=undefined)
    endif()
endif()

# Global definitions
add_definitions(-DSPDLOG_COMPILED_LIB)

# Find packages
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

# Find or build third-party dependencies
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found, using third_party version")
    add_subdirectory(third_party/eigen)
endif()

find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    message(STATUS "yaml-cpp not found, using third_party version")
    add_subdirectory(third_party/yaml-cpp)
endif()

find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    message(STATUS "spdlog not found, using third_party version")
    add_subdirectory(third_party/spdlog)
endif()

# Check for DDS
find_package(fastrtps QUIET)
if(fastrtps_FOUND)
    set(DDS_SUPPORT ON)
    add_definitions(-DDDS_SUPPORT)
    message(STATUS "DDS support enabled")
else()
    set(DDS_SUPPORT OFF)
    message(STATUS "DDS support disabled - FastDDS not found")
endif()

# Global include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/interfaces/include
    ${CMAKE_CURRENT_SOURCE_DIR}/protocols
)

# Options
option(BUILD_TESTS "Build test programs" ON)
option(BUILD_BENCHMARKS "Build benchmark programs" ON)
option(ENABLE_PROFILING "Enable profiling support" OFF)
option(ENABLE_REAL_TIME "Enable real-time scheduling" OFF)

if(ENABLE_PROFILING)
    add_definitions(-DENABLE_PROFILING)
endif()

if(ENABLE_REAL_TIME)
    add_definitions(-DENABLE_REAL_TIME)
endif()

# Protocol buffer generation
add_subdirectory(protocols)

# Shared libraries
add_subdirectory(shared)

# Applications
add_subdirectory(apps)

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

install(DIRECTORY config/ DESTINATION config)
install(DIRECTORY scripts/ DESTINATION scripts 
        FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

# CPack configuration
set(CPACK_PACKAGE_NAME "RadarTrackingSystem")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Defense Radar Tracking System")
set(CPACK_PACKAGE_VENDOR "Defense Systems Inc.")
set(CPACK_GENERATOR "TGZ;DEB;RPM")

include(CPack)

# Custom targets
add_custom_target(format
    COMMAND find ${CMAKE_SOURCE_DIR} -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
    COMMENT "Formatting source code"
)

add_custom_target(docs
    COMMAND doxygen ${CMAKE_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation"
)