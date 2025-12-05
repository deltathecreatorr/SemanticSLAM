cmake_minimum_required(VERSION 3.30)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "vcpkg toolchain file")

project(SemanticSLAM VERSION 0.1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#UPDATE THIS LINE WHEN ADDING DEPENDENCIES
find_package(fmt CONFIG REQUIRED)
#find_package(libtorch CONFIG REQUIRED)

set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

file(GLOB_RECURSE slam_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_SOURCE_DIR}/src/*.inl"
)

add_executable(slam ${slam_SOURCES})

target_include_directories(slam PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(slam PRIVATE
    MY_TEST_DEFINE
)

target_link_libraries(slam PRIVATE
    fmt::fmt
    #libtorch::libtorch
)

if (MSVC)
    target_compile_options(slam PRIVATE /W4 /WX)
else()
    target_compile_options(slam PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT slam)

source_group(TREE "${CMAKE_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${slam_SOURCES})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)