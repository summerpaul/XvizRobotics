cmake_minimum_required(VERSION 3.10.0)
project(xviz_robotics)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


IF (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    set(VCPKG_ROOT "D:/vcpkg" CACHE PATH "Path to vcpkg root directory")
    set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "vcpkg target triplet")
elseif(APPLE)
    set(VCPKG_ROOT "~/vcpkg" CACHE PATH "Path to vcpkg root directory")
    set(VCPKG_TARGET_TRIPLET "arm64-osx" CACHE STRING "vcpkg target triplet")
elseif(UNIX)
endif()

include(${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)


find_package(protobuf CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)





include_directories(xvizMsgBridge/include)
if(APPLE)
    link_directories(xvizMsgBridge/lib/arm64-osx)
    include_directories(${CMAKE_BINARY_DIR}/vcpkg_installed/arm64-osx/include)
    
elseif(WIN32)
    link_directories(xvizMsgBridge/lib/x64-windows)
    include_directories(${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include)
endif()

include_directories(app)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(demo)
add_subdirectory(app)