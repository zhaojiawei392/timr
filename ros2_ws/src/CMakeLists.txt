 ## 
 ##    This file is part of timr.
 ## 
 ##    timr is free software: you can redistribute it and/or modify 
 ##    it under the terms of the GNU General Public License as published 
 ##    by the Free Software Foundation, either version 3 of the License, 
 ##    or (at your option) any later version.
 ## 
 ##    timr is distributed in the hope that it will be useful, 
 ##    but WITHOUT ANY WARRANTY; without even the implied warranty of 
 ##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 ##    See the GNU General Public License for more details.
 ## 
 ##    You should have received a copy of the GNU General Public License
 ##    along with timr. If not, see <https://www.gnu.org/licenses/>.
 ##

 ##
 ##    Filename CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2024-2025
 ##
 ##    This project includes [qpOASES](https://github.com/coin-or/qpOASES.git), 
 ##    licensed under the GNU LESSER GENERAL PUBLIC LICENSE Version 2.1, February 1999
 ##
 ##    This project includes [nlohmann/json](https://github.com/nlohmann/json), 
 ##    licensed under the MIT License 

cmake_minimum_required(VERSION 3.22)
project(timr VERSION 1.0.0 LANGUAGES CXX)

########################################################################################################################################
########################################################################################################################################
##########################################             CONFIGURE  BUILD          #######################################################
########################################################################################################################################
########################################################################################################################################
# Macro options
option(timr_BUILD_EXAMPLES "Build examples." ON)
option(timr_BUILD_ARM "Enable cross-compilation for ARM" OFF)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Do not build examples for qpoases.")
set(JSON_Install ON CACHE BOOL "Install JSON library.")
set(CMAKE_SYSROOT ${CMAKE_SYSROOT} CACHE PATH "Sysroot for ARM cross-compilation")

message(STATUS "timr_BUILD_EXAMPLES  ${timr_BUILD_EXAMPLES}")
message(STATUS "timr_BUILD_ARM  ${timr_BUILD_ARM}")
message(STATUS "QPOASES_BUILD_EXAMPLES  ${QPOASES_BUILD_EXAMPLES}")
message(STATUS "JSON_Install  ${JSON_Install}")
message(STATUS "CMAKE_SYSROOT  ${CMAKE_SYSROOT}")

if (timr_BUILD_ARM)
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR aarch64)

    # Check for required tools
    find_program(ARM_CC aarch64-linux-gnu-gcc REQUIRED)
    find_program(ARM_CXX aarch64-linux-gnu-g++ REQUIRED)
    message(STATUS "Cross-compiler found: ${ARM_CC}, ${ARM_CXX}")
    if (NOT ARM_CC OR NOT ARM_CXX)
        message(FATAL_ERROR "Cross-compilation tools not found. Install aarch64-linux-gnu-gcc and aarch64-linux-gnu-g++.")
    endif()
    # Specify the cross-compiler
    set(CMAKE_C_COMPILER ${ARM_CC})
    set(CMAKE_CXX_COMPILER ${ARM_CXX})
    set(CMAKE_LINKER aarch64-linux-gnu-ld)
    set(CMAKE_AR aarch64-linux-gnu-ar)
    set(CMAKE_RANLIB aarch64-linux-gnu-ranlib)

    # Append compilation flags
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a" CACHE STRING "C compiler flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a" CACHE STRING "C++ compiler flags")
else()
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED True)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_DEBUG_POSTFIX d)
endif()
# add compiler warning flags just when building this project via
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
add_compile_options(
    "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-pedantic>>"
    "$<${msvc_cxx}:$<BUILD_INTERFACE:/W4>>"
)

add_subdirectory(external/qpOASES)
add_subdirectory(external/json)
find_package(Boost COMPONENTS system REQUIRED)
if (NOT Boost_FOUND)
    message(FATAL_ERROR "Boost not found. Install Boost and ensure the system component is available.")
endif()
set(remoteApiCoppeliasim_LIB ${PROJECT_SOURCE_DIR}/external/coppeliasim/share/libremoteApiCoppeliasim.so)
if(NOT EXISTS ${remoteApiCoppeliasim_LIB})
    message(FATAL_ERROR "libremoteApiCoppeliasim.so not found in ${PROJECT_SOURCE_DIR}/external/coppeliasim/share")
endif()
set(CMAKE_BUILD_RPATH ${PROJECT_SOURCE_DIR}/external/coppeliasim/share)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/external")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

########################################################################################################################################
########################################################################################################################################
##########################################            BUILD AND INSTALL          #######################################################
########################################################################################################################################
########################################################################################################################################
add_library(timr INTERFACE)
target_include_directories(timr INTERFACE 
    $<BUILD_INTERFACE:
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/timr
        ${PROJECT_SOURCE_DIR}/external/qpOASES/include
        ${PROJECT_SOURCE_DIR}/external/json/include
        ${PROJECT_SOURCE_DIR}/external/coppeliasim/include>
    $<INSTALL_INTERFACE:include> 
)

target_link_libraries(timr INTERFACE
    qpOASES
    Boost::system
    ${remoteApiCoppeliasim_LIB}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp"
)

# Create and install the package configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/timrConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(TARGETS timr
        EXPORT timrTargets
        INCLUDES DESTINATION include
)
export(EXPORT timrTargets 
        FILE ${CMAKE_CURRENT_BINARY_DIR}/timrTargets.cmake
)
install(EXPORT timrTargets
        FILE timrTargets.cmake
        DESTINATION lib/cmake/timr
)
configure_file(timrConfig.cmake.in timrConfig.cmake @ONLY)
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/timrConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/timrConfigVersion.cmake"
        DESTINATION lib/cmake/timr
)
install(FILES ${PROJECT_SOURCE_DIR}/external/coppeliasim/share/libremoteApiCoppeliasim.so
        DESTINATION lib
)
    
########################################################################################################################################
########################################################################################################################################
##########################################             BUILD  EXECUTABLES          #####################################################
########################################################################################################################################
########################################################################################################################################
# Build examples
if(timr_BUILD_EXAMPLES)
    set(EXAMPLE_NAMES
        timr_example1
    )
    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
        target_link_libraries(${EXAMPLE} timr)
    endforeach()
endif()

add_executable(circling src/circling.cpp)
target_link_libraries(circling PRIVATE
    timr
)

set(SRC_NAMES
    homing
    teleop
)
foreach(SRC ${SRC_NAMES})
    add_executable(${SRC} src/${SRC}.cpp)
    target_link_libraries(${SRC} PRIVATE
        timr
    )
endforeach()
