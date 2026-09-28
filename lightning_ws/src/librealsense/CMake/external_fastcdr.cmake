# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#
# Fetch and build FastCDR library for CDR serialization support
# This is used for ROS2 bag file read/write without requiring full DDS support
# Uses configure-time git clone + add_subdirectory (compatible with CMake 3.10+)

set(FASTCDR_SOURCE_DIR ${CMAKE_BINARY_DIR}/third-party/fastcdr)
# [G1 vendored] 随包源码，存在时复制到原位置，下面的 git clone 会因 CMakeLists.txt 已存在而跳过
set(LRS_VENDORED_FASTCDR_DIR "${CMAKE_CURRENT_LIST_DIR}/../third-party/vendored/fastcdr")
if(EXISTS "${LRS_VENDORED_FASTCDR_DIR}/CMakeLists.txt" AND NOT EXISTS "${FASTCDR_SOURCE_DIR}/CMakeLists.txt")
    message(STATUS "Using vendored fastcdr: ${LRS_VENDORED_FASTCDR_DIR}")
    file(COPY "${LRS_VENDORED_FASTCDR_DIR}/" DESTINATION "${FASTCDR_SOURCE_DIR}")
endif()

function(get_fastcdr_only)
    message(STATUS "Fetching fastcdr...")

    if(NOT EXISTS ${FASTCDR_SOURCE_DIR}/CMakeLists.txt)
        find_package(Git REQUIRED)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} clone --depth 1 --branch v1.0.25
                https://github.com/eProsima/Fast-CDR.git
                ${FASTCDR_SOURCE_DIR}
            RESULT_VARIABLE GIT_RESULT
        )
        if(NOT GIT_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to clone FastCDR repository")
        endif()
    endif()

    # Set special values for fastcdr build
    set(BUILD_SHARED_LIBS OFF)

    # FastCDR v1.0.25 has cmake_minimum_required(VERSION 2.x) which CMake 4.0+ rejects.
    # Override the minimum policy version so the old cmake_minimum_required is accepted.
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

    add_subdirectory(${FASTCDR_SOURCE_DIR} ${CMAKE_BINARY_DIR}/third-party/fastcdr-build EXCLUDE_FROM_ALL)

    # Place fastcdr with other 3rd-party projects
    set_target_properties(fastcdr PROPERTIES FOLDER "3rd Party")

    # Make sure fastcdr include directories are available
    target_include_directories(fastcdr INTERFACE
        $<BUILD_INTERFACE:${FASTCDR_SOURCE_DIR}/include>
    )

    message(STATUS "Fetching fastcdr... done")
endfunction()

# Only fetch fastcdr if not already available (i.e., BUILD_WITH_DDS is OFF)
if(NOT TARGET fastcdr)
    get_fastcdr_only()
endif()
