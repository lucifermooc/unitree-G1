# Fetch yaml-cpp headers for rosbag2 metadata parsing

# [G1 vendored] 随包源码（yaml-cpp-0.7.0），存在时不联网下载
set(LRS_VENDORED_YAML_CPP_DIR "${CMAKE_CURRENT_LIST_DIR}/../third-party/vendored/yaml-cpp")
if(NOT TARGET yaml_cpp AND EXISTS "${LRS_VENDORED_YAML_CPP_DIR}/include")
    include(ExternalProject)
    ExternalProject_Add(yaml_cpp
        SOURCE_DIR "${LRS_VENDORED_YAML_CPP_DIR}"
        DOWNLOAD_COMMAND ""
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
    )
endif()
if(NOT TARGET yaml_cpp)
    include(ExternalProject)
    ExternalProject_Add(yaml_cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-0.7.0
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
    )
endif()

ExternalProject_Get_Property(yaml_cpp SOURCE_DIR)
set(yaml_cpp_SOURCE_DIR ${SOURCE_DIR})

set(HEADER_DIR_YAML_CPP
    ${yaml_cpp_SOURCE_DIR}/include
)
