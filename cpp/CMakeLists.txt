cmake_minimum_required(VERSION 3.16)

# Also usable as the configure root: cmake -S cpp -B build
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
        message(FATAL_ERROR
            "In-source builds are not supported. Use:\n"
            "  cmake -S cpp -B build && cmake --build build")
    endif()
    project(dual_arm_v2_2_control_sdk VERSION 1.0.0 LANGUAGES C CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Submodules live at the SDK repository root (sibling of this cpp/ directory).
get_filename_component(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(ROBOT_PLATFORM_UTILS_DIR "${SDK_ROOT}/robot_platform_utils")
set(CURI_UDP_DIR "${SDK_ROOT}/curi_udp")
add_library(dual_arm_v2_2_control_sdk SHARED
    src/control_api.cpp
    ${ROBOT_PLATFORM_UTILS_DIR}/cpp/src/config_loader.cpp
    ${ROBOT_PLATFORM_UTILS_DIR}/cpp/src/cuarm_message_handler.cpp
    ${ROBOT_PLATFORM_UTILS_DIR}/cpp/src/time_utils.cpp
    ${CURI_UDP_DIR}/c/curi_udp.c
)
add_library(dual_arm_v2_2_control_sdk::dual_arm_v2_2_control_sdk ALIAS dual_arm_v2_2_control_sdk)

include(GNUInstallDirs)

set_target_properties(dual_arm_v2_2_control_sdk PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(dual_arm_v2_2_control_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk>
    PRIVATE
        ${SDK_ROOT}
        ${ROBOT_PLATFORM_UTILS_DIR}/cpp/include
        ${CURI_UDP_DIR}/c
)

if(TARGET yaml-cpp::yaml-cpp)
    set(_dual_arm_yaml yaml-cpp::yaml-cpp)
else()
    set(_dual_arm_yaml yaml-cpp)
endif()

# yaml-cpp is linked into the shared library only; not exposed to SDK consumers.
target_link_libraries(dual_arm_v2_2_control_sdk
    PUBLIC Threads::Threads
    PRIVATE ${_dual_arm_yaml}
)
unset(_dual_arm_yaml)

set(_dual_arm_sdk_export dual_arm_v2_2_control_sdkTargets)
install(TARGETS dual_arm_v2_2_control_sdk
    EXPORT ${_dual_arm_sdk_export}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

set(_dual_arm_sdk_cmake_dir ${CMAKE_INSTALL_LIBDIR}/cmake/dual_arm_v2_2_control_sdk)
install(EXPORT ${_dual_arm_sdk_export}
    NAMESPACE dual_arm_v2_2_control_sdk::
    DESTINATION ${_dual_arm_sdk_cmake_dir}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/dual_arm_v2_2_control_sdkConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfig.cmake
    INSTALL_DESTINATION ${_dual_arm_sdk_cmake_dir}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfigVersion.cmake
    DESTINATION ${_dual_arm_sdk_cmake_dir}
)
unset(_dual_arm_sdk_export)
unset(_dual_arm_sdk_cmake_dir)
