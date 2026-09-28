cmake_minimum_required(VERSION 3.16)

add_library(dual_arm_v2_2_control_sdk_c SHARED
    src/control_api_c_impl.cpp
)
add_library(dual_arm_v2_2_control_sdk_c::dual_arm_v2_2_control_sdk_c
    ALIAS dual_arm_v2_2_control_sdk_c)

include(GNUInstallDirs)

set_target_properties(dual_arm_v2_2_control_sdk_c PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(dual_arm_v2_2_control_sdk_c
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk_c>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/include
)

target_link_libraries(dual_arm_v2_2_control_sdk_c
    PUBLIC dual_arm_v2_2_control_sdk
)

set(_dual_arm_c_sdk_export dual_arm_v2_2_control_sdk_cTargets)
install(TARGETS dual_arm_v2_2_control_sdk_c
    EXPORT ${_dual_arm_c_sdk_export}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk_c
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk_c
    FILES_MATCHING PATTERN "*.h"
)

set(_dual_arm_c_sdk_cmake_dir ${CMAKE_INSTALL_LIBDIR}/cmake/dual_arm_v2_2_control_sdk_c)
install(EXPORT ${_dual_arm_c_sdk_export}
    NAMESPACE dual_arm_v2_2_control_sdk_c::
    DESTINATION ${_dual_arm_c_sdk_cmake_dir}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/dual_arm_v2_2_control_sdk_cConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdk_cConfig.cmake
    INSTALL_DESTINATION ${_dual_arm_c_sdk_cmake_dir}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdk_cConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdk_cConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdk_cConfigVersion.cmake
    DESTINATION ${_dual_arm_c_sdk_cmake_dir}
)
unset(_dual_arm_c_sdk_export)
unset(_dual_arm_c_sdk_cmake_dir)
