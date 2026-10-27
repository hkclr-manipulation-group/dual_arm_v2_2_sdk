cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "In-source builds are not supported. Use:\n"
        "  cmake -S . -B build && cmake --build build")
endif()

project(dual_arm_v2_2_control_sdk VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install"
        CACHE PATH "Install path prefix" FORCE)
endif()
message(STATUS "Dual Arm V2.2 SDK install prefix: ${CMAKE_INSTALL_PREFIX}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(cpp)
add_subdirectory(c)

set(_dual_arm_cpp_examples
    go_home
    move_arms_demo
    test_move_line
    test_single_joint_increment
    test_get_joint_all
    teach_replay
    test_control_api
    test_control_api_various_command_frequency
    test_move_joint_curve_fit
    test_gripper_api
    test_dual_gripper_api
    test_arm_gripper_safe
    test_dual_arm_100hz_follow
    dual_arm_independent_move_line
    staggered_dual_arm_stream
)

option(DUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES "Build C++ SDK example programs" ON)
if(DUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES)
    set(_dual_arm_cpp_example_dir ${CMAKE_SOURCE_DIR}/examples/cpp)

    function(add_sdk_cpp_example target)
        add_executable(${target}_cpp ${_dual_arm_cpp_example_dir}/${target}.cpp)
        set_target_properties(${target}_cpp PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/cpp
            OUTPUT_NAME ${target}
        )
        target_include_directories(${target}_cpp PRIVATE ${_dual_arm_cpp_example_dir})
        target_link_libraries(${target}_cpp PRIVATE dual_arm_v2_2_control_sdk)
    endfunction()

    foreach(_dual_arm_example ${_dual_arm_cpp_examples})
        add_sdk_cpp_example(${_dual_arm_example})
    endforeach()

    unset(_dual_arm_cpp_example_dir)
endif()

option(DUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES "Build C SDK example programs" ON)
if(DUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES)
    find_package(Threads REQUIRED)
    set(_dual_arm_c_example_dir ${CMAKE_SOURCE_DIR}/examples/c)

    function(add_sdk_c_example target)
        add_executable(${target}_c ${_dual_arm_c_example_dir}/${target}.c)
        set_target_properties(${target}_c PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/c
            OUTPUT_NAME ${target}
        )
        target_include_directories(${target}_c PRIVATE ${_dual_arm_c_example_dir})
        target_link_libraries(${target}_c PRIVATE
            dual_arm_v2_2_control_sdk_c Threads::Threads m)
    endfunction()

    foreach(_dual_arm_example ${_dual_arm_cpp_examples})
        add_sdk_c_example(${_dual_arm_example})
    endforeach()
    foreach(_dual_arm_example ${_dual_arm_c_only_examples})
        add_sdk_c_example(${_dual_arm_example})
    endforeach()

    unset(_dual_arm_c_example_dir)
endif()

unset(_dual_arm_cpp_examples)
unset(_dual_arm_c_only_examples)
