cmake_minimum_required(VERSION 3.16)
project(dual_arm_v2_2_control_sdk LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

get_filename_component(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(EXAMPLE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)

add_library(dual_arm_v2_2_control_sdk
    src/control_api.cpp
    ${REPOSITORY_ROOT}/robot_platform_utils/cpp/src/config_loader.cpp
    ${REPOSITORY_ROOT}/robot_platform_utils/cpp/src/cuarm_message_handler.cpp
    ${REPOSITORY_ROOT}/robot_platform_utils/cpp/src/time_utils.cpp
    ${REPOSITORY_ROOT}/curi_udp/c/curi_udp.c
)

target_include_directories(dual_arm_v2_2_control_sdk PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${REPOSITORY_ROOT}
    ${REPOSITORY_ROOT}/robot_platform_utils/cpp/include
    ${REPOSITORY_ROOT}/curi_udp/c
)

target_link_libraries(dual_arm_v2_2_control_sdk PUBLIC
    Threads::Threads
    yaml-cpp::yaml-cpp
)

function(add_sdk_example target)
    add_executable(${target} examples/${target}.cpp)
    target_include_directories(${target} PRIVATE ${EXAMPLE_INCLUDE_DIR})
    target_link_libraries(${target} PRIVATE dual_arm_v2_2_control_sdk)
endfunction()

# Getting started
add_sdk_example(go_home)
add_sdk_example(move_arms_demo)

# Applications
add_sdk_example(teach_replay)

# Validation / advanced
add_sdk_example(test_control_api)
add_sdk_example(test_control_api_various_command_frequency)
add_sdk_example(test_move_joint_curve_fit)
add_sdk_example(test_gripper_api)
add_sdk_example(test_dual_gripper_api)
add_sdk_example(test_arm_gripper_safe)
