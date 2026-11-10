find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.6
    )
    # Offline builds: pass -DFETCHCONTENT_SOURCE_DIR_PYBIND11=/path/to/pybind11-src
    # to reuse an already downloaded source tree instead of cloning.
    FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(dual_arm_v2_2_sdk control_api_module.cpp)
target_link_libraries(dual_arm_v2_2_sdk PRIVATE dual_arm_v2_2_control_sdk)
target_compile_definitions(dual_arm_v2_2_sdk PRIVATE
    DUAL_ARM_PY_VERSION=${PROJECT_VERSION}
)

# Build tree: $ORIGIN/libs is unused, so also search the SDK library directory.
# The wheel script rewrites the extension RPATH to $ORIGIN/libs only.
set_target_properties(dual_arm_v2_2_sdk PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/python"
    BUILD_RPATH "$ORIGIN/libs:${CMAKE_BINARY_DIR}/lib"
    INSTALL_RPATH "$ORIGIN/libs"
)
