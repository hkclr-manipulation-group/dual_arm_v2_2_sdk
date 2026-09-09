#ifndef DUAL_ARM_V2_2_EXAMPLE_COMMON_H
#define DUAL_ARM_V2_2_EXAMPLE_COMMON_H

#include "control_api.h"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace example {

// Path is relative to running binaries from cpp/build/.
constexpr const char* kConfigPath = "../../../cuarm_configuration/dual_v2_2";
constexpr int kDefaultStateTimeoutMs = 3000;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kRadToDeg = 180.0F / kPi;

inline const char* ret_code_name(dual_arm_v2_2_sdk::RetCode code) {
    using dual_arm_v2_2_sdk::RetCode;
    switch (code) {
        case RetCode::SUCCESS: return "SUCCESS";
        case RetCode::CONTROLLER_ERROR: return "CONTROLLER_ERROR";
        case RetCode::SEND_FAILED: return "SEND_FAILED";
        case RetCode::RECEIVE_FAILED: return "RECEIVE_FAILED";
        case RetCode::PARSE_FAILED: return "PARSE_FAILED";
        case RetCode::TIMEOUT: return "TIMEOUT";
    }
    return "UNKNOWN";
}

inline const char* group_name(dual_arm_v2_2_sdk::Group group) {
    using dual_arm_v2_2_sdk::Group;
    switch (group) {
        case Group::ALL: return "ALL";
        case Group::LEFT_ARM: return "LEFT_ARM";
        case Group::RIGHT_ARM: return "RIGHT_ARM";
        case Group::HEAD: return "HEAD";
        case Group::ELEVATOR: return "ELEVATOR";
    }
    return "UNKNOWN";
}

inline bool wait_for_joint_state(dual_arm_v2_2_sdk::ControlApi& api,
                                 dual_arm_v2_2_sdk::Group group,
                                 std::vector<float>& joints,
                                 int timeout_ms = kDefaultStateTimeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    do {
        auto [code, value] = api.get_joint(group);
        if (code == dual_arm_v2_2_sdk::RetCode::SUCCESS && !value.empty()) {
            joints = std::move(value);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

inline bool wait_for_enter(const char* message) {
    std::cout << '\n' << message
              << "\nPress Enter to continue, or Ctrl+D to abort..."
              << std::flush;
    std::string input;
    return static_cast<bool>(std::getline(std::cin, input));
}

}  // namespace example

#endif
