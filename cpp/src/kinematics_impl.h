#ifndef DUAL_ARM_V2_2_KINEMATICS_IMPL_H
#define DUAL_ARM_V2_2_KINEMATICS_IMPL_H

#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "kinematics_types.h"

#include "cuarm_robotics/src/robotics/robotics.h"

namespace dual_arm_v2_2_sdk {

struct Kinematics::Impl {
    std::unique_ptr<Robotics> robotics_;
    int arm_i_ = 0;
    int joint_size_ = 7;
    int last_joint_id_ = 6;
    Joint7Rad joint_lower_rad{};
    Joint7Rad joint_upper_rad{};

    void initialize(const std::string& config_prefix_path, const YAML::Node& config);
};

}  // namespace dual_arm_v2_2_sdk

#endif
