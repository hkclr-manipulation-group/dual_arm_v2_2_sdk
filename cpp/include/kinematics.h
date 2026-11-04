#ifndef DUAL_ARM_V2_2_KINEMATICS_H
#define DUAL_ARM_V2_2_KINEMATICS_H

#include <memory>
#include <string>

#include "kinematics_types.h"

namespace dual_arm_v2_2_sdk {

class Kinematics {
public:
    /** @param arm_index planner index: 0=left, 2=right (7-DOF arms only). */
    explicit Kinematics(const std::string& config_prefix_path, int arm_index);
    ~Kinematics();

    Kinematics(const Kinematics&) = delete;
    Kinematics& operator=(const Kinematics&) = delete;

    int arm_index() const;

    ToolPose7 forward_tool(const Joint7Rad& q_rad) const;

    /** Tool-frame IK; seed and output in radians. */
    MotionRetCode inverse_tool(const ToolPose7& target,
                               const Joint7Rad& seed_rad,
                               Joint7Rad& out_rad,
                               const SingularityPolicy& policy) const;

    float sigma_min(const Joint7Rad& q_rad) const;

    bool within_soft_limits(const Joint7Rad& q_rad) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dual_arm_v2_2_sdk

#endif
