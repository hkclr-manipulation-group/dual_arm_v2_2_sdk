#include "kinematics.h"
#include "kinematics_impl.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <Eigen/Dense>

#include "robot_platform_utils/cpp/include/config_loader.h"

namespace dual_arm_v2_2_sdk {
namespace {

constexpr float kPi = 3.14159265358979323846F;

VectorXd joint_array_to_vector(const Joint7Rad& q) {
    VectorXd v(7);
    for (int i = 0; i < 7; ++i) v(i) = static_cast<double>(q[i]);
    return v;
}

Joint7Rad vector_to_joint_array(const VectorXd& v) {
    Joint7Rad q{};
    for (int i = 0; i < 7; ++i) q[i] = static_cast<float>(v(i));
    return q;
}

ToolPose7 vector7_to_tool_pose(const Vector7d& v) {
    return ToolPose7{static_cast<float>(v(0)), static_cast<float>(v(1)),
                     static_cast<float>(v(2)), static_cast<float>(v(3)),
                     static_cast<float>(v(4)), static_cast<float>(v(5)),
                     static_cast<float>(v(6))};
}

float joint_delta_norm(const Joint7Rad& a, const Joint7Rad& b) {
    float s = 0.0F;
    for (int i = 0; i < 7; ++i) {
        const float d = a[i] - b[i];
        s += d * d;
    }
    return std::sqrt(s);
}

Joint7Rad perturb_seed(const Joint7Rad& base, int variant) {
    Joint7Rad q = base;
    const float bump[] = {0.05F, -0.05F, 0.1F, -0.08F};
    q[static_cast<size_t>(variant % 7)] +=
        bump[static_cast<size_t>(variant % 4)];
    return q;
}

}  // namespace

void Kinematics::Impl::initialize(const std::string& config_prefix_path,
                                  const YAML::Node& config) {
    const YAML::Node arm_entries = config["robot"]["arm"];
    if (arm_i_ < 0 || arm_i_ >= static_cast<int>(arm_entries.size())) {
        throw std::out_of_range("Kinematics: invalid arm_index " +
                                std::to_string(arm_i_));
    }
    YAML::Node arm_config = arm_entries[arm_i_];
    joint_size_ = arm_config["joint_size"].as<int>();
    if (joint_size_ != 7) {
        throw std::runtime_error("Kinematics: only 7-DOF arms supported");
    }
    last_joint_id_ = joint_size_ - 1;

    Eigen::Matrix<float, 2, 7> joint_pos_limits;
    for (int i = 0; i < joint_size_; ++i) {
        joint_pos_limits(0, i) =
            arm_config["safety"]["joint_soft_limit"]["position"][0][i]
                .as<float>() *
            kPi / 180.0F;
        joint_pos_limits(1, i) =
            arm_config["safety"]["joint_soft_limit"]["position"][1][i]
                .as<float>() *
            kPi / 180.0F;
        joint_lower_rad[i] = joint_pos_limits(0, i);
        joint_upper_rad[i] = joint_pos_limits(1, i);
    }

    const std::string robot_type = arm_config["type"].as<std::string>();
    std::vector<std::string> joint_names =
        arm_config["urdf_joints"].as<std::vector<std::string>>();
    std::vector<std::string> link_names =
        arm_config["urdf_links"].as<std::vector<std::string>>();
    Vector3d tool_offset = Vector3d::Zero();
    if (config["robot"]["gripper"] &&
        config["robot"]["gripper"].size() > static_cast<size_t>(arm_i_)) {
        YAML::Node offset_node =
            config["robot"]["gripper"][arm_i_]["offset_from_ee"];
        tool_offset << offset_node[0].as<float>(), offset_node[1].as<float>(),
            offset_node[2].as<float>();
    }
    const std::string urdf_filename = config["robot"]["urdf"].as<std::string>();
    const std::string urdf_path = std::filesystem::canonical(
        std::filesystem::path(config_prefix_path + '/' + urdf_filename));
    const std::string library_name =
        config["rt_control"]["robotics_library"].as<std::string>();
    const std::string ee_link = link_names.back();
    robotics_ = std::make_unique<Robotics>(
        urdf_path, joint_names, ee_link, tool_offset, arm_i_, joint_size_,
        joint_pos_limits, library_name);
    VectorXf gear_ratio(joint_size_);
    robotics_->setArm(robot_type, gear_ratio);
}

Kinematics::Kinematics(const std::string& config_prefix_path, int arm_index)
    : impl_(std::make_unique<Impl>()) {
    impl_->arm_i_ = arm_index;
    YAML::Node config = loadYamlConfig(config_prefix_path + "/config.yaml");
    impl_->initialize(config_prefix_path, config);
}

Kinematics::~Kinematics() = default;

int Kinematics::arm_index() const { return impl_->arm_i_; }

ToolPose7 Kinematics::forward_tool(const Joint7Rad& q_rad) const {
    Vector7d tool_pose;
    impl_->robotics_->FK(joint_array_to_vector(q_rad), nullptr, &tool_pose);
    return vector7_to_tool_pose(tool_pose);
}

bool Kinematics::within_soft_limits(const Joint7Rad& q_rad) const {
    for (int i = 0; i < 7; ++i) {
        if (q_rad[i] < impl_->joint_lower_rad[i] ||
            q_rad[i] > impl_->joint_upper_rad[i]) {
            return false;
        }
    }
    return true;
}

float Kinematics::sigma_min(const Joint7Rad& q_rad) const {
    const Matrix6Xd J = impl_->robotics_->getJointJacobian(
        joint_array_to_vector(q_rad), impl_->last_joint_id_);
    Eigen::JacobiSVD<Matrix6Xd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& s = svd.singularValues();
    if (s.size() == 0) return 0.0F;
    return static_cast<float>(s(s.size() - 1));
}

MotionRetCode Kinematics::inverse_tool(const ToolPose7& target,
                                       const Joint7Rad& seed_rad,
                                       Joint7Rad& out_rad,
                                       const SingularityPolicy& policy) const {
    Eigen::Quaterniond quat(target[3], target[4], target[5], target[6]);
    quat.normalize();
    const Matrix3d R = quat.toRotationMatrix();
    const Vector3d P(target[0], target[1], target[2]);

    MotionRetCode best_code = MotionRetCode::IK_FAILED;
    Joint7Rad best_q = seed_rad;
    float best_sigma = -1.0F;
    float best_delta = 1e9F;

    for (int attempt = 0; attempt < policy.max_ik_retries; ++attempt) {
        Joint7Rad seed =
            attempt == 0 ? seed_rad : perturb_seed(seed_rad, attempt);
        VectorXd q = joint_array_to_vector(seed);
        if (!impl_->robotics_->IK(R, P, q)) continue;

        Joint7Rad candidate = vector_to_joint_array(q);
        if (!within_soft_limits(candidate)) {
            best_code = MotionRetCode::JOINT_LIMIT;
            continue;
        }
        const float sigma = sigma_min(candidate);
        if (sigma < policy.sigma_min_threshold) {
            best_code = MotionRetCode::IK_NEAR_SINGULAR;
            continue;
        }
        const float delta = joint_delta_norm(candidate, seed_rad);
        if (delta > policy.max_joint_jump_rad) continue;

        if (sigma > best_sigma ||
            (std::abs(sigma - best_sigma) < 1e-6F && delta < best_delta)) {
            best_sigma = sigma;
            best_delta = delta;
            best_q = candidate;
            best_code = MotionRetCode::SUCCESS;
        }
    }

    if (best_code != MotionRetCode::SUCCESS) return best_code;

    out_rad = best_q;
    const float step = joint_delta_norm(best_q, seed_rad);
    if (step > policy.max_delta_q_rad) {
        const float scale = policy.max_delta_q_rad / step;
        for (int i = 0; i < 7; ++i) {
            out_rad[i] = seed_rad[i] + (best_q[i] - seed_rad[i]) * scale;
        }
    }
    return MotionRetCode::SUCCESS;
}

}  // namespace dual_arm_v2_2_sdk
