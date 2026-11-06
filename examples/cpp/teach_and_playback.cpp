/**
 * @file teach_and_playback.cpp
 * @brief 重力补偿示教采点：双臂拖动 + 键盘记录关节角到 txt。
 *
 * 流程：
 *   1. 等待 rt_control 状态
 *   2. 启用左/右臂全部关节（set_group）
 *   3. start_teach_mode(true) 进入重力补偿示教
 *   4. 按 s 或 SPACE 记录当前关节角（度）到文件
 *   5. 按 q 或 ENTER 退出示教；可用 teach_replay 回放同一文件
 *
 * 示教文件格式与 teach_replay 一致（17 列，单位：度）：
 *   idx, left_j1..j7, left_gripper, right_j1..j7, right_gripper
 *
 * 用法：
 *   ./teach_and_playback
 *   ./teach_and_playback /path/to/teach_points.txt
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::default_config_path;
using example::default_teach_file_path;
using example::ensure_teach_file_parent_dirs;
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

void append_joint_line(const ControlApi& api, const std::string& filepath,
                       int index) {
    auto [left_code, left_rad] = api.get_joint(Group::LEFT_ARM);
    auto [right_code, right_rad] = api.get_joint(Group::RIGHT_ARM);
    if (left_code != RetCode::SUCCESS || right_code != RetCode::SUCCESS ||
        left_rad.size() != 7 || right_rad.size() != 7) {
        std::cerr << "  -> Failed to read joint feedback, point not saved\n";
        return;
    }

    constexpr float kGripperPlaceholder = 0.0F;

    std::ostringstream line;
    line << index;
    line << std::fixed << std::setprecision(3);
    for (int i = 0; i < 7; ++i) line << ", " << left_rad[i] * kRadToDeg;
    line << ", " << kGripperPlaceholder;
    for (int i = 0; i < 7; ++i) line << ", " << right_rad[i] * kRadToDeg;
    line << ", " << kGripperPlaceholder;

    bool write_header = true;
    {
        std::ifstream in(filepath);
        write_header =
            !in.good() || in.peek() == std::ifstream::traits_type::eof();
    }

    ensure_teach_file_parent_dirs(filepath);
    std::ofstream out(filepath, std::ios::app);
    if (!out) {
        return;
    }
    if (write_header) {
        out << "# Teach points (unit: deg)\n";
        out << "# Columns: idx, left_j1..left_j7, left_gripper, "
               "right_j1..right_j7, right_gripper\n";
    }
    out << line.str() << '\n';
    std::cout << "  -> Saved point #" << index << '\n';
}

int record_teach_points(ControlApi& api, const std::string& filepath,
                        float debounce_s = 0.4F) {
    int count = 0;
    auto last_record = std::chrono::steady_clock::now() -
                       std::chrono::seconds(static_cast<int>(debounce_s) + 1);

    termios oldt{};
    tcgetattr(STDIN_FILENO, &oldt);
    termios newt = oldt;
    newt.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    try {
        while (true) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0) {
                continue;
            }

            char key = 0;
            if (read(STDIN_FILENO, &key, 1) != 1) continue;

            const auto now = std::chrono::steady_clock::now();
            const float elapsed =
                std::chrono::duration<float>(now - last_record).count();

            if ((key == 's' || key == ' ') && elapsed >= debounce_s) {
                ++count;
                append_joint_line(api, filepath, count);
                last_record = now;
            } else if (key == 'q' || key == '\r' || key == '\n') {
                break;
            }
        }
    } catch (...) {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        throw;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return count;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string record_file =
            argc > 1 ? argv[1] : default_teach_file_path();
        ensure_teach_file_parent_dirs(record_file);

        ControlApi api(default_config_path());
        std::vector<float> left_current;
        std::vector<float> head_current;

        std::cout << "Waiting for rt_control state...\n";
        if (!wait_for_joint_state(api, Group::LEFT_ARM, left_current, 3000)) {
            std::cerr << "No state received within 3 seconds.\n";
            return 1;
        }
        (void)head_current;
        if (!wait_for_joint_state(api, Group::HEAD, head_current, 3000)) {
            std::cerr << "No head state received within 3 seconds.\n";
            return 1;
        }

        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS) {
            std::cerr << "Failed to enable both arms.\n";
            return 1;
        }
        std::cout << "Both arms enabled (all joints)\n";

        std::cout << "Entering gravity-compensated teach mode...\n";
        const RetCode teach_start = api.start_teach_mode(true);
        if (teach_start != RetCode::SUCCESS) {
            std::cerr << "start_teach_mode: " << ret_code_name(teach_start)
                      << '\n';
            return 1;
        }
        std::cout << "Teach mode active. Move both arms by hand.\n";
        std::cout << "Press 's' or SPACE to record, 'q' or ENTER to stop.\n";

        const int num_points = record_teach_points(api, record_file);
        std::cout << "Recorded " << num_points << " point(s) -> " << record_file
                  << '\n';
        std::cout << "Hand off both arms now.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "Stopping teach mode...\n";
        const RetCode teach_stop = api.stop_teach_mode();
        if (teach_stop != RetCode::SUCCESS) {
            std::cerr << "stop_teach_mode: " << ret_code_name(teach_stop)
                      << '\n';
            return 1;
        }

        std::cout << "Done. Replay with: teach_replay " << record_file << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
