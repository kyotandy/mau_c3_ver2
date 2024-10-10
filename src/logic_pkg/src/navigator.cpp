#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include "fuzzy_controller.h"  // FuzzyControllerクラスをインクルード
#include <map>
#include <string>

// ロボットのステータス辞書
std::map<std::string, std::string> robot_status_dict = {
    {"STRAIGHT", "10"},
    {"TURN_RIGHT", "11"},
    {"TURN_LEFT", "12"},
    {"PAUSE", "13"},
    {"STOP", "14"}
};

std::map<std::string, std::string> rotation_status_dict = {
    {"LEFT", "0"},
    {"CENTER", "1"},
    {"RIGHT", "2"}
};

std::map<std::string, std::string> clump_status_dict = {
    {"ALL_OFF", "0"},
    {"FORWARD_ON", "1"},
    {"BACKWARD_ON", "2"},
    {"ALL_ON", "3"}
};

std::map<std::string, std::string> rail_status_dict = {
    {"RAIL_OFF", "0"},
    {"RAIL_ON", "1"}
};

std::map<std::string, std::string> panel_status_dict = {
    {"PANEL_ALL_ON", "0"},
    {"PANEL_RIGHT_FORWARD_OFF", "1"},
    {"PANEL_RIGHT_BACKWARD_OFF", "2"},
    {"PANEL_LEFT_FORWARD_OFF", "3"},
    {"PANEL_LEFT_BACKWARD_OFF", "4"}
};

std::map<std::string, std::string> wheel_status_dict = {
    {"WHEEL_STOP", "0"},
    {"WHEEL_STRAIGHT", "1"},
    {"WHEEL_RIGHT", "2"},
    {"WHEEL_LEFT", "3"}
};

class NavigatorNode {
public:
    NavigatorNode() {
        ros::NodeHandle nh;

        // サブスクライバーの初期化
        data_sub_ = nh.subscribe("image_info", 1, &NavigatorNode::callback, this);

        // パブリッシャーの初期化
        rotation_position_pub_ = nh.advertise<std_msgs::String>("rotation_position", 1);
        clump_position_pub_ = nh.advertise<std_msgs::String>("clump_position", 1);
        rail_position_pub_ = nh.advertise<std_msgs::String>("rail_position", 1);
        panel_position_pub_ = nh.advertise<std_msgs::String>("panel_position", 1);
        wheel_speed_pub_ = nh.advertise<std_msgs::Int32MultiArray>("wheel_speeds", 1);

        robot_status_ = robot_status_dict["STRAIGHT"];

        // メッセージの初期化
        rotatoin_msg_.data = rotation_status_dict["CENTER"];
        clump_msg_.data = clump_status_dict["ALL_OFF"];
        rail_msg_.data = rail_status_dict["RAIL_ON"];
        panel_msg_.data = panel_status_dict["PANEL_ALL_ON"];
        wheel_speeds_msg_.data.resize(2, 0);

        acceptable_offset_mm_ = 10;
        acceptable_angle_deg_ = 5;
        acceptable_arc_distance_mm_ = 5;
        log_counter_ = 0;

        // ファジィコントローラの初期化
        fuzzy_controller_ = new FuzzyController();
    }

    // メッセージのパブリッシュ
    void publish_msg() {
        rotation_position_pub_.publish(rotatoin_msg_);
        clump_position_pub_.publish(clump_msg_);
        rail_position_pub_.publish(rail_msg_);
        panel_position_pub_.publish(panel_msg_);
        wheel_speed_pub_.publish(wheel_speeds_msg_);
    }

    // コマンドをチェック
    bool cmd_check(const std::string& cmd) {
        if (cmd.front() != 'S' || cmd.back() != 'E' || cmd.length() != 7) {
            error_msg_ = "The command must be entered in the format S00000E";
            return false;
        }
        if (rotation_status_dict.find(cmd.substr(1, 1)) == rotation_status_dict.end()) {
            error_msg_ = "1st number must be 0~2";
            return false;
        }
        if (clump_status_dict.find(cmd.substr(2, 1)) == clump_status_dict.end()) {
            error_msg_ = "2nd number must be 0 or 1";
            return false;
        }
        if (rail_status_dict.find(cmd.substr(3, 1)) == rail_status_dict.end()) {
            error_msg_ = "3rd number must be 0 or 1";
            return false;
        }
        if (panel_status_dict.find(cmd.substr(4, 1)) == panel_status_dict.end()) {
            error_msg_ = "4th number must be 0~4";
            return false;
        }
        if (wheel_status_dict.find(cmd.substr(5, 1)) == wheel_status_dict.end()) {
            error_msg_ = "5th number must be 0~3";
            return false;
        }
        return true;
    }

    // モーター速度の設定
    void motor_speed_set(const std::string& wheel_cmd, float speed_weight) {
        if (wheel_cmd == wheel_status_dict["WHEEL_STOP"]) {
            left_motor_speed_ = 0;
            right_motor_speed_ = 0;
        } else if (wheel_cmd == wheel_status_dict["WHEEL_STRAIGHT"]) {
            left_motor_speed_ = 100;
            right_motor_speed_ = 100 + abs(speed_weight);
        } else if (wheel_cmd == wheel_status_dict["WHEEL_LEFT"]) {
            left_motor_speed_ = -50;
            right_motor_speed_ = 50;
        } else if (wheel_cmd == wheel_status_dict["WHEEL_RIGHT"]) {
            left_motor_speed_ = 50;
            right_motor_speed_ = -50;
        }
    }

    // ロボットのステータスを設定
    void robot_status_set(const std::string& wheel_cmd) {
        if (wheel_cmd == wheel_status_dict["WHEEL_STOP"]) {
            robot_status_ = robot_status_dict["PAUSE"];
        } else if (wheel_cmd == wheel_status_dict["WHEEL_STRAIGHT"]) {
            robot_status_ = robot_status_dict["STRAIGHT"];
        } else if (wheel_cmd == wheel_status_dict["WHEEL_LEFT"]) {
            robot_status_ = robot_status_dict["TURN_LEFT"];
        } else if (wheel_cmd == wheel_status_dict["WHEEL_RIGHT"]) {
            robot_status_ = robot_status_dict["TURN_RIGHT"];
        }
    }

    // ファジィ推論でモーター速度を決定
    float motor_speed_fuzzy(float offset_mm, float angle_deg) {
        return fuzzy_controller_->compute(offset_mm, angle_deg);
    }

    // コールバック関数
    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        float offset_mm = msg->data[0];
        float angle_deg = msg->data[1];
        float arc_distance_mm = msg->data[2];
        int pause_signal = static_cast<int>(msg->data[3]);
        float motor_speed_weight = motor_speed_fuzzy(offset_mm, angle_deg);

        if (robot_status_ == robot_status_dict["STRAIGHT"]) {
            if (abs(offset_mm) <= acceptable_offset_mm_) {
                motor_speed_set(wheel_status_dict["WHEEL_STRAIGHT"], motor_speed_weight);
            } else if (abs(offset_mm) > acceptable_offset_mm_) {
                robot_status_ = robot_status_dict["STOP"];
                motor_speed_set(wheel_status_dict["WHEEL_STOP"], motor_speed_weight);
            } else if (pause_signal) {
                robot_status_ = robot_status_dict["PAUSE"];
                motor_speed_set(wheel_status_dict["WHEEL_STOP"], motor_speed_weight);
            }
        } else if (robot_status_ == robot_status_dict["TURN_RIGHT"]) {
            if (abs(arc_distance_mm) > acceptable_arc_distance_mm_) {
                robot_status_ = robot_status_dict["STOP"];
                motor_speed_set(wheel_status_dict["WHEEL_STOP"], motor_speed_weight);
            } else if (pause_signal) {
                robot_status_ = robot_status_dict["PAUSE"];
                motor_speed_set(wheel_status_dict["WHEEL_STOP"], motor_speed_weight);
            }
        } else if (robot_status_ == robot_status_dict["TURN_LEFT"]) {
            if (abs(arc_distance_mm) > acceptable_arc_distance_mm_) {
                robot_status_ = robot_status_dict["STOP"];
                motor_speed_set(wheel_status_dict["WHEEL_STOP"], motor_speed_weight);
            } else if (pause_signal) {
                robot_status_ = robot_status_dict["PAUSE"];
                motor_speed_set(wheel_status_dict["WHEEL_STOP"], motor_speed_weight);
            }
        } else if (robot_status_ == robot_status_dict["PAUSE"]) {
            std::string cmd;
            std::cout << "Please input command: ";
            std::cin >> cmd;
            while (!cmd_check(cmd)) {
                std::cout << "Command error: " << error_msg_ << std::endl;
                std::cin >> cmd;
            }

            // コマンド処理
            std::string rotation_cmd = cmd.substr(1, 1);
            std::string clump_cmd = cmd.substr(2, 1);
            std::string rail_cmd = cmd.substr(3, 1);
            std::string panel_cmd = cmd.substr(4, 1);
            std::string wheel_cmd = cmd.substr(5, 1);

            rotatoin_msg_.data = rotation_cmd;
            clump_msg_.data = clump_cmd;
            rail_msg_.data = rail_cmd;
            panel_msg_.data = panel_cmd;
            motor_speed_set(wheel_cmd, motor_speed_weight);
            robot_status_set(wheel_cmd);
        } else if (robot_status_ == robot_status_dict["STOP"]) {
            publish_msg();
            std::string cmd;
            do {
                std::cout << "Please input \"done\": ";
                std::cin >> cmd;
            } while (cmd != "done");

            robot_status_ = robot_status_dict["STRAIGHT"];
        }

        log_counter_++;
        if (log_counter_ >= 20) {
            ROS_INFO("status: %s, offset: %.2f, angle: %.2f, speed weight: %.2f, left motor: %d, right motor: %d",
                     robot_status_.c_str(), offset_mm, angle_deg, motor_speed_weight, left_motor_speed_, right_motor_speed_);
            log_counter_ = 0;
        }

        // メッセージをパブリッシュ
        publish_msg();
    }

    void run() {
        ros::spin();
    }

private:
    ros::Subscriber data_sub_;
    ros::Publisher rotation_position_pub_, clump_position_pub_, rail_position_pub_, panel_position_pub_, wheel_speed_pub_;
    std::string robot_status_, error_msg_;
    std_msgs::String rotatoin_msg_, clump_msg_, rail_msg_, panel_msg_;
    std_msgs::Int32MultiArray wheel_speeds_msg_;
    int left_motor_speed_, right_motor_speed_, log_counter_;
    float acceptable_offset_mm_, acceptable_angle_deg_, acceptable_arc_distance_mm_;
    FuzzyController* fuzzy_controller_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigator_node");
    NavigatorNode navigator_node;
    navigator_node.run();
    return 0;
}