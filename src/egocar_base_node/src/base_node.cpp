#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <memory>
#include <string>

// 使用 C++11 的 using 來簡化程式碼
using std::placeholders::_1;

// 在 ROS2 中，節點通常被寫成一個繼承自 rclcpp::Node 的類別
class RobotBase : public rclcpp::Node {
public:
    // 建構子，初始化節點名稱為 "odometry_publisher"
    RobotBase() : Node("odometry_publisher"),
        x_pos_(0.0), y_pos_(0.0), heading_(0.0) {

        // 1. 宣告與獲取參數 (ROS2 的新方式)
        this->declare_parameter<double>("linear_scale_x", 1.0);
        this->declare_parameter<double>("linear_scale_y", 1.0);
        this->declare_parameter<double>("angular_scale", 1.0);
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");

        this->get_parameter("linear_scale_x", linear_scale_x_);
        this->get_parameter("linear_scale_y", linear_scale_y_);
        this->get_parameter("angular_scale", angular_scale_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_footprint_frame", base_footprint_frame_);

        RCLCPP_INFO(this->get_logger(), "Parameters: linear_scale_x=%.2f, linear_scale_y=%.2f, angular_scale=%.2f", 
                    linear_scale_x_, linear_scale_y_, angular_scale_);

        // 2. 建立訂閱者 (Subscriber)
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/vel_raw", // 根據您的 launch 檔，話題已 remap 為 /vel_raw
            10, 
            std::bind(&RobotBase::velCallback, this, _1)
        );

        // 3. 建立發布者 (Publisher)
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10); // remap 為 /odom_raw

        // 4. 建立 TF 廣播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 初始化時間
        last_time_ = this->now();
    }

private:
    // 訂閱者的回呼函式
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 從收到的 Twist 訊息中獲取速度，並應用校正比例
        double vx = msg->linear.x * linear_scale_x_;
        double vy = msg->linear.y * linear_scale_y_;
        double vtheta = msg->angular.z * angular_scale_;

        // 透過積分計算機器人的位置和航向角
        double delta_x = (vx * cos(heading_) - vy * sin(heading_)) * dt;
        double delta_y = (vx * sin(heading_) + vy * cos(heading_)) * dt;
        double delta_th = vtheta * dt;

        x_pos_ += delta_x;
        y_pos_ += delta_y;
        heading_ += delta_th;

        // --- 準備並發布 Odometry 訊息 ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_footprint_frame_;

        // 設定位置
        odom_msg.pose.pose.position.x = x_pos_;
        odom_msg.pose.pose.position.y = y_pos_;
        odom_msg.pose.pose.position.z = 0.0;

        // 使用 tf2::Quaternion 將歐拉角 (RPY) 轉換為四元數
        tf2::Quaternion q;
        q.setRPY(0, 0, heading_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 設定速度
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = vtheta;

        odom_pub_->publish(odom_msg);

        // --- 準備並廣播 TF 變換 ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_footprint_frame_;

        t.transform.translation.x = x_pos_;
        t.transform.translation.y = y_pos_;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    // 宣告成員變數
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double linear_scale_x_;
    double linear_scale_y_;
    double angular_scale_;
    std::string odom_frame_;
    std::string base_footprint_frame_;

    double x_pos_, y_pos_, heading_;
    rclcpp::Time last_time_;
};

// ROS2 節點的主程式入口點
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // 初始化 ROS2
    auto node = std::make_shared<RobotBase>(); // 建立 RobotBase 節點
    rclcpp::spin(node); // 執行節點，等待回呼函式被觸發
    rclcpp::shutdown(); // 關閉 ROS2
    return 0;
}
