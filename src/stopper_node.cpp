// === Robot Navigation & Logging Node (ROS2) ===
// Author: Syed Zain Hasan
// Description: Reactive robot controller using laser scan and odometry feedback
// Logs trajectory, velocities, and laser scans to CSV for offline analysis

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

// === Utility Structures ===
struct EulerAngles { double roll, pitch, yaw; };
struct Quaternion { double w, x, y, z; };

// === Converts Quaternion to Euler Angles ===
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    angles.pitch = std::fabs(sinp) >= 1 ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

// === Main ROS2 Node ===
class Stopper : public rclcpp::Node {
public:
    Stopper()
    : Node("stopper_node"), count_(0) {
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Stopper::odomCallback, this, std::placeholders::_1));
        laserScan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
    }

    void start();

private:
    // === Robot Movement Methods ===
    void moveStop();
    void moveForward(double speed);
    void moveRight(double speed);
    void moveForwardRight(double forward, double turn);

    // === ROS Callback Handlers ===
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    // === Helper Functions ===
    void transformMapPoint(std::ofstream &fp, double range, double laserTh,
                           double robotTh, double robotX, double robotY);

    // === File Paths & Utilities ===
    std::string file_path = "/tmp/ros_logs/";
    std::string file_name(const std::string &name) {
        return file_path + name;
    }

    std::ofstream odomTrajFile, odomVelFile, laserFile, laserMapFile;

    // === ROS Handles ===
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;

    // === Robot State ===
    Quaternion robotQuat;
    EulerAngles robotAngles;
    double robotYaw;
    double posX = 0.0, posY = 0.0;
    double frontRange, mleftRange, leftRange, mrightRange, rightRange;
    int stage = 1, index = 0, cycle = 0;

    constexpr static double STOP = 0.0;

    constexpr static double VEL[3][3] = {
        {0.1, 0.2, 0.3},        // Forward: low/med/high
        {0.3, 0.6, 1.0},        // Left
        {-0.35, -0.4, -0.6}     // Right
    };

    const double threshold[5] = {1.2, 1.3, 0.65, 0.7, 0.4};
};

// === Movement Implementations ===
void Stopper::moveStop() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = STOP;
    publisher_->publish(msg);
}

void Stopper::moveForward(double speed) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = speed;
    publisher_->publish(msg);
}

void Stopper::moveRight(double speed) {
    geometry_msgs::msg::Twist msg;
    msg.angular.z = speed;
    publisher_->publish(msg);
}

void Stopper::moveForwardRight(double forward, double turn) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = forward;
    msg.angular.z = turn;
    publisher_->publish(msg);
}

// === Laser Scan Processing ===
void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int radar[5] = {300, 250, 200, 150, 100};
    leftRange = scan->ranges[radar[0]];
    mleftRange = scan->ranges[radar[1]];
    frontRange = scan->ranges[radar[2]];
    mrightRange = scan->ranges[radar[3]];
    rightRange = scan->ranges[radar[4]];

    laserFile << leftRange << "," << mleftRange << "," << frontRange << ","
              << mrightRange << "," << rightRange << "," << index++ << std::endl;

    // Mapping
    transformMapPoint(laserMapFile, frontRange, 0, robotYaw, posX, posY);
    transformMapPoint(laserMapFile, mleftRange, M_PI / 4, robotYaw, posX, posY);
    transformMapPoint(laserMapFile, leftRange, M_PI / 2, robotYaw, posX, posY);
    transformMapPoint(laserMapFile, mrightRange, -M_PI / 4, robotYaw, posX, posY);
    transformMapPoint(laserMapFile, rightRange, -M_PI / 2, robotYaw, posX, posY);

    // State machine logic
    switch (stage) {
        case 1: frontRange > threshold[0] ? moveForward(VEL[0][1]) : stage = 2; break;
        case 2: mleftRange < threshold[1] ? moveForwardRight(VEL[0][1], VEL[2][1]) : stage = 3; break;
        case 3: frontRange > threshold[2] ? moveForward(VEL[0][1]) : stage = 4; break;
        case 4: mleftRange > threshold[3] ? moveForwardRight(VEL[0][0], VEL[2][0]) : stage = 5; break;
        case 5: frontRange > threshold[4] ? moveForward(VEL[0][1]) : stage = 6; break;
        case 6: moveStop(); break;
    }
}

// === Coordinate Transformation for Mapping ===
void Stopper::transformMapPoint(std::ofstream &fp, double range, double laserTh,
                                double robotTh, double robotX, double robotY) {
    double x = range * std::cos(robotTh + laserTh) + robotX;
    double y = range * std::sin(robotTh + laserTh) + robotY;
    fp << std::max(0.0, x) << "," << std::max(0.0, y) << std::endl;
}

// === Odometry Callback ===
void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robotQuat = { msg->pose.pose.orientation.w,
                  msg->pose.pose.orientation.x,
                  msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z };

    robotAngles = ToEulerAngles(robotQuat);
    robotYaw = robotAngles.yaw;

    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;

    double velocity = msg->twist.twist.linear.x;

    RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Stage: %d", posX, posY, stage);
    odomVelFile << cycle << "," << velocity << std::endl;
    odomTrajFile << posX << "," << posY << std::endl;
    cycle++;
}

// === Start Movement and Logging ===
void Stopper::start() {
    odomTrajFile.open(file_name("odom_trajectory.csv"), std::ios::trunc);
    odomVelFile.open(file_name("velocity_data.csv"), std::ios::trunc);
    laserFile.open(file_name("laser_scan.csv"), std::ios::trunc);
    laserMapFile.open(file_name("laser_map.csv"), std::ios::trunc);

    RCLCPP_INFO(this->get_logger(), "Navigation started...");

    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }

    odomTrajFile.close();
    odomVelFile.close();
    laserFile.close();
    laserMapFile.close();
}

// === Main Entry Point ===
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Stopper>();
    node->start();
    rclcpp::shutdown();
    return 0;
}
