#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <charging_station/GoToChargingStation.h>
#include "charging_station/dec_circle.h"
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

geometry_msgs::Pose current_pose;
geometry_msgs::Pose2D jieware_pose;

ros::Publisher cmd_vel_pub;

// 回调函数：获取机器人当前位姿
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose = msg->pose.pose;
}

// 回调函数：获取机器人定位的当前位姿
void poseCallback(const geometry_msgs::Pose2D &msg)
{
    jieware_pose = msg;
}

// 角度归一化函数
double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

// 转向目标点的函数
void turnToTarget(geometry_msgs::Pose target_pose)
{
    ROS_INFO("Turning to target point");
    // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    // double roll, pitch, yaw;
    // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // double target_yaw = atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x);
    // double angle_diff = normalizeAngle(target_yaw - yaw);

    double target_yaw = atan2(jieware_pose.y - target_pose.position.y, jieware_pose.x - target_pose.position.x);
    double angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);

    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 0;

    while (fabs(angle_diff) > 0.1)
    {
        angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);
        vel_msg.angular.z = angle_diff * 2.0;
        cmd_vel_pub.publish(vel_msg);

        // if (print_counter % 5 == 0) {
        //     ROS_INFO("当前位姿: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        //     ROS_INFO("目标偏航: %.2f, 角度差: %.2f", target_yaw, angle_diff);
        // }
        ROS_INFO("turn 当前位姿: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("turn 目标偏航: %.2f, 角度差: %.2f", target_yaw, angle_diff);
        print_counter++;

        ros::spinOnce();
        rate.sleep();

        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    // 停止旋转
    vel_msg.angular.z = 0.0;
    cmd_vel_pub.publish(vel_msg);
    ROS_INFO("目标方向已到达");
}

// 计算两点之间的距离
double getDistance(geometry_msgs::Pose a, geometry_msgs::Pose b, bool print = true)
{
    double dx = b.position.x - a.position.x;
    double dy = b.position.y - a.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (print)
    {
        ROS_INFO("计算距离: %.2f", distance);
    }
    return distance;
}

// geometry_msgs::Pose（三维位姿，含位置和四元数旋转）转换为 geometry_msgs::Pose2D（二维位姿，仅平面坐标和偏航角）
geometry_msgs::Pose2D pose3DTo2D(const geometry_msgs::Pose &pose3d)
{
    geometry_msgs::Pose2D pose2d;

    // 提取平面位置 (x,y)
    pose2d.x = pose3d.position.x;
    pose2d.y = pose3d.position.y;

    // 从四元数提取偏航角 (yaw)
    tf2::Quaternion q(
        pose3d.orientation.x,
        pose3d.orientation.y,
        pose3d.orientation.z,
        pose3d.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // 获取欧拉角

    pose2d.theta = yaw; // 仅保留偏航角

    return pose2d;
}

// geometry_msgs::Pose2D（二维平面位姿，包含 x、y 坐标和偏航角 θ）转换为 geometry_msgs::Pose（三维位姿，包含位置和四元数旋转）
geometry_msgs::Pose pose2DTo3DManual(const geometry_msgs::Pose2D &pose2d)
{
    geometry_msgs::Pose pose3d;
    pose3d.position.x = pose2d.x;
    pose3d.position.y = pose2d.y;
    pose3d.position.z = 0.0;

    // 直接计算四元数（绕Z轴旋转）
    pose3d.orientation.w = cos(pose2d.theta / 2.0);
    pose3d.orientation.x = 0.0;
    pose3d.orientation.y = 0.0;
    pose3d.orientation.z = sin(pose2d.theta / 2.0);

    return pose3d;
}

// 计算两点之间的距离
double getDistance(geometry_msgs::Pose2D a, geometry_msgs::Pose b, bool print = true)
{
    double dx = b.position.x - a.x;
    double dy = b.position.y - a.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (print)
    {
        ROS_INFO("计算的距离: %.2f", distance);
    }
    return distance;
}

// 计算导航目标点的位姿
geometry_msgs::Pose calculateTargetPose(geometry_msgs::Pose charging_station_pose)
{

    geometry_msgs::Pose2D pose2d = pose3DTo2D(charging_station_pose);
    pose2d.x = pose2d.x + 0.5;
    geometry_msgs::Pose target_pose = pose2DTo3DManual(pose2d);

    return target_pose;
}

// 世界机器坐标转换到充电座坐标系下
/*
point:充电座坐标
robot：机器人坐标
*/
geometry_msgs::Pose2D WCoor2RCoor(geometry_msgs::Pose charge_pose, geometry_msgs::Pose2D robot)
{
    geometry_msgs::Pose2D rcoor;

    tf::Quaternion q(charge_pose.orientation.x, charge_pose.orientation.y, charge_pose.orientation.z, charge_pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw = normalizeAngle(yaw);

    double dis_x = robot.x - charge_pose.position.x;
    double dis_y = robot.y - charge_pose.position.y;

    rcoor.x = dis_x * cos(yaw) - dis_y * sin(yaw);
    rcoor.y = dis_x * sin(yaw) + dis_y * cos(yaw);
    return rcoor;
}

// 移动到目标点的函数，理论上此时已经朝向目标点
void moveToTarget(geometry_msgs::Pose target_pose)
{
    ROS_INFO("移动到目标点");
    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 0;
    double previous_distance = getDistance(jieware_pose, target_pose, false);

    while (ros::ok() && getDistance(jieware_pose, target_pose, false) > 0.3)
    {
        double distance = getDistance(jieware_pose, target_pose);
        double linear_speed = -0.1;

        vel_msg.linear.x = linear_speed;
        cmd_vel_pub.publish(vel_msg);

        // if (print_counter % 5 == 0) {
        //     ROS_INFO("当前位姿: x=%.2f, y=%.2f", jieware_pose.x, jieware_pose.y);
        //     ROS_INFO("目标位姿: x=%.2f, y=%.2f, 距离: %.2f", target_pose.position.x, target_pose.position.y, distance);
        //     ROS_INFO("当前线速度: %.2f", linear_speed); // 打印当前移动速度
        // }
        ROS_INFO("move 当前位姿: x=%.2f, y=%.2f", jieware_pose.x, jieware_pose.y);
        ROS_INFO("move 目标位姿: x=%.2f, y=%.2f, 距离: %.2f", target_pose.position.x, target_pose.position.y, distance);
        ROS_INFO("move 当前线速度: %.2f", linear_speed); // 打印当前移动速度
        print_counter++;

        // 检测距离是否在逐渐增大
        // if (distance > previous_distance)
        // {
        //     ROS_INFO("距离在增大，重新对齐目标点");
        //     turnToTarget(target_pose);
        //     moveToTarget(target_pose);
        //     return;
        // }
        // previous_distance = distance;

        ros::spinOnce();
        rate.sleep();
    }

    // 停止机器人
    vel_msg.linear.x = 0.0;
    cmd_vel_pub.publish(vel_msg);
    ROS_INFO("目标位置已到达");
}

// 前进移动到目标点的函数
void moveForWardRecharge(geometry_msgs::Pose target_pose, geometry_msgs::Pose charging_station_pose)
{
    ROS_INFO("forward移动到目标点");
    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 0;

    // 计算前进角度
    double target_yaw = atan2(target_pose.position.y - jieware_pose.y, target_pose.position.x - jieware_pose.x);
    double angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);

    while (fabs(angle_diff) > 0.1)
    {
        angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);
        vel_msg.angular.z = angle_diff * 2.0;
        cmd_vel_pub.publish(vel_msg);

        // if (print_counter % 5 == 0) {
        //     ROS_INFO("当前位姿: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        //     ROS_INFO("目标偏航: %.2f, 角度差: %.2f", target_yaw, angle_diff);
        // }
        ROS_INFO("turn forward当前位姿: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("turn forward 目标偏航: %.2f, 角度差: %.2f", target_yaw, angle_diff);
        print_counter++;

        ros::spinOnce();
        rate.sleep();

        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    // 计算机器人在目标点坐标系下的位姿
    geometry_msgs::Pose2D pose_in_target;
    pose_in_target = WCoor2RCoor(target_pose, jieware_pose);

    // 移动到导航目标点前方0.3m处开始对接
    while (ros::ok() && pose_in_target.x < 0.3)
    {
        double linear_speed = 0.1;

        vel_msg.linear.x = linear_speed;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("move forward 当前位姿: x=%.2f, y=%.2f", jieware_pose.x, jieware_pose.y);
        ROS_INFO("move forward 目标位姿: x=%.2f, y=%.2f", target_pose.position.x, target_pose.position.y);
        ROS_INFO("move forward 当前线速度: %.2f", linear_speed); // 打印当前移动速度

        ros::spinOnce();
        rate.sleep();
    }
}

// 对接充电桩的函数
bool alignWithChargingStation(geometry_msgs::Pose target_pose, geometry_msgs::Pose charging_station_pose)
{
    bool align_charge_success = false;
    ROS_INFO("准备对齐充电座");

    // 获取当前机器人的朝向（四元数转欧拉角）
    // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    // double roll, pitch, yaw;
    // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 获取目标充电站的朝向（四元数转欧拉角）
    tf::Quaternion target_q(charging_station_pose.orientation.x, charging_station_pose.orientation.y, charging_station_pose.orientation.z,
                            charging_station_pose.orientation.w);
    double target_roll, target_pitch, target_yaw;
    tf::Matrix3x3(target_q).getRPY(target_roll, target_pitch, target_yaw);

    double angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);

    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 0;

    // 先旋转对齐
    while (fabs(angle_diff) > 0.1)
    {
        angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);
        double angular_speed = angle_diff * 1.5;
        // 设置旋转速度
        vel_msg.angular.z = angular_speed;
        cmd_vel_pub.publish(vel_msg);

        // if (print_counter % 5 == 0) {
        //     ROS_INFO("align 当前位姿: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        //     ROS_INFO("align 目标偏航: %.2f, 角度差: %.2f", target_yaw, angle_diff);
        // }
        ROS_INFO("align rotate 当前位姿: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("align rotate 目标偏航: %.2f, 角度差: %.2f", target_yaw, angle_diff);
        print_counter++;

        ros::spinOnce();
        rate.sleep();

        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    // 停止旋转
    vel_msg.angular.z = 0.0;
    cmd_vel_pub.publish(vel_msg);
    ROS_INFO("充电站方向已对齐");

    geometry_msgs::Pose2D pose_in_charge_coordination;
    pose_in_charge_coordination = WCoor2RCoor(charging_station_pose, jieware_pose);
    double charge_distance = getDistance(jieware_pose, charging_station_pose, false);

    // 如果机器人在充电座的坐标系前面，发布后退速度
    while (ros::ok() && pose_in_charge_coordination.x >= 0)
    {
        charge_distance = getDistance(jieware_pose, charging_station_pose, false);
        pose_in_charge_coordination = WCoor2RCoor(charging_station_pose, jieware_pose);
        ROS_INFO("align move 机器人位姿: x=%.2f, y=%.2f ", pose_in_charge_coordination.x, pose_in_charge_coordination.y);

        double linear_speed = -0.1;

        vel_msg.linear.x = linear_speed;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("align move 当前位姿: x=%.2f, y=%.2f ", jieware_pose.x, jieware_pose.y);
        ROS_INFO("align move 目标位姿: x=%.2f, y=%.2f ", charging_station_pose.position.x, charging_station_pose.position.y);
        ROS_INFO("align move 当前线速度: %.2f", linear_speed); // 打印当前移动速度

        ros::spinOnce();
        rate.sleep();

        // 在移动过程中持续对齐充电座
        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);
        // 边旋转边对齐，迭代
        if (fabs(angle_diff) > 0.1)
        {
            ROS_INFO("移动过程中重新对齐充电站");
            alignWithChargingStation(target_pose, charging_station_pose);
        }
        // 机器人充上电，停止机器人
        if (charge_distance < 0.1)
        {
            vel_msg.linear.x = 0.0;
            cmd_vel_pub.publish(vel_msg);
            ROS_INFO("充电站已到达");
            align_charge_success = true;
            break;
        }
    }
    return align_charge_success;
}

bool reCharge(geometry_msgs::Pose target_pose, geometry_msgs::Pose charging_station_pose)
{
    bool reCharge_success = false;
    // Step 1: 转向导航目标点
    turnToTarget(target_pose);

    // Step 2: 移动到导航目标位置
    moveToTarget(target_pose);

    // Step 3: 转向充电座朝向并对齐充电桩
    reCharge_success = alignWithChargingStation(target_pose, charging_station_pose);
    return reCharge_success;
}

// 服务回调函数
bool goToChargingStationCallback(charging_station::GoToChargingStation::Request &req,
                                 charging_station::GoToChargingStation::Response &res)
{
    ROS_INFO("收到服务请求，前往充电站");
    ros::Rate rate(10); // 设置循环频率为每秒10次
    bool charge_success = false;
    int charge_count = 0;
    res.success = false;
    // 获取充电桩的位姿
    charging_station::ImageRectDetector detector;
    geometry_msgs::Pose charging_station_pose = detector.getChargingStationPoseStatic();

    // 打印充电桩的位姿
    ROS_INFO("充电桩位姿: x=%.2f, y=%.2f, z=%.2f",
             charging_station_pose.position.x,
             charging_station_pose.position.y,
             charging_station_pose.position.z);

    // 计算目标点的位姿
    geometry_msgs::Pose target_pose = calculateTargetPose(charging_station_pose);

    // 打印目标点的位姿
    ROS_INFO("目标点位姿: x=%.2f, y=%.2f, z=%.2f",
             target_pose.position.x,
             target_pose.position.y,
             target_pose.position.z);

    while (ros::ok() && charge_count < 3)
    {
        charge_success = reCharge(target_pose, charging_station_pose);
        if (charge_success)
        {
            res.success = true;
            break;
        }
        else
        {
            // 如果机器人退到了充电座后面
            moveForWardRecharge(target_pose, charging_station_pose);
        }
        charge_count++;
        ros::spinOnce();
        rate.sleep();
    }
    return charge_success;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "charging_station_service_server");
    ros::NodeHandle nh;

    // ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber pose_sub = nh.subscribe("/pose_data", 10, poseCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = nh.advertiseService("go_to_charging_station", goToChargingStationCallback);
    ROS_INFO("准备前往充电站");

    ros::spin();
    return 0;
}