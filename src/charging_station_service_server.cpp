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
#include <robot_sensors/Charge.h>

geometry_msgs::Pose current_pose;
geometry_msgs::Pose2D jieware_pose;
robot_sensors::Charge charge_info;
QRcodePoseTemp qr_pose_in_robot;

ros::Publisher cmd_vel_pub;
geometry_msgs::Pose2D RCoor2WCoor(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D robot);
geometry_msgs::Pose2D WCoor2RCoor(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D robot);
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

// 回调函数：获取机器人定位的当前位姿
void chargeCallback(const robot_sensors::Charge &msg)
{
    charge_info = msg;
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

// 角度归一化函数
double restrictAngleVel(double angle)
{
    if (angle > 0.3)
    {
        angle = 0.3;
    }
    if (angle < -0.3)
    {
        angle = -0.3;
    }
    return angle;
}

// 计算两点之间的距离
double getDistance(geometry_msgs::Pose a, geometry_msgs::Pose b, bool print = true)
{
    double dx = b.position.x - a.position.x;
    double dy = b.position.y - a.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (print)
    {
        ROS_INFO("compute distance: %.2f", distance);
    }
    return distance;
}

geometry_msgs::Pose2D getChargePoseInWorld(const QRcodePoseTemp &qrcodePose)
{
    geometry_msgs::Pose2D charge_pose_in_world;
    geometry_msgs::Pose2D charge_pose_in_robot;

    charge_pose_in_robot.x = qrcodePose.z + 0.25 + 0.08;
    charge_pose_in_robot.y = -qrcodePose.x;
    ROS_INFO("charge_pose_local : %.2f  %.2f", charge_pose_in_robot.x, charge_pose_in_robot.y);
    charge_pose_in_robot.theta = normalizeAngle(jieware_pose.theta - (qrcodePose.pitch / 180.00 * M_PI) + M_PI);
    charge_pose_in_world = RCoor2WCoor(charge_pose_in_robot, jieware_pose);
    charge_pose_in_world.theta = charge_pose_in_robot.theta;
    ROS_INFO("charge_pose_in_world : %.2f  %.2f  %.2f ", charge_pose_in_world.x, charge_pose_in_world.y, charge_pose_in_world.theta);
    ROS_INFO("robot_pose : %.2f  %.2f  %.2f ", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
    return charge_pose_in_world;
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
double getDistance(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b, bool print = true)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (print)
    {
        ROS_INFO("compute distance: %.2f", distance);
    }
    return distance;
}

// 计算导航目标点的位姿
geometry_msgs::Pose2D calculateTargetPose(geometry_msgs::Pose2D pose_charge)
{
    geometry_msgs::Pose2D target_in_charge_coordination;
    target_in_charge_coordination.x = 0.5;
    target_in_charge_coordination.y = 0.0;
    target_in_charge_coordination.theta = 0.0;
    geometry_msgs::Pose2D target_in_world = RCoor2WCoor(target_in_charge_coordination, pose_charge);
    ROS_INFO("target pose2d in world: : %.2f  %.2f  %.2f ", target_in_world.x, target_in_world.y, target_in_world.theta);
    ROS_INFO("charge pose2d in world: %.2f  %.2f  %.2f ", pose_charge.x, pose_charge.y, pose_charge.theta);
    return target_in_world;
}

// 机器人坐标系转换到世界坐标下
/*
pose:  机器人下的坐标
robot：机器人坐标
*/
geometry_msgs::Pose2D RCoor2WCoor(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D robot)
{
    ROS_INFO("RCoor2WCoor pose : %.2f  %.2f  ", pose.x, pose.y);
    ROS_INFO("RCoor2WCoor robot : %.2f  %.2f  %.2f ", robot.x, robot.y, robot.theta);
    geometry_msgs::Pose2D wcoor;
    double theta = atan2(pose.y, pose.x);
    double dis = sqrt(pose.y * pose.y + pose.x * pose.x);
    ROS_INFO("RCoor2WCoor theta %.2f  dis  %.2f normalizeAngle(theta + robot.theta)= %.2f", theta, dis, normalizeAngle(theta + robot.theta));
    wcoor.x = robot.x + dis * cos(normalizeAngle(theta + robot.theta));
    wcoor.y = robot.y + dis * sin(normalizeAngle(theta + robot.theta));
    wcoor.theta = normalizeAngle(theta + robot.theta);
    return wcoor;
}

// 世界坐标转换到机器人坐标系下
/*
pose:  世界坐标坐标
robot：机器人坐标
*/
geometry_msgs::Pose2D WCoor2RCoor(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D robot)
{
    geometry_msgs::Pose2D rcoor;

    float dis_x = pose.x - robot.x;
    float dis_y = pose.y - robot.y;

    rcoor.x = dis_x * cos(robot.theta) + dis_y * sin(robot.theta);
    rcoor.y = -dis_x * sin(robot.theta) + dis_y * cos(robot.theta);
    rcoor.theta = normalizeAngle(pose.theta);
    return rcoor;
}

// 转向目标点的函数
void turnToTarget(geometry_msgs::Pose2D target_pose)
{
    ROS_INFO("rotate to target point");

    double target_yaw = atan2(jieware_pose.y - target_pose.y, jieware_pose.x - target_pose.x);
    double angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);

    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 0;
    double angle_vel = 0.0;

    while (fabs(angle_diff) > 0.1)
    {
        angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);

        angle_vel = restrictAngleVel(angle_diff * 2.0);
        vel_msg.angular.z = angle_vel;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("rotate robot pose: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("rotate target pose: x=%.2f, y=%.2f ", target_pose.x, target_pose.y);
        ROS_INFO("rotate target direction: %.2f, gap angle: %.2f", target_yaw, angle_diff);
        print_counter++;

        ros::spinOnce();
        rate.sleep();

        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    // 停止旋转
    vel_msg.angular.z = 0.0;
    cmd_vel_pub.publish(vel_msg);
    ROS_INFO("target direction achieve");
}

// 移动到目标点的函数，理论上此时已经朝向目标点
void moveToTarget(geometry_msgs::Pose2D target_pose)
{
    ROS_INFO("move to target !");
    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 1;
    double angle_diff = 0.0;
    double target_yaw = 0.0;
    double previous_distance = getDistance(jieware_pose, target_pose, false);
    double angle_vel = 0.0;

    geometry_msgs::Pose2D robot_pose_in_target_coordinate = WCoor2RCoor(target_pose, jieware_pose);

    // 如果机器人在目标点与充电点之前，先前进至目标点,判断目标点在机器人坐标系下y轴是否>7cm

    while (ros::ok() && (getDistance(jieware_pose, target_pose, false) > 0.2 || fabs(robot_pose_in_target_coordinate.y) > 0.07))
    {
        double distance = getDistance(jieware_pose, target_pose);
        robot_pose_in_target_coordinate = WCoor2RCoor(target_pose, jieware_pose);
        // 检测目标的角度是否变化
        if (print_counter % 5 == 0)
        {
            target_yaw = atan2(jieware_pose.y - target_pose.y, jieware_pose.x - target_pose.x);
            angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);
            ROS_INFO("move angle_diff: %.2f", angle_diff);
        }
        angle_vel = restrictAngleVel(angle_diff);
        vel_msg.linear.x = -0.1;
        vel_msg.angular.z = angle_vel;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("move robot pose: x=%.2f, y=%.2f", jieware_pose.x, jieware_pose.y);
        ROS_INFO("move target pose: x=%.2f, y=%.2f, distance: %.2f y distance :%.2f", target_pose.x, target_pose.y, distance, robot_pose_in_target_coordinate.y);
        ROS_INFO("move now line speed: %.2f angle speed: %.2f  ", vel_msg.linear.x, angle_vel); // 打印当前移动速度
        print_counter++;

        // 开过头的情况(一定周期检测距离是否在逐渐增大)
        if (print_counter % 18 == 0)
        {
            previous_distance = distance;
            if (distance > previous_distance)
            {
                ROS_INFO("move away from target ,restart to rotate to target");
                turnToTarget(target_pose);
                moveToTarget(target_pose);
                return;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 停止机器人
    vel_msg.linear.x = 0.0;
    cmd_vel_pub.publish(vel_msg);
    ROS_INFO("arrive at target pose");
}

// 前进移动到目标点的函数
void moveForWardRecharge(geometry_msgs::Pose2D target_pose, geometry_msgs::Pose2D charging_station_pose)
{
    ROS_INFO("forward move to target");
    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;
    int print_counter = 0;
    double angle_vel = 0.0;

    // 计算前进角度
    double target_yaw = atan2(target_pose.y - jieware_pose.y, target_pose.x - jieware_pose.x);
    double angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);

    while (fabs(angle_diff) > 0.1)
    {
        angle_diff = normalizeAngle(target_yaw - jieware_pose.theta);
        angle_vel = restrictAngleVel(angle_diff * 2);
        vel_msg.angular.z = angle_vel;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("forward rotate robot pose: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("forward rotate target pose: x=%.2f, y=%.2f ", target_pose.x, target_pose.y);
        ROS_INFO("forward rotate gap yaw: %.2f, angle speed: %.2f angle_diff %.2f ", target_yaw, angle_vel, angle_diff);
        print_counter++;

        ros::spinOnce();
        rate.sleep();

        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    // 计算机器人在目标点坐标系下的位姿
    geometry_msgs::Pose2D pose_in_target_coordinate;
    pose_in_target_coordinate = WCoor2RCoor(target_pose, jieware_pose);

    // 移动到导航目标点前方处开始对接
    while (ros::ok() && pose_in_target_coordinate.x > 0)
    {
        double linear_speed = 0.1;
        pose_in_target_coordinate = WCoor2RCoor(target_pose, jieware_pose);
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = 0.0;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("move forward robot pose: x=%.2f, y=%.2f theta=%.2f ", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("move forward target: x=%.2f, y=%.2f", target_pose.x, target_pose.y);
        ROS_INFO("move forward line speed: %.2f pose_in_target_coordinate.x %.2f", linear_speed, pose_in_target_coordinate.x); // 打印当前移动速度和相对位姿

        ros::spinOnce();
        rate.sleep();
    }
}

// 对接充电桩的函数
bool alignWithChargingStation(geometry_msgs::Pose2D target_pose, geometry_msgs::Pose2D charging_station_pose)
{
    bool align_charge_success = false;
    ROS_INFO("prepare to align charge point");

    double angle_diff = normalizeAngle(charging_station_pose.theta - jieware_pose.theta);

    ros::Rate rate(10); // 设置循环频率为每秒10次
    geometry_msgs::Twist vel_msg;

    double angular_speed = 0.0;
    // 先旋转对齐
    while (fabs(angle_diff) > 0.07)
    {
        angle_diff = normalizeAngle(charging_station_pose.theta - jieware_pose.theta);
        angular_speed = angle_diff * 1.5;
        angular_speed = restrictAngleVel(angular_speed);
        // 设置旋转速度
        vel_msg.angular.z = angular_speed;
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("align rotate robot pose: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("align rotate charge pose: x=%.2f, y=%.2f, yaw=%.2f , angle speed: %.2f angle_diff %.2f ",
                 charging_station_pose.x, charging_station_pose.y, charging_station_pose.theta, angular_speed, angle_diff);

        ros::spinOnce();
        rate.sleep();

        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    // 停止旋转
    vel_msg.angular.z = 0.0;
    cmd_vel_pub.publish(vel_msg);
    ROS_INFO("charge align ok");
    ROS_INFO("prepare to align back ...");
    geometry_msgs::Pose2D pose_in_charge_coordination;
    ROS_INFO("robot pose: x=%.2f, y=%.2f, yaw=%.2f", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
    ROS_INFO("charge pose:  x=%.2f, y=%.2f ", charging_station_pose.x, charging_station_pose.y);
    pose_in_charge_coordination = WCoor2RCoor(charging_station_pose, jieware_pose);
    ROS_INFO("align back charge in coordinate robot pose  x=%.2f, y=%.2f ", pose_in_charge_coordination.x, pose_in_charge_coordination.y);
    double charge_distance = getDistance(jieware_pose, charging_station_pose, false);
    // 后退的过程中计算和充电点的角度差
    // target_yaw = atan2(jieware_pose.y - charging_station_pose.position.y, jieware_pose.x - charging_station_pose.position.x);
    angle_diff = normalizeAngle(charging_station_pose.theta - jieware_pose.theta);
    double linear_speed = -0.1;
    // 如果机器人推到充电座后面了，停止发布后退速度
    int print_counter = 0;
    double angle_vel = 0.0;

    while (ros::ok() && pose_in_charge_coordination.x < 0)
    {
        charge_distance = getDistance(jieware_pose, charging_station_pose, false);
        pose_in_charge_coordination = WCoor2RCoor(charging_station_pose, jieware_pose);
        ROS_INFO("align back charge in coordinate robot pose x=%.2f, y=%.2f ", pose_in_charge_coordination.x, pose_in_charge_coordination.y);

        // 后退的过程中计算和充电点的角度差
        // 检测目标的角度是否变化
        if (print_counter % 5 == 0)
        {
            // target_yaw = atan2(jieware_pose.y - charging_station_pose.position.y, jieware_pose.x - charging_station_pose.position.x);
            angle_diff = normalizeAngle(charging_station_pose.theta - jieware_pose.theta);
            ROS_INFO("move angle_diff: %.2f", angle_diff);
        }
        print_counter++;
        angle_vel = restrictAngleVel(angle_diff);
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = angle_vel;

        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("align back now pose: x=%.2f, y=%.2f theta=%.2f ", jieware_pose.x, jieware_pose.y, jieware_pose.theta);
        ROS_INFO("align back target pose: x=%.2f, y=%.2f ", charging_station_pose.x, charging_station_pose.y);
        ROS_INFO("align back line speed: %.2f angle speed: %.2f", linear_speed, angle_vel); // 打印当前移动速度

        ros::spinOnce();
        rate.sleep();

        // 在移动过程中持续对齐充电座
        // tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double angle_diff = normalizeAngle(charging_station_pose.theta - jieware_pose.theta);
        // // 边旋转边对齐，迭代
        // if (fabs(angle_diff) > 0.1)
        // {
        //     ROS_INFO("移动过程中重新对齐充电站");
        //     alignWithChargingStation(target_pose, charging_station_pose);
        // }
        // 机器人充上电,用临时位置判断，停止机器人
        // if (charge_distance < 0.07)
        // {
        //     vel_msg.linear.x = 0.0;
        //     cmd_vel_pub.publish(vel_msg);
        //     ROS_INFO("充电站已到达");
        //     align_charge_success = true;
        //     break;
        // }

        // 机器人充上电，停止机器人
        if (charge_info.battery_status)
        {
            vel_msg.linear.x = 0.0;
            cmd_vel_pub.publish(vel_msg);
            ROS_INFO("arrive at charge target");
            align_charge_success = true;
            break;
        }
    }
    return align_charge_success;
}

bool reCharge(geometry_msgs::Pose2D target_pose, geometry_msgs::Pose2D charging_station_pose)
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
    ROS_INFO("achieve server request ,back charge start !");
    ros::Rate rate(10); // 设置循环频率为每秒10次

    bool charge_success = false;
    bool detect = false;
    int charge_count = 0;
    res.success = false;
    // 获取充电桩的位姿
    charging_station::ImageRectDetector detector;

    detect = detector.QRcodeDectectPnp(qr_pose_in_robot);
    if (!detect)
    {
        ROS_INFO("cannot detected qrcode ,back charge stop !");
        return charge_success;
    }
    geometry_msgs::Pose2D charge_pose2d = getChargePoseInWorld(qr_pose_in_robot);

    // 打印充电桩的位姿
    ROS_INFO("charge point: x=%.2f, y=%.2f, theta=%.2f",
             charge_pose2d.x,
             charge_pose2d.y,
             charge_pose2d.theta);

    // 计算目标点的位姿
    geometry_msgs::Pose2D target_pose2d = calculateTargetPose(charge_pose2d);

    // 打印目标点的位姿
    ROS_INFO("target point: x=%.2f, y=%.2f, theta=%.2f",
             target_pose2d.x,
             target_pose2d.y,
             target_pose2d.theta);
    // 二维码与充电座朝向偏差小于5度
    if (fabs(qr_pose_in_robot.pitch) < 5)
    {
        // Step 3: 转向充电座朝向并对齐充电桩
        charge_success = alignWithChargingStation(target_pose2d, charge_pose2d);
    }

    // 没有对接成功，且对接小于3次
    while (ros::ok() && !charge_success && charge_count < 3)
    {

        charge_success = reCharge(target_pose2d, charge_pose2d);
        if (charge_success)
        {
            res.success = true;
            break;
        }
        else
        {
            // 如果机器人退到了充电座后面
            moveForWardRecharge(target_pose2d, charge_pose2d);
        }
        charge_count++;
        ros::spinOnce();
        rate.sleep();
    }
    if (charge_success)
    {
        res.success = true;
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
    ros::Subscriber charge_sub = nh.subscribe("/charge", 10, chargeCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = nh.advertiseService("go_to_charging_station", goToChargingStationCallback);
    ROS_INFO("prepare to back charge");

    ros::spin();
    return 0;
}