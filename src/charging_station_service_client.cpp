// 可以通过该命令来启动服务：rosservice call /go_to_charging_station
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
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool readQRcodePicture()
{
    // 保存图片
    const char *ps8Cmd = "curl -s -X GET 'http://127.0.0.1/WAPI/V1.0/Event/Snapshot?channelName=bottom&snapType=1&streamType=0&imagePath=/data/ld_ros/Qrcode.png'";
    FILE *pipe = popen(ps8Cmd, "r");
    if (!pipe)
    {
        std::cerr << "popen failed !!!" << std::endl;
        return -1;
    }
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe))
    {
        std::cout << buffer;
    }
    pclose(pipe);
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

move_base_msgs::MoveBaseGoal getNaviRechargeGoal()
{
    geometry_msgs::Pose2D pose_2d;
    pose_2d.x = 0.23;
    pose_2d.y = 0.0;
    pose_2d.theta = -3.12;
    geometry_msgs::Pose pose3d = pose2DTo3DManual(pose_2d);
    std::cout << "导航到目标点 : " << pose_2d.x << "  " << pose_2d.y << "  " << pose3d.position.x << "  " << pose3d.position.y << std::endl;
    move_base_msgs::MoveBaseGoal goal;

    // 设置目标坐标系（必须与地图一致！）
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置目标位置 (x=1.0, y=0.0)
    goal.target_pose.pose = pose3d;
    return goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "charging_station_service_client");
    ros::NodeHandle nh;

    // ========== 第二步：执行 Action ==========
    MoveBaseClient nav_client("move_base", true);
    ROS_INFO("Waiting for move_base server...");
    nav_client.waitForServer(); // 等待 Action 服务器
    move_base_msgs::MoveBaseGoal goal = getNaviRechargeGoal();
    ROS_INFO("Sending navigation goal...");
    nav_client.sendGoal(goal);

    // 阻塞式等待 Action 完成
    bool finished = nav_client.waitForResult();

    if (nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigation succeeded!");
    }
    else
    {
        ROS_ERROR("Navigation failed!");
    }
    // 等待2秒，防止机器人没停稳，图片模糊
    ros::Duration(2.0).sleep();
    // 摄像头获取实时二维码图片
    readQRcodePicture();
    ros::Duration(1.0).sleep();
    // 获取充电桩的位姿
    QRcodePoseTemp qr_pose_in_robot_test;
    QRcodePoseTemp camera_in_qr_code;
    charging_station::ImageRectDetector detector_test;
    bool detect = detector_test.QRcodeDectectPnp(qr_pose_in_robot_test, camera_in_qr_code);

    // 调用定点回充服务
    ros::ServiceClient client = nh.serviceClient<charging_station::GoToChargingStation>("go_to_charging_station");

    charging_station::GoToChargingStation srv;

    if (client.call(srv))
    {
        ROS_INFO("Service call succeeded. Success: %s", srv.response.success ? "true" : "false");
    }
    else
    {
        ROS_ERROR("Failed to call service go_to_charging_station");
        return 1;
    }

    return 0;
}
