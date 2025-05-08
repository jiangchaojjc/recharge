#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "charging_station_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到“充电桩坐标信息”话题
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("Charging_station_position", 1);

    // 设置发布频率
    ros::Rate rate(10); // 10 Hz

    // 创建一个Pose消息
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    // 设置方位角（yaw）为3.14弧度
    double yaw = 3.14;
    tf::Quaternion q;
    q = tf::createQuaternionFromYaw(yaw);
    tf::quaternionTFToMsg(q, pose.orientation);

    while (ros::ok()) {
        // 发布Pose消息
        pub.publish(pose);

        // 处理所有回调函数
        ros::spinOnce();

        // 按照设定的频率休眠
        rate.sleep();
    }

    return 0;
}