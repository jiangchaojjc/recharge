#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>  
  
 
   

int main(int argc, char** argv)
{
    //设置编码
    setlocale(LC_ALL,"");
    // 初始化ROS节点
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // 初始化图像传输对象，用于发布图像
    image_transport::ImageTransport it(nh);
    // 创建图像发布者
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    // 读取图像文件
    cv::Mat image = cv::imread("/home/lqc/7.12image/3/left.png", cv::IMREAD_COLOR);
    // 检查图像是否加载成功
    if (image.empty()) {
        ROS_ERROR("无法加载图像");
        return -1;
    }

    // 设置循环频率为5Hz
    ros::Rate loop_rate(5); 

    // 当ROS节点状态正常时，循环发布图像
    while (nh.ok()) {
        // 将OpenCV图像转换为ROS图像消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        // 发布图像消息
        pub.publish(msg);
        // 处理ROS消息
        ros::spinOnce();
        // 等待下一循环
        loop_rate.sleep();
    }

    return 0;
}
