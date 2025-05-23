#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <image_transport/image_transport.h>
#include <numeric>

class ImageRectDetector
{
public:
    ImageRectDetector() : it_(nh_)
    {
        // image_sub_ = it_.subscribe("/xv_sdk/xv_dev/fisheye_cameras/left/image", 1, &ImageRectDetector::imageCallback, this);
        image_sub_ = it_.subscribe("camera/image", 1, &ImageRectDetector::imageCallback, this);

        depth_sub_ = it_.subscribe("/xv_sdk/xv_dev/fisheye_cameras/left/depth", 1, &ImageRectDetector::depthCallback, this);
        center_pub_ = nh_.advertise<geometry_msgs::Point>("/detected_rectangle_center", 10);
        normal_pub_ = nh_.advertise<geometry_msgs::Vector3>("/detected_rectangle_normal", 10);
        fx_ = 500;
        fy_ = 500;
        cx_ = 320;
        cy_ = 240;
    }

    /**
     * 图像回调函数，用于处理接收到的图像消息
     * @param msg 接收到的图像消息指针
     */
    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            ROS_INFO("Received image message");
            // 将ROS图像消息转换为OpenCV图像格式，并创建一个新的cv_bridge对象
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // 获取转换后的图像
            cv::Mat image = cv_ptr->image;
            ROS_INFO("Image converted to OpenCV format");

            // 处理图像并返回需要绘制的矩形信息
            std::vector<std::tuple<cv::Point, int, int>> rectangles_to_draw = processImage(image);
            ROS_INFO("Image processed, %zu rectangles detected", rectangles_to_draw.size());

            // 根据矩形数量执行不同操作
            if (rectangles_to_draw.size() == 1)
            {
                // 获取唯一矩形的中心点、宽度和高度
                auto [center, width, height] = rectangles_to_draw[0];
                ROS_INFO("Single rectangle detected at (%d, %d) with width %d and height %d", center.x, center.y, width, height);

                // 计算矩形四个角点的位置
                std::vector<cv::Point> rectangle_corners = {
                    cv::Point(center.x - width / 2, center.y - height / 2),
                    cv::Point(center.x + width / 2, center.y - height / 2),
                    cv::Point(center.x + width / 2, center.y + height / 2),
                    cv::Point(center.x - width / 2, center.y + height / 2)};

                // 获取与图像对应的时间戳匹配的深度图像
                cv::Mat depth_image = getDepthImage(msg->header.stamp);
                if (depth_image.empty())
                {
                    ROS_WARN("Depth image not found or too old");
                    return;
                }
                ROS_INFO("Depth image retrieved");

                // 计算矩形在深度图像上的法线向量
                cv::Vec3f normal = calculateRectangleNormal(depth_image, rectangle_corners);
                ROS_INFO("Rectangle normal calculated: (%f, %f, %f)", normal[0], normal[1], normal[2]);

                // 将法线向量转换为ROS消息格式并发布
                geometry_msgs::Vector3 normal_msg;
                normal_msg.x = normal[0];
                normal_msg.y = normal[1];
                normal_msg.z = normal[2];
                normal_pub_.publish(normal_msg);

                // 将图像中的中心点转换为相机坐标系下的3D点
                cv::Vec3f center_3d = imageToCameraCoordinates(depth_image, center);
                ROS_INFO("Center 3D point calculated: (%f, %f, %f)", center_3d[0], center_3d[1], center_3d[2]);

                // 将3D中心点转换为ROS消息格式并发布
                geometry_msgs::Point center_3d_msg;
                center_3d_msg.x = center_3d[0];
                center_3d_msg.y = center_3d[1];
                center_3d_msg.z = center_3d[2];
                center_pub_.publish(center_3d_msg);
            }
            else if (rectangles_to_draw.size() > 1)
            {
                ROS_INFO("Multiple rectangles detected. Waiting for single rectangle detection.");
            }
            else
            {
                ROS_INFO("No rectangles detected. Waiting for single rectangle detection.");
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception in imageCallback: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            ROS_INFO("Received depth image message");
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat depth_image = cv_ptr->image;
            depth_image_buffer_[msg->header.stamp] = depth_image;
            ROS_INFO("Depth image added to buffer");

            // 限制深度图像缓存大小
            const size_t MAX_DEPTH_IMAGES = 10; // 根据需要调整
            if (depth_image_buffer_.size() > MAX_DEPTH_IMAGES)
            {
                depth_image_buffer_.erase(depth_image_buffer_.begin());
                ROS_INFO("Depth image buffer size exceeded, oldest image removed");
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception in depthCallback: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher center_pub_;
    ros::Publisher normal_pub_;
    std::map<ros::Time, cv::Mat> depth_image_buffer_;
    double fx_, fy_, cx_, cy_;

    cv::Mat getDepthImage(const ros::Time &timestamp)
    {
        auto closest_timestamp = std::min_element(depth_image_buffer_.begin(), depth_image_buffer_.end(),
                                                  [&](const std::pair<ros::Time, cv::Mat> &a, const std::pair<ros::Time, cv::Mat> &b)
                                                  {
                                                      return std::abs((a.first - timestamp).toSec()) < std::abs((b.first - timestamp).toSec());
                                                  });

        if (closest_timestamp == depth_image_buffer_.end())
        {
            ROS_WARN("No depth image found for timestamp %f", timestamp.toSec());
            return cv::Mat();
        }

        if (std::abs((closest_timestamp->first - timestamp).toSec()) > 0.05)
        {
            ROS_WARN("Closest depth image timestamp %f is too far from requested timestamp %f", closest_timestamp->first.toSec(), timestamp.toSec());
            return cv::Mat();
        }

        ROS_INFO("Closest depth image timestamp %f found for requested timestamp %f", closest_timestamp->first.toSec(), timestamp.toSec());
        cv::Mat depth_image = closest_timestamp->second;
        depth_image_buffer_.erase(closest_timestamp->first);
        return depth_image;
    }

    void drawCross(cv::Mat &img, const cv::Point &center, const cv::Scalar &color = cv::Scalar(255, 0, 0), int thickness = 1, int size = 10)
    {
        cv::line(img, cv::Point(center.x - size, center.y), cv::Point(center.x + size, center.y), color, thickness);
        cv::line(img, cv::Point(center.x, center.y - size), cv::Point(center.x, center.y + size), color, thickness);
        cv::putText(img, "rect", cv::Point(center.x - 5, center.y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    cv::Mat thresholdImage(const cv::Mat &image, double threshold_value = 110, double max_value = 255, int method = cv::THRESH_BINARY)
    {
        // 创建一个空的图像矩阵，用于存储阈值处理后的结果
        cv::Mat thresh;
        // 调用OpenCV的threshold函数进行阈值处理
        cv::threshold(image, thresh, threshold_value, max_value, method);
        // 返回阈值处理后的图像矩阵
        return thresh;
    }

    double calculatePerimeter(const std::vector<cv::Point> &cnt)
    {
        return cv::arcLength(cnt, true);
    }

    bool isRectangleLike(const std::vector<cv::Point> &cnt, const std::vector<cv::Point> &approx, double min_aspect_ratio = 3.7, double max_aspect_ratio = 7.7, double min_fill_ratio = 0.5)
    {
        if (approx.size() != 4)
            return false;

        cv::Rect rect = cv::boundingRect(approx);
        double aspect_ratio = static_cast<double>(rect.width) / rect.height;
        if (!(min_aspect_ratio <= aspect_ratio && aspect_ratio <= max_aspect_ratio))
            return false;

        double area = cv::contourArea(cnt);
        double rect_area = rect.width * rect.height;
        double fill_ratio = area / rect_area;

        if (fill_ratio < min_fill_ratio)
            return false;

        return true;
    }

    std::vector<std::tuple<cv::Point, int, int>> filterRegions(const cv::Mat &image, int min_area, int max_area)
    {
        std::vector<std::tuple<cv::Point, int, int>> rectangles_info;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &cnt : contours)
        {
            double area = cv::contourArea(cnt);
            double perimeter = calculatePerimeter(cnt);

            if (max_area >= area && area >= min_area)
            {
                std::vector<cv::Point> approx;
                cv::approxPolyDP(cnt, approx, 0.04 * perimeter, true);
                if (isRectangleLike(cnt, approx))
                {
                    cv::Rect rect = cv::boundingRect(approx);
                    cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
                    rectangles_info.emplace_back(center, rect.width, rect.height);
                }
            }
        }
        return rectangles_info;
    }

    std::vector<std::tuple<cv::Point, int, int>> processImage(cv::Mat &image)
    {
        // 将图像转换为灰度图像，减少颜色信息，增强处理效果
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        // 应用二值化反转阈值处理，以便更好地识别目标对象
        cv::Mat thresh = thresholdImage(gray_image, 110, 255, cv::THRESH_BINARY_INV);

        // 显示二值化后的图像
        cv::imshow("Thresholded Image", thresh);
        cv::waitKey(1);

        // 过滤得到满足面积条件的矩形区域
        auto rectangles_to_draw = filterRegions(thresh, 50, 2000);

        // 遍历所有检测到的矩形区域，绘制矩形框和十字标记
        for (const auto &[center, width, height] : rectangles_to_draw)
        {
            // 绘制矩形框，用绿色标记检测到的区域
            cv::rectangle(image, cv::Point(center.x - width / 2, center.y - height / 2),
                          cv::Point(center.x + width / 2, center.y + height / 2), cv::Scalar(0, 255, 0), 2);
            // 在区域中心绘制蓝色十字标记，用于精确定位
            drawCross(image, center, cv::Scalar(255, 0, 0), 2, 10);
        }

        // 如果没有检测到任何矩形区域，输出提示信息
        if (rectangles_to_draw.empty())
        {
            ROS_INFO("No rectangles detected.");
        }

        // 显示处理后的图像，用于可视化检查
        cv::imshow("Processed Image", image);
        // 等待按键，用于实时显示图像，1毫秒是为了保持程序的流畅
        cv::waitKey(1);

        // 返回检测到的矩形区域信息
        return rectangles_to_draw;
    }

    cv::Vec3f calculateRectangleNormal(const cv::Mat &depth_image, const std::vector<cv::Point> &rectangle_corners)
    {
        std::vector<float> depth_points;
        for (const auto &corner : rectangle_corners)
        {
            if (corner.y >= 0 && corner.y < depth_image.rows && corner.x >= 0 && corner.x < depth_image.cols)
            {
                depth_points.push_back(depth_image.at<float>(corner.y, corner.x));
            }
            else
            {
                ROS_WARN("Corner (%d, %d) out of depth image bounds", corner.x, corner.y);
                depth_points.push_back(0.0f);
            }
        }

        std::vector<cv::Vec3f> points_3d;
        for (size_t i = 0; i < rectangle_corners.size(); ++i)
        {
            float Z = depth_points[i];
            float X = (rectangle_corners[i].x - cx_) * Z / fx_;
            float Y = (rectangle_corners[i].y - cy_) * Z / fy_;
            points_3d.emplace_back(X, Y, Z);
        }

        if (points_3d.empty())
        {
            ROS_WARN("No 3D points calculated for rectangle corners");
            return cv::Vec3f(0, 0, 0); // 或者其他适当的默认值
        }

        cv::Vec3f centroid = std::accumulate(points_3d.begin(), points_3d.end(), cv::Vec3f(0, 0, 0));
        centroid = centroid / static_cast<float>(points_3d.size()); // 转换为 float 类型

        cv::Mat points_centered(points_3d.size(), 3, CV_32F);
        for (size_t i = 0; i < points_3d.size(); ++i)
        {
            points_centered.at<float>(i, 0) = points_3d[i][0] - centroid[0];
            points_centered.at<float>(i, 1) = points_3d[i][1] - centroid[1];
            points_centered.at<float>(i, 2) = points_3d[i][2] - centroid[2];
        }

        cv::Mat U, S, Vt;
        cv::SVD::compute(points_centered, U, S, Vt);
        cv::Vec3f normal(Vt.at<float>(2, 0), Vt.at<float>(2, 1), Vt.at<float>(2, 2));
        return normal;
    }

    cv::Vec3f imageToCameraCoordinates(const cv::Mat &depth_image, const cv::Point &center)
    {
        float depth = 0.0f;
        if (center.y >= 0 && center.y < depth_image.rows && center.x >= 0 && center.x < depth_image.cols)
        {
            depth = depth_image.at<float>(center.y, center.x);
        }
        else
        {
            ROS_WARN("Center (%d, %d) out of depth image bounds", center.x, center.y);
        }
        float Z = depth;
        float X = (center.x - cx_) * Z / fx_;
        float Y = (center.y - cy_) * Z / fy_;
        return cv::Vec3f(X, Y, Z);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_rect_detector");
    ImageRectDetector detector;
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}