#include "charging_station/dec_circle.h"

namespace charging_station
{

    // 图像矩形检测器类构造函数
    ImageRectDetector::ImageRectDetector()
        : it_(nh_)
    {
        // image_sub_ = it_.subscribe("/xv_sdk/xv_dev/fisheye_cameras/left/image", 1, &ImageRectDetector::imageCallback, this);
        // image_sub_ = it_.subscribe("camera/image", 1, &ImageRectDetector::imageCallback, this);

        // depth_sub_ = it_.subscribe("/xv_sdk/xv_dev/fisheye_cameras/left/depth", 1, &ImageRectDetector::depthCallback, this);
        // center_pub_ = nh_.advertise<geometry_msgs::Point>("/detected_rectangle_center", 10);
        // normal_pub_ = nh_.advertise<geometry_msgs::Vector3>("/detected_rectangle_normal", 10);
        // fx_ = 500;
        // fy_ = 500;
        // cx_ = 320;
        // cy_ = 240;
    }

    // geometry_msgs::Pose（三维位姿，含位置和四元数旋转）转换为 geometry_msgs::Pose2D（二维位姿，仅平面坐标和偏航角）
    geometry_msgs::Pose2D ImageRectDetector::pose3DTo2D(const geometry_msgs::Pose &pose3d)
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
    geometry_msgs::Pose ImageRectDetector::pose2DTo3DManual(const geometry_msgs::Pose2D &pose2d)
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

    // 图像回调函数
    void ImageRectDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            // 用于将 ROS 的图像消息（sensor_msgs::Image）转换为 OpenCV 的图像格式（cv::Mat)输出图像的像素编码格式（如 BGR8、RGB8、MONO8 等）
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // 输出图片的行列
        ROS_INFO("Received image with dimensions: %d x %d", cv_ptr_->image.cols, cv_ptr_->image.rows);

        // clone() 是 OpenCV 的 cv::Mat 成员函数，会生成一个全新的、完全独立的内存副本。修改克隆后的图像不会影响原始数据。
        cv::Mat processed_image = cv_ptr_->image.clone();
        processImage(processed_image);
    }

    void ImageRectDetector::depthCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            // 用于将 ROS 的图像消息（sensor_msgs::Image）转换为 OpenCV 的图像格式（cv::Mat)输出图像的像素编码格式（如 BGR8、RGB8、MONO8 等）
            cv::Mat depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            ROS_INFO("Received depth image with dimensions: %d x %d", depth_image.cols, depth_image.rows);
            depth_image_buffer_[msg->header.stamp] = depth_image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    // 获取离timestamp时间戳最近的一帧深度图像
    cv::Mat ImageRectDetector::getDepthImage(const ros::Time &timestamp)
    {
        auto closest_timestamp = std::min_element(depth_image_buffer_.begin(), depth_image_buffer_.end(),
                                                  [&](const std::pair<ros::Time, cv::Mat> &a, const std::pair<ros::Time, cv::Mat> &b)
                                                  {
                                                      return (a.first - timestamp).toSec() < (b.first - timestamp).toSec();
                                                  });

        if (closest_timestamp == depth_image_buffer_.end())
        {
            ROS_WARN("No depth image found for the given timestamp.");
            return cv::Mat();
        }

        if (std::abs((closest_timestamp->first - timestamp).toSec()) > 0.05)
        {
            ROS_WARN("No suitable depth image found within 0.05 seconds.");
            return cv::Mat();
        }
        // 取完cv::mat之后删除该事件戳
        cv::Mat depth_image = closest_timestamp->second;
        depth_image_buffer_.erase(closest_timestamp);
        return depth_image;
    }

    // 处理图像
    void ImageRectDetector::processImage(cv::Mat &image)
    {
        ROS_INFO("Processing image with dimensions: %d x %d", image.cols, image.rows);

        cv::Mat gray_image;
        // 将 BGR 彩色图像 转换为 灰度图像
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        ROS_INFO("Gray image dimensions: %d x %d", gray_image.cols, gray_image.rows);
        // 用于将灰度图像转换为黑白二值图像,THRESH_BINARY_INV,反向二值化（大于阈值110变黑，否则白）
        cv::Mat thresh = thresholdImage(gray_image, 110, 255, cv::THRESH_BINARY_INV);
        ROS_INFO("Threshold image dimensions: %d x %d", thresh.cols, thresh.rows);
        // 用于 检测并过滤图像中的矩形区域 的方法，返回轮廓的中心点和宽高
        auto rectangles_to_draw = filterRegions(thresh, 50, 2000);
        ROS_INFO("Detected %zu rectangles", rectangles_to_draw.size());

        for (const auto &[center, width, height] : rectangles_to_draw)
        {
            // 用于在图像上绘制矩形的核心函数// cv::Scalar(0, 255, 0)颜色(B,G,R)，线宽2
            cv::rectangle(image, cv::Point(center.x - width / 2, center.y - height / 2),
                          cv::Point(center.x + width / 2, center.y + height / 2), cv::Scalar(0, 255, 0), 2);
            // 绘制十字
            drawCross(image, center, cv::Scalar(255, 0, 0), 2, 10);
        }

        if (rectangles_to_draw.empty())
        {
            ROS_INFO("No rectangles detected.");
        }
        else if (rectangles_to_draw.size() == 1)
        {
            // 只有一个矩形时获取矩形的四个点的坐标
            auto [center, width, height] = rectangles_to_draw[0];
            std::vector<cv::Point> rectangle_corners = {
                cv::Point(center.x - width / 2, center.y - height / 2),
                cv::Point(center.x + width / 2, center.y - height / 2),
                cv::Point(center.x + width / 2, center.y + height / 2),
                cv::Point(center.x - width / 2, center.y + height / 2)};
            // 根据时间戳获取深度图像
            cv::Mat depth_image = getDepthImage(cv_ptr_->header.stamp);
            if (depth_image.empty())
            {
                ROS_WARN("Depth image is empty. Skipping depth-related operations.");
            }
            else
            {
                // 得到深度图像的法向量，代表图像的朝向，同时也是充电座的坐标
                cv::Vec3f normal = calculateRectangleNormal(depth_image, rectangle_corners);
                geometry_msgs::Vector3 normal_msg;
                normal_msg.x = normal[0];
                normal_msg.y = normal[1];
                normal_msg.z = normal[2];
                // 发布法向量坐标
                normal_pub_.publish(normal_msg);

                // 中心点转换到3D空间再发布
                cv::Vec3f center_3d = imageToCameraCoordinates(depth_image, center, fx_, fy_, cx_, cy_);
                geometry_msgs::Point center_3d_msg;
                center_3d_msg.x = center_3d[0];
                center_3d_msg.y = center_3d[1];
                center_3d_msg.z = center_3d[2];
                center_pub_.publish(center_3d_msg);

                charging_station_pose_.position.x = center_3d[0];
                charging_station_pose_.position.y = center_3d[1];
                charging_station_pose_.position.z = center_3d[2];

                // 以四元素的表达方式计算充电座位姿
                tf::Quaternion q;
                q.setRPY(0, 0, atan2(normal[1], normal[0]));
                charging_station_pose_.orientation.x = q.x();
                charging_station_pose_.orientation.y = q.y();
                charging_station_pose_.orientation.z = q.z();
                charging_station_pose_.orientation.w = q.w();
            }
        }
        else
        {
            ROS_INFO("Multiple rectangles detected. Waiting for single rectangle detection.");
        }

        ROS_INFO("Displaying processed image.");
        cv::imshow("Processed Image", image);
        cv::waitKey(1);
    }

    // 绘制十字
    void ImageRectDetector::drawCross(cv::Mat &img, cv::Point center, cv::Scalar color, int thickness, int size)
    {
        cv::line(img, cv::Point(center.x - size, center.y), cv::Point(center.x + size, center.y), color, thickness);
        cv::line(img, cv::Point(center.x, center.y - size), cv::Point(center.x, center.y + size), color, thickness);
        cv::putText(img, "rect", cv::Point(center.x - 5, center.y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    // 图像阈值处理，用于将灰度图像转换为黑白二值图像
    cv::Mat ImageRectDetector::thresholdImage(const cv::Mat &image, double threshold_value, double max_value, int method)
    {
        cv::Mat thresh;
        // 用于将灰度图像转换为黑白二值图像,THRESH_BINARY_INV,反向二值化（大于阈值110变黑，否则白）
        cv::threshold(image, thresh, threshold_value, max_value, method);
        if (thresh.empty())
        {
            ROS_ERROR("Threshold image is empty.");
        }
        return thresh;
    }

    // 计算轮廓周长
    double ImageRectDetector::calculatePerimeter(const std::vector<cv::Point> &cnt)
    {
        return cv::arcLength(cnt, true);
    }

    // 判断是否为矩形，通过外接矩形宽高比和外接矩形的面积和否切合轮廓面积
    bool ImageRectDetector::isRectangleLike(const std::vector<cv::Point> &cnt, const std::vector<cv::Point> &approx, double min_aspect_ratio, double max_aspect_ratio, double min_fill_ratio)
    {
        if (approx.size() != 4)
        {
            return false;
        }
        // 计算轮廓的最小外接矩形 的函数
        cv::Rect bounding_rect = cv::boundingRect(approx);
        // 计算宽高比
        double aspect_ratio = static_cast<double>(bounding_rect.width) / bounding_rect.height;
        // 宽高比不符合要求，返回false
        if (!(min_aspect_ratio <= aspect_ratio && aspect_ratio <= max_aspect_ratio))
        {
            return false;
        }
        // 计算轮廓面积 的函数
        double area = cv::contourArea(cnt);
        double rect_area = bounding_rect.width * bounding_rect.height;
        double fill_ratio = area / rect_area;
        // 轮廓面积相差大，返回false
        if (fill_ratio < min_fill_ratio)
        {
            return false;
        }

        return true;
    }

    // 用于 检测并过滤图像中的矩形区域 的方法,返回轮廓的中心点和宽高
    std::vector<std::tuple<cv::Point, int, int>> ImageRectDetector::filterRegions(const cv::Mat &image, double min_area, double max_area)
    {
        std::vector<std::tuple<cv::Point, int, int>> rectangles_info;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        // 在二值图像中查找所有连通区域（轮廓）cv::RETR_EXTERNAL：仅检测最外层轮廓（忽略嵌套轮廓）cv::CHAIN_APPROX_SIMPLE：压缩轮廓点（例如矩形只存储 4 个顶点）
        cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &cnt : contours)
        {
            // 计算当前轮廓的面积。
            double area = cv::contourArea(cnt);
            // 计算周长
            double perimeter = calculatePerimeter(cnt);
            // 过滤条件：只保留面积在 [min_area, max_area] 之间的轮廓
            if (area >= min_area && area <= max_area)
            {
                std::vector<cv::Point> approx;
                // 用多边形逼近轮廓，减少轮廓点数。0.04 * perimeter：逼近精度（周长比例，值越小越精确）approx 存储逼近后的顶点（如果是矩形，应有 4 个点），true轮廓闭合
                cv::approxPolyDP(cnt, approx, 0.04 * perimeter, true);
                // 如果是矩形，计算其中心点和宽高
                if (isRectangleLike(cnt, approx))
                {
                    // 计算包围轮廓的最小矩形（轴对齐）
                    cv::Rect bounding_rect = cv::boundingRect(approx);
                    // 中心点计算
                    cv::Point center(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y + bounding_rect.height / 2);
                    rectangles_info.emplace_back(center, bounding_rect.width, bounding_rect.height);
                }
            }
        }

        if (rectangles_info.empty())
        {
            ROS_INFO("No rectangles detected.");
        }

        return rectangles_info;
    }

    // 计算矩形法线，,确定矩形物体的朝向。从深度图像中计算矩形平面的法向量，主要应用于三维视觉任务（如平面检测、物体姿态估计）
    cv::Vec3f ImageRectDetector::calculateRectangleNormal(const cv::Mat &depth_image, const std::vector<cv::Point> &rectangle_corners)
    {
        if (depth_image.empty())
        {
            ROS_ERROR("Depth image is empty. Cannot calculate rectangle normal.");
            return cv::Vec3f(0, 0, 0);
        }

        std::vector<cv::Point3f> points_3d;
        for (const auto &corner : rectangle_corners)
        {
            if (corner.x < 0 || corner.x >= depth_image.cols || corner.y < 0 || corner.y >= depth_image.rows)
            {
                ROS_ERROR("Corner point out of depth image bounds: (%d, %d)", corner.x, corner.y);
                continue;
            }
            float depth = depth_image.at<float>(corner.y, corner.x);
            // 从深度图像中读取角点位置的深度值 Z
            float Z = depth;
            // 2D→3D转换：通过相机内参（fx_, fy_ 焦距，cx_, cy_ 光心）将像素坐标转换为3D点
            float X = (corner.x - cx_) * Z / fx_;
            float Y = (corner.y - cy_) * Z / fy_;
            points_3d.emplace_back(X, Y, Z);
        }

        cv::Mat points_mat(points_3d);
        cv::Mat centroid = cv::Mat::zeros(3, 1, CV_32F);
        // 计算3D点集的质心
        for (int i = 0; i < points_3d.size(); ++i)
        {
            // row(i)返回一个 1×N 的临时矩阵（行向量),t()对行向量进行转置，将其从 1×N 转换为 N×1 的列向量
            centroid += points_mat.row(i).t();
        }
        centroid /= points_3d.size();
        // 计算协方差矩阵
        cv::Mat covariance_matrix = cv::Mat::zeros(3, 3, CV_32F);
        for (int i = 0; i < points_3d.size(); ++i)
        {
            cv::Mat diff = points_mat.row(i).t() - centroid;
            covariance_matrix += diff * diff.t();
        }
        covariance_matrix /= points_3d.size();

        // PCA原理：协方差矩阵的特征向量对应数据的主方向。最小特征值对应的特征向量是平面的法向量（因为平面点沿法向变化最小）。
        cv::Mat eigenvalues, eigenvectors;
        cv::eigen(covariance_matrix, eigenvalues, eigenvectors);

        cv::Vec3f normal;
        // 归一化：将法向量转换为单位向量
        cv::normalize(eigenvectors.row(2).t(), normal);
        return normal;
    }

    // 角点转换到3D空间
    cv::Vec3f ImageRectDetector::imageToCameraCoordinates(const cv::Mat &depth_image, cv::Point image_point, double fx, double fy, double cx, double cy)
    {
        if (depth_image.empty())
        {
            ROS_ERROR("Depth image is empty. Cannot convert image point to camera coordinates.");
            return cv::Vec3f(0, 0, 0);
        }

        if (image_point.x < 0 || image_point.x >= depth_image.cols || image_point.y < 0 || image_point.y >= depth_image.rows)
        {
            ROS_ERROR("Image point out of depth image bounds: (%d, %d)", image_point.x, image_point.y);
            return cv::Vec3f(0, 0, 0);
        }
        float depth = depth_image.at<float>(image_point.y, image_point.x);
        float Z = depth;
        float X = (image_point.x - cx) * Z / fx;
        float Y = (image_point.y - cy) * Z / fy;
        return cv::Vec3f(X, Y, Z);
    }

    // 获取充电站位姿
    geometry_msgs::Pose ImageRectDetector::getChargingStationPose()
    {
        return charging_station_pose_;
    }

    // 获取充电站静态位姿
    geometry_msgs::Pose ImageRectDetector::getChargingStationPoseStatic()
    {
        geometry_msgs::Pose pose;
        geometry_msgs::Pose2D manner_charge_pose;
        manner_charge_pose.x = 3.48;
        manner_charge_pose.y = -2.07;
        manner_charge_pose.theta = -3.07;

        pose = pose2DTo3DManual(manner_charge_pose);

        // pose.position.x = 0.0;
        // pose.position.y = 0.0;
        // pose.position.z = 0.0;

        // tf::Quaternion q;
        // q.setRPY(0, 0, 0.0);
        // pose.orientation.x = q.x();
        // pose.orientation.y = q.y();
        // pose.orientation.z = q.z();
        // pose.orientation.w = q.w();

        return pose;
    }

    bool ImageRectDetector::QRcodeDectectPnp(QRcodePoseTemp &qr_pose_incamera, QRcodePoseTemp &camera_pose_inqrcode)
    {
        // 2. 检测二维码（假设已经检测到 4 个角点）
        cv::Mat image = cv::imread("/data/ld_ros/Qrcode.png");
        if (image.empty())
        {
            ROS_INFO("can not read image !!! ");
            return false;
        }

        // 6. 定义相机内参（需提前标定）
        // cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 941.0, 0, 1403, // fx, 0, cx
        // 0, 941.0, 855,                            // 0, fy, cy
        // 0, 0, 1);

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 631.0, 0, 1440, // fx, 0, cx
                                 0, 631.0, 808,                            // 0, fy, cy
                                 0, 0, 1);
        // 畸变
        // cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.342, 0.1567, 0.0009, -0.00008, -0.04);
        cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

        cv::Mat rvec, tvec; // 旋转向量和平移向量
        bool success = false;
        // aruco二维码识别
        if (arcuo)
        {
            ROS_INFO("start arcuo check QRcode !!! ");
            //  1. 加载 ArUco 字典（AprilTag 可用类似方法）
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

            // 3. 检测二维码
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

            if (ids.empty())
            {
                ROS_INFO("can not check QRcode !!! ");
                return false;
            }
            else
            {
                ROS_INFO("find QRcode !!! ");
            }

            // 3. 定义二维码的 3D 世界坐标（假设二维码边长 10cm，位于 Z=0 平面）
            const float qrCodeSize = 0.045f; // 假设二维码边长为10cm
            std::vector<cv::Point3f> qrCodeCorners3D = {
                cv::Point3f(0, 0, 0),                   // 左上
                cv::Point3f(qrCodeSize, 0, 0),          // 右上
                cv::Point3f(qrCodeSize, qrCodeSize, 0), // 右下
                cv::Point3f(0, qrCodeSize, 0)           // 左下
            };

            // 4. 使用 PnP 计算位姿

            success = cv::solvePnP(
                qrCodeCorners3D, // 3D 世界坐标
                corners[0],      // 2D 图像坐标
                camera_matrix,   // 相机内参
                dist_coeffs,     // 畸变系数
                rvec,            // 输出：旋转向量
                tvec             // 输出：平移向量
            );
        }
        // QRCodeDetector二维码识别
        else
        {
            ROS_INFO("start QRCodeDetector check QRcode !!! ");
            cv::QRCodeDetector qrDetector;
            std::vector<cv::Point2f> corners; // 存储二维码的 4 个图像角点（2D）
            bool detected = qrDetector.detect(image, corners);

            if (!detected || corners.size() != 4)
            {
                ROS_INFO("can not check QRcode !!! ");
                return false;
            }
            else
            {
                ROS_INFO("find QRcode !!! ");
            }

            // 3. 定义二维码的 3D 世界坐标（假设二维码边长 10cm，位于 Z=0 平面）
            const float qrCodeSize = 0.045f; // 假设二维码边长为10cm
            std::vector<cv::Point3f> qrCodeCorners3D = {
                cv::Point3f(0, 0, 0),                   // 左上
                cv::Point3f(qrCodeSize, 0, 0),          // 右上
                cv::Point3f(qrCodeSize, qrCodeSize, 0), // 右下
                cv::Point3f(0, qrCodeSize, 0)           // 左下
            };

            // 4. 使用 PnP 计算位姿
            cv::Mat rvec, tvec; // 旋转向量和平移向量
            success = cv::solvePnP(
                qrCodeCorners3D, // 3D 世界坐标
                corners,         // 2D 图像坐标
                camera_matrix,   // 相机内参
                dist_coeffs,     // 畸变系数
                rvec,            // 输出：旋转向量
                tvec             // 输出：平移向量
            );
        }

        if (!success)
        {
            ROS_INFO("pnp compute failed !!! ");
            return false;
        }

        // 使用示例：得到机器人坐标系下的二维码位姿 ，机器人坐标系 ---------->x,轴朝向外
        //                                                    |
        //                                                    |
        //                                                    |
        //                                                    |y
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // 可选：转换为欧拉角
        // cv::Mat M;
        // cv::Mat N;
        // cv::Vec3d euler_angles_camera = cv::RQDecomp3x3(R, M, N);
        // std::cout << "二维码相对于机器人欧拉角 (roll, pitch, yaw): "
        //           << euler_angles_camera(0) << " " << euler_angles_camera(1)
        //           << " " << euler_angles_camera(2) << " " << std::endl;
        double yaw, pitch, roll;
        pitch = asin(-R.at<double>(2, 0));
        roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        // std::cout << "二维码相对于机器人的角度: " << std::endl;
        // std::cout << "Yaw (Z): " << yaw * 180 / CV_PI << "°" << std::endl;
        // std::cout << "Pitch (Y): " << pitch * 180 / CV_PI << "°" << std::endl;
        // std::cout << "Roll (X): " << roll * 180 / CV_PI << "°" << std::endl;

        // 方法1：使用 at<double> 访问
        double tx = tvec.at<double>(0);
        double ty = tvec.at<double>(1);
        double tz = tvec.at<double>(2);
        std::cout << "二维码相对于鱼眼相机的位置 (x, y, z): " << tx << "  " << ty << "  " << tz << std::endl;

        qr_pose_incamera.x = tx;
        qr_pose_incamera.y = ty;
        qr_pose_incamera.z = tz;
        qr_pose_incamera.roll = roll;
        qr_pose_incamera.pitch = pitch;
        qr_pose_incamera.yaw = yaw;
        std::cout << "二维码相对于鱼眼相机的角度 (roll, pitch, yaw): "
                  << qr_pose_incamera.roll * 180 / CV_PI << "  " << qr_pose_incamera.pitch * 180 / CV_PI
                  << "  " << qr_pose_incamera.yaw * 180 / CV_PI << std::endl;

        // 9. 计算机器人相对于二维码的位姿
        cv::Mat R_robot_to_tag = R.t();                  // 旋转矩阵的逆 = 转置
        cv::Mat t_robot_to_tag = -R_robot_to_tag * tvec; // 平移向量

        // 可选：转换为欧拉角
        // cv::Mat K;
        // cv::Mat Q;
        // cv::Vec3d euler_angles = cv::RQDecomp3x3(R_robot_to_tag, K, Q);
        pitch = asin(-R_robot_to_tag.at<double>(2, 0));
        roll = atan2(R_robot_to_tag.at<double>(2, 1), R_robot_to_tag.at<double>(2, 2));
        yaw = atan2(R_robot_to_tag.at<double>(1, 0), R_robot_to_tag.at<double>(0, 0));
        camera_pose_inqrcode.x = t_robot_to_tag.at<double>(0);
        camera_pose_inqrcode.y = t_robot_to_tag.at<double>(1);
        camera_pose_inqrcode.z = t_robot_to_tag.at<double>(2);
        camera_pose_inqrcode.roll = roll;
        camera_pose_inqrcode.pitch = pitch;
        camera_pose_inqrcode.yaw = yaw;
        std::cout << "鱼眼相机相对于二维码 (x, y, z) "
                  << camera_pose_inqrcode.x << " " << camera_pose_inqrcode.y << " "
                  << camera_pose_inqrcode.z << " (roll, pitch, yaw): " << camera_pose_inqrcode.roll * 180 / CV_PI << " "
                  << camera_pose_inqrcode.pitch * 180 / CV_PI << " " << camera_pose_inqrcode.yaw * 180 / CV_PI << " " << std::endl;
        return true;
    }

} // namespace charging_station