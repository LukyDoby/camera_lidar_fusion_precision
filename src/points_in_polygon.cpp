
#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include <iostream>
#include <random>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/optional.hpp>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/filters/convolution_3d.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "tf2/exceptions.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <fstream>
#include <iomanip> 

using namespace std::chrono_literals;
using namespace std;
using namespace cv;

class Ransac_node : public rclcpp::Node
{
  public:
    Ransac_node()
    : Node("ransac_node")
    {
      rclcpp::QoS qos(30);
      auto rmw_qos_profile = qos.get_rmw_qos_profile();

      ptcloud_sub_.subscribe(this, "/lidar/poincloud", rmw_qos_profile);
      image_sub_.subscribe(this, "/camera/image", rmw_qos_profile);
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(30), ptcloud_sub_, image_sub_);
      sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.5)); // 0.5 s
      sync_->registerCallback(
        std::bind(&Ransac_node::ptCloudCallback, this, std::placeholders::_1, std::placeholders::_2));

      pub_camera_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);
      pub_colored_image = this->create_publisher<sensor_msgs::msg::Image>("/image_color", 10);
      pub_fusion_precision = this->create_publisher<std_msgs::msg::Float64>("/fusion_precision", 10);
          
      bufferCamLidTf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*bufferCamLidTf_);

      _frame_camera = this->declare_parameter<std::string>("_frame_camera", "camera_link");
      _frame_lidar = this->declare_parameter<std::string>("_frame_lidar", "lidar_link");
      _frame_origin = this->declare_parameter<std::string>("_frame_origin", "lidar_link");
      patternSize = cv::Size(8,5);
      cameraMatrix = (cv::Mat_<double>(3, 3) << 1182.1, 0.0, 931.3, 0.0, 1182.5, 586.7, 0.0, 0.0, 1.0);
      distCoeffs = (cv::Mat_<double>(1, 5) << -0.2184, 0.0610, -0.0324, 0.0, 0.0);

      cam_model.fromCameraInfo(buildCameraInfo());

      int board_width = 8;
      int board_height = 5;
      float squareSize = 42.5;

      for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
          world_points.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
        }
      }
      corners_wrld = {
        cv::Point3f(-148.5f, -135.5f, 0.0f),
        cv::Point3f(-148.5f,  313.5f, 0.0f),
        cv::Point3f( 449.0f, -135.5f, 0.0f),
        cv::Point3f( 449.0f,  313.5f, 0.0f)
      };
    }

    ~Ransac_node(){}

  private:

    void ptCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
    {
      public_camera_info();

      cv_bridge::CvImageConstPtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
      }
      catch (const cv_bridge::Exception & ex)
      {
        RCLCPP_INFO(this->get_logger(), "cv_bridge conversion failed: %s", ex.what());
        return;
      }
      cv::Mat modifiable_image = cv_ptr->image;

      imagePoints.clear();
      try
      {
        cv::Mat gray, small;
        cv::cvtColor(modifiable_image, gray, cv::COLOR_RGB2GRAY);
        const double s = 0.5;
        cv::resize(gray, small, cv::Size(), s, s, cv::INTER_AREA);

        bool found = cv::findChessboardCorners(small, patternSize, imagePoints,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);
        if (found) {
          for (auto & p : imagePoints) p *= static_cast<float>(1.0 / s);   // back to full-res pixels
          cv::cornerSubPix(gray, imagePoints, {11, 11}, {-1, -1},
              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
        }
        
        if (!found || imagePoints.size() != 40)
        {
          RCLCPP_INFO(this->get_logger(), "Checkerboard not found");
          publishImage(image_msg->header, modifiable_image);
          return;
        }
      }
      catch (const std::exception & ex)
      {
        RCLCPP_INFO(this->get_logger(), "found_checker_corners failed: %s", ex.what());
        publishImage(image_msg->header, modifiable_image);
        return;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();  //(new pcl::PointCloud<pcl::PointXYZ>)

      try
      {
        pcl::fromROSMsg(*msg, *cloud);
      }
      catch(const std::exception & ex)
      {
        RCLCPP_INFO(this->get_logger(), "fromROSMsg failed: %s", ex.what());
        publishImage(image_msg->header, modifiable_image);
        return;
      }

      pcl::PointIndices::Ptr points_in(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      for (int i = 0; i < (*cloud).size(); i++)
      {
        pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        if ((pt.x > x_min && pt.x < x_max) && (pt.y > y_min && pt.y < y_max) && (pt.z > z_min && pt.z < z_max)) // e.g. remove all pts below zAvg
        {
          points_in->indices.push_back(i);
        }
      }

      extract.setInputCloud(cloud);
      extract.setIndices(points_in);
      extract.setNegative(false);
      extract.filter(*cloud);

      // Apllying RANSAC algorithm

      std::vector<int> inliers;
      pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

      if(cloud->points.size() > 3)
      {
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_plane);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
        pcl::copyPointCloud (*cloud, inliers, *final);
      }
      else
      {
        publishImage(image_msg->header, modifiable_image);
        RCLCPP_INFO(this->get_logger(), "Point cloud too small");
        return;
      }

      geometry_msgs::msg::TransformStamped tfGeomLidartoCam_actual;

      // Detecting checkerboard
      cv::Size patternSize(8, 5);  // Inner corners
      std::vector<cv::Point2f> projected_pts;
      std::vector<cv::Point2f> projected_pts_final;

      try
      {
        cv::Mat rotation_vector, translation_vector;
        bool success = cv::solvePnP(
          world_points,                 // std::vector<cv::Point3f>
          imagePoints,                   // std::vector<cv::Point2f>
          cameraMatrix,                 // cv::Mat (3x3)
          distCoeffs,                    // cv::Mat (1x5)
          rotation_vector,                // Output rotation vector
          translation_vector              // Output translation vector
        );
        if(success)
        {
          cv::projectPoints(
            corners_wrld,            // std::vector<cv::Point3f>
            rotation_vector,         // cv::Mat (rvec from solvePnP)
            translation_vector,      // cv::Mat (tvec from solvePnP)
            cameraMatrix,           // cv::Mat (3x3)
            distCoeffs,             // cv::Mat (1x5)
            projected_pts            // Output: std::vector<cv::Point2f>
          );

          int thickness = 3;

          projected_pts_final.push_back(projected_pts[0]);
          projected_pts_final.push_back(projected_pts[2]);
          projected_pts_final.push_back(projected_pts[3]);
          projected_pts_final.push_back(projected_pts[1]);

          cv::line(modifiable_image, projected_pts_final[0], projected_pts_final[1], Scalar(0, 255, 255), thickness, LINE_8);
          cv::line(modifiable_image, projected_pts_final[1], projected_pts_final[2], Scalar(0, 255, 255), thickness, LINE_8);
          cv::line(modifiable_image, projected_pts_final[2], projected_pts_final[3], Scalar(0, 255, 255), thickness, LINE_8);
          cv::line(modifiable_image, projected_pts_final[3], projected_pts_final[0], Scalar(0, 255, 255), thickness, LINE_8);
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "PnP not succeded");
          publishImage(image_msg->header, modifiable_image);
        }
      }
      catch (const cv::Exception & ex)
      {
        RCLCPP_INFO(this->get_logger(), "Checkerboard detection/PnP threw an exception: %s", ex.what());
        publishImage(image_msg->header, modifiable_image);
        return;
      }
      try
      {
        tfGeomLidartoCam_actual = bufferCamLidTf_->lookupTransform(_frame_camera,_frame_lidar, tf2::TimePointZero);
      }
      catch(const tf2::TransformException & ex)
      {
        RCLCPP_INFO(this->get_logger(), "Invalid transform lidar -> camera");
        RCLCPP_INFO(this->get_logger(), "[%s]",_frame_lidar.c_str() );
        publishImage(image_msg->header, modifiable_image);
        return;
      }

      tf2::TimePoint time = tf2_ros::fromMsg(msg->header.stamp);

      tfGeomLidartoCam_actual.header.stamp = tf2_ros::toMsg(time);
      double fusion_precision = 0;
      double num_pts_in_polygon = 0;
      double total_num_pts = 0;

      const Eigen::Isometry3d T_cam_lidar = tf2::transformToEigen(tfGeomLidartoCam_actual);
      std::vector<cv::Point3f> pts_cam;
      pts_cam.reserve(final->size());
      for (const pcl::PointXYZ& p : final->points)
      {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        const Eigen::Vector3d pc = T_cam_lidar * Eigen::Vector3d(p.x, p.y, p.z);
        if (pc.z() <= 0.0) continue;                       // behind the camera
        pts_cam.emplace_back(pc.x(), pc.y(), pc.z());
      }
      std::vector<cv::Point2f> uv;
      if (!pts_cam.empty()) {
        const cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);  // already in the camera frame
        cv::projectPoints(pts_cam, zero, zero, cameraMatrix, distCoeffs, uv);
      }
      for(const auto & q : uv)
      {
        if ( q.y >= 2 && q.y < (cv_ptr->image.rows-2) && q.x >= 2 && q.x < (cv_ptr->image.cols-2))
        {
          total_num_pts++;

          if(cv::pointPolygonTest(projected_pts_final, cv::Point2f(q), false) >= 0)
          {
            // Color image point to green
            num_pts_in_polygon++;

            modifiable_image.at<cv::Vec3b>(int(q.y), int(q.x)) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)-1, int(q.x)) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)+1, int(q.x)) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y), int(q.x)-1) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y), int(q.x)+1) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)-1, int(q.x)-1) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)-1, int(q.x)+1) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)+1, int(q.x)-1) = cv::Vec3b(0, 255, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)+1, int(q.x)+1) = cv::Vec3b(0, 255, 0);
          }
          else
          {
            // Color image point to red
            modifiable_image.at<cv::Vec3b>(int(q.y), int(q.x)) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)-1, int(q.x)) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)+1, int(q.x)) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y), int(q.x)-1) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y), int(q.x)+1) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)-1, int(q.x)-1) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)-1, int(q.x)+1) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)+1, int(q.x)-1) = cv::Vec3b(255, 0, 0);
            modifiable_image.at<cv::Vec3b>(int(q.y)+1, int(q.x)+1) = cv::Vec3b(255, 0, 0);
          }
        }
      }

      fusion_precision = (total_num_pts > 0) ? (num_pts_in_polygon / total_num_pts) * 100.0 : 0.0;

      std_msgs::msg::Float64 fusion_precision_msg;
      fusion_precision_msg.data = fusion_precision;
      pub_fusion_precision->publish(fusion_precision_msg);

      publishImage(image_msg->header, modifiable_image);
    }
    sensor_msgs::msg::CameraInfo buildCameraInfo()
    {
      sensor_msgs::msg::CameraInfo msgs;
      msgs.header.stamp = this->now();
      msgs.header.frame_id = "camera_link";
      msgs.distortion_model = "plumb_bob";
      msgs.height = 1200;
      msgs.width = 1920;
      msgs.k = {1182.1, 0.0, 931.3,
                0.0, 1182.5, 586.7,
                0.0, 0.0, 1.0};
      msgs.d = {-0.2184, 0.0610, -0.0324, 0, 0.000000};
      msgs.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      msgs.p = {1182.1, 0.0, 931.3, 0.0,
                0.0, 1182.5, 586.7, 0.0,
                0.0, 0.0, 1.0, 0.0};
      return msgs;
    }
    void public_camera_info()
    {
      pub_camera_info->publish(buildCameraInfo());
    }
    void publishImage(const std_msgs::msg::Header & header, const cv::Mat & img)
    {
      pub_colored_image->publish(*cv_bridge::CvImage(header, "rgb8", img).toImageMsg());
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_colored_image;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fusion_precision;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ptcloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::string _frame_origin;
    std::string _frame_lidar;
    std::string _frame_camera;
    std::unique_ptr<tf2_ros::Buffer>bufferCamLidTf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
    image_geometry::PinholeCameraModel cam_model;

    float x_max = -0.5;
    float x_min = -3;
    float y_min = -1.5;
    float y_max = 1.5;
    float z_min = -1;
    float z_max = 1.8;

    std::vector<cv::Point2f> imagePoints;
    cv::Size patternSize;  // Inner corners
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Point3f> world_points;
    std::vector<cv::Point3f> corners_wrld;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ransac_node>());
  rclcpp::shutdown();
  return 0;
}