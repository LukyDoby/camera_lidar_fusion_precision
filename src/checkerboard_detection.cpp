
#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <random>

#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.hpp>
//#include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/camera_info.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
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

using namespace std::chrono_literals;
using namespace std;
using namespace cv;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Ransac_node : public rclcpp::Node
{
  public:
    Ransac_node()
    : Node("ransac_node"),  count_(0)
    {
      rclcpp::QoS qos(10);
      auto rmw_qos_profile = qos.get_rmw_qos_profile();
      sub_ptCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/poincloud",10, std::bind(&Ransac_node::ptCloudCallback, this, std::placeholders::_1));
      
      sub_camera_img = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10, std::bind(&Ransac_node::imageCallback, this, std::placeholders::_1));

      sub_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 10, std::bind(&Ransac_node::cam_cb, this, std::placeholders::_1));
      
      pub_camera_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);
      
      pub_ransac_ptCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_plane_cloud", 10);
      // _frame_origin = this->declare_parameter<std::string>("_frame_origin", "base_link");
      pub_colored_image = this->create_publisher<sensor_msgs::msg::Image>("/image_color", 10);
    
      bufferCamLidTf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      bufferLidarOrigin_= std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_buffer= std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*bufferCamLidTf_);
      tf_listener_2 = std::make_shared<tf2_ros::TransformListener>(*bufferLidarOrigin_);
      tf_listener_3 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
      // timer_ = this->create_wall_timer(
      // 500ms, std::bind(&Ransac_node::timer_callback, this));

      _frame_camera = this->declare_parameter<std::string>("_frame_camera", "camera_link");
      _frame_lidar = this->declare_parameter<std::string>("_frame_lidar", "lidar_link");
      //_frame_lidar = this->get_parameter("_frame_lidar", _frame_lidar);
      _frame_origin = this->declare_parameter<std::string>("_frame_origin", "base_link");
      patternSize = cv::Size(8,5);
      cameraMatrix = (cv::Mat_<double>(3, 3) << 1182.1, 0.0, 931.3, 0.0, 1182.5, 586.7, 0.0, 0.0, 1.0);
      distCoeffs = (cv::Mat_<double>(1, 5) << -0.2184, 0.0610, -0.0324, 0.0, 0.0);

      int board_width = 8;
      int board_height = 5;
      float squareSize = 42.5;

      for (int i = 0; i < board_height; i++)
      {
        for (int j = 0; j < board_width; j++)
        {
          world_points.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
        }
      }
      corners_wrld = 
      {
        cv::Point3f(-148.5f, -135.5f, 0.0f),
        cv::Point3f(-148.5f,  313.5f, 0.0f),
        cv::Point3f( 449.0f, -135.5f, 0.0f),
        cv::Point3f( 449.0f,  313.5f, 0.0f)
      };
    }

  private:
    void ptCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
      counter++;
      public_camera_info();

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();  //(new pcl::PointCloud<pcl::PointXYZ>)
     
      pcl::fromROSMsg(*msg, *cloud);

      pcl::PointIndices::Ptr points_in(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      for (int i = 0; i < (*cloud).size(); i++)
      {
        pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        if ((pt.x > x_min && pt.x < x_max) && (pt.y > y_min && pt.y < y_max) && (pt.z > z_min && pt.z < z_max)) // e.g. remove all pts below zAvg
        {
          points_in->indices.push_back(i);
          // RCLCPP_INFO(this->get_logger(), "points in");
        }
      }
      extract.setInputCloud(cloud);
      extract.setIndices(points_in);
      extract.setNegative(false);
      extract.filter(*cloud);

        // Apllying RANSAC algorithm

      std::vector<int> inliers;
      pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
      model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

      pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_plane);
      ransac.setDistanceThreshold (.01);
      ransac.computeModel();
      ransac.getInliers(inliers);
      pcl::copyPointCloud (*cloud, inliers, *final);

      // Publishing segmented calibration board
      auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*final, *output);
      output->header.frame_id = _frame_lidar;
      output->header.stamp    = msg->header.stamp;
      pub_ransac_ptCloud->publish(*output);
      geometry_msgs::msg::TransformStamped tfGeomLidartoCam_actual;
      image_geometry::PinholeCameraModel cam_model_actual = cam_model;

      if(cv_ptr_temp != nullptr)
      {

        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_ptr_temp;
        cv::Mat modifiable_image = cv_ptr->image.clone();

        // Detecting checkerboard

        // cv::Mat udistortedIm;
        // cv::undistort(modifiable_image, udistortedIm, cameraMatrix, distCoeffs);
    
        // Detecting checkerboard
        cv::Size patternSize(8, 5);  // Inner corners
        found_checker_corners = cv::findChessboardCorners(modifiable_image, patternSize, imagePoints);
        if(found_checker_corners && imagePoints.size() == 40)
        {   // 8x5 = 40

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
            std::vector<cv::Point2f> projected_pts;
            cv::projectPoints(
                corners_wrld,            // std::vector<cv::Point3f>
                rotation_vector,         // cv::Mat (rvec from solvePnP)
                translation_vector,      // cv::Mat (tvec from solvePnP)
                cameraMatrix,           // cv::Mat (3x3)
                distCoeffs,             // cv::Mat (1x5)
                projected_pts            // Output: std::vector<cv::Point2f>
            );
            int thickness = 3; 
            cv::line(modifiable_image, projected_pts[0], projected_pts[2], Scalar(0, 255, 0), thickness, LINE_8);
            cv::line(modifiable_image, projected_pts[2], projected_pts[3], Scalar(0, 255, 0), thickness, LINE_8); 
            cv::line(modifiable_image, projected_pts[3], projected_pts[1], Scalar(0, 255, 0), thickness, LINE_8); 
            cv::line(modifiable_image, projected_pts[1], projected_pts[0], Scalar(0, 255, 0), thickness, LINE_8); 

            RCLCPP_INFO(this->get_logger(), "Corners detected");
          }
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "checker not found");
        }
        try
        {
          tfGeomLidartoCam_actual = bufferCamLidTf_->lookupTransform(_frame_camera,_frame_lidar, tf2::TimePointZero);

        }
        catch(const tf2::TransformException & ex)
        {
          RCLCPP_INFO(this->get_logger(), "Invalid transform lidar -> camera");
          RCLCPP_INFO(this->get_logger(), "[%s]",_frame_lidar.c_str() );
          return;
        }
        geometry_msgs::msg::TransformStamped tfGeom_lidar_origin;
        tf2::TimePoint time = tf2_ros::fromMsg(msg->header.stamp);
        tfGeomLidartoCam_actual.header.stamp = tf2_ros::toMsg(time);
        for(const pcl::PointXYZ& p : final->points)
        {
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          {
            continue;
          }
          const double range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
          //if (range > 0.1 && range < 12.0) {
          if(true)
          {
            geometry_msgs::msg::PointStamped point;
            
            point.point.x = p.x;
            point.point.y = p.y;
            point.point.z = p.z;
    
            geometry_msgs::msg::PointStamped point_cam;

            tf2::doTransform(point, point_cam, tfGeomLidartoCam_actual);
            geometry_msgs::msg::PointStamped point_world;

            cv::Point2d uv; 
            //cv::Point3d pt_cv(-point_cam.point.y, -point_cam.point.z,point_cam.point.x);
            cv::Point3d pt_cv(point_cam.point.x, point_cam.point.y,point_cam.point.z);
            // RCLCPP_INFO(this->get_logger(), "x: %f, y : %f, z: %f", point_cam.point.x,point_cam.point.y,point_cam.point.z );
            cv::Point2d uv_rectified;
            if(cam_model_actual.initialized())
            {
              uv_rectified = cam_model_actual.project3dToPixel(pt_cv); 
              uv = cam_model_actual.unrectifyPoint(uv_rectified);   // change px to raw image px
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "cam model not initialized");
              return;
            }
            if(cv_ptr != NULL)
            {
              if ( uv.y >= 2 && uv.y < (cv_ptr->image.rows-2) && uv.x >= 2 && uv.x < (cv_ptr->image.cols-2))
              {
                modifiable_image.at<cv::Vec3b>(int(uv.y), int(uv.x)) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y)-1, int(uv.x)) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y)+1, int(uv.x)) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y), int(uv.x)-1) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y), int(uv.x)+1) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y)-1, int(uv.x)-1) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y)-1, int(uv.x)+1) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y)+1, int(uv.x)-1) = cv::Vec3b(255, 0, 0);
                modifiable_image.at<cv::Vec3b>(int(uv.y)+1, int(uv.x)+1) = cv::Vec3b(255, 0, 0);
              } 
            }
          }
        }
        auto modified_msg = cv_bridge::CvImage(msg->header, "rgb8", modifiable_image).toImageMsg();
        pub_colored_image->publish(*modified_msg);
        RCLCPP_INFO(this->get_logger(), "Color image published");
      }
    }
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_ptr_temp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }  
  void cam_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    cam_model.fromCameraInfo(msg);  
  }

  void public_camera_info()
  {
    //auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
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
    
    pub_camera_info->publish(msgs);
    //  RCLCPP_INFO(this->get_logger(), "Publishing CameraInfo message...");
  }
  
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ransac_ptCloud;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_colored_image;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ptCloud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_img;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;

  cv_bridge::CvImageConstPtr cv_ptr_temp;
 // rclcpp::TimerBase::SharedPtr timer_;
  int count_;
  size_t counter = 0;
  
  std::string _frame_origin;
  std::string _frame_lidar;
  std::string _frame_camera;
  std::unique_ptr<tf2_ros::Buffer>bufferLidarOrigin_;
  std::unique_ptr<tf2_ros::Buffer>bufferCamLidTf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_2{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_3{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer; 
  image_geometry::PinholeCameraModel cam_model;

  float x_max = -0.5;
  float x_min = -3;
  float y_min = -1.5;
  float y_max = 1.5;
  float z_min = -1;
  float z_max = 1.8;

  bool found_checker_corners;

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