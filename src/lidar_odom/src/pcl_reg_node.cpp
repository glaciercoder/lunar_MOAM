#include <small_gicp/pcl/pcl_registration.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <small_gicp/util/downsampling_omp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <chrono>

using namespace std::chrono_literals;

namespace small_gicp {
class LidarOdometryNode : public rclcpp::Node
{

  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double CorrespondenceRandomness;
      double MaxCorrespondenceDistance;
      double VoxelResolution;
      const int numthreads = 4;
      std::string RegistrationType;
      std::string scan_topic1;
      std::string scan_topic2;
      std::string pointcloud2_topic_name;
      std::string odom_topic_name;
      std::string odom_frame_id;
      std::string base_frame_id;

      

      // Clouds
      pcl::PointCloud<pcl::PointXYZ>::Ptr points1(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr points2(new pcl::PointCloud<pcl::PointXYZ>);

      

      this->get_parameter("CorrespondenceRandomness", CorrespondenceRandomness);
      this->get_parameter("MaxCorrespondenceDistance", MaxCorrespondenceDistance);
      this->get_parameter("VoxelResolution", VoxelResolution);
      this->get_parameter("RegistrationType", RegistrationType);
      this->get_parameter("scan_topic1", scan_topic1);
      this->get_parameter("scan_topic2", scan_topic2);
      this->get_parameter("odom_topic_name", odom_topic_name);
      this->get_parameter("pointcloud2_topic_name", pointcloud2_topic_name);
      this->get_parameter("odom_frame_id", odom_frame_id);
      this->get_parameter("base_frame_id", base_frame_id);

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "CorrespondenceRandomness: %.4f", CorrespondenceRandomness);
      RCLCPP_INFO(this->get_logger(), "MaxCorrespondenceDistance: %.4f", MaxCorrespondenceDistance);
      RCLCPP_INFO(this->get_logger(), "VoxelResolution %.4f", VoxelResolution);
      RCLCPP_INFO(this->get_logger(), "RegistrationType: %s", RegistrationType.c_str());
      RCLCPP_INFO(this->get_logger(), "scan_topic1: %s", scan_topic1.c_str());
      RCLCPP_INFO(this->get_logger(), "scan_topic2: %s", scan_topic2.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "pointcloud2_topic_name: %s", pointcloud2_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id.c_str());
      RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id.c_str());

      // Declare registrator
      reg.setNumThreads(numthreads);
      reg.setCorrespondenceRandomness(CorrespondenceRandomness);
      reg.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
      reg.setVoxelResolution(VoxelResolution);
      reg.setRegistrationType(RegistrationType.c_str());  // or "GICP" (default = "GICP")

      // Transformation matrix
      T = Eigen::Isometry3d::Identity();

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);

      // No need to execute registration in callback
      scan_sub1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic1, 2, std::bind(&LidarOdometryNode::scan_callback1, this, std::placeholders::_1)
      );
      scan_sub2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic2, 2, std::bind(&LidarOdometryNode::scan_callback2, this, std::placeholders::_1)
      );

      pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud2_topic_name, 100);
      odometry_transform_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
      realtime_odometry_transform_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);
      timer_ = this->create_wall_timer(500ms, std::bind(&LidarOdometryNode::timer_callback, this));
    }

    private:
      Eigen::Isometry3d T;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odometry_transform_publisher_;
      std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub1;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub2;
      pcl::PointCloud<pcl::PointXYZ>::Ptr points1;
      pcl::PointCloud<pcl::PointXYZ>::Ptr points2;
      pcl::PointCloud<pcl::PointXYZ>::Ptr target;
      pcl::PointCloud<pcl::PointXYZ>::Ptr source;

      RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;


      void parameter_initilization() {
        this->declare_parameter<double>("CorrespondenceRandomness", 20);
        this->declare_parameter<double>("MaxCorrespondenceDistance", 1.0);
        this->declare_parameter<double>("VoxelResolution", 1.0);
        this->declare_parameter<std::string>("RegistrationType", "VGICP");
        this->declare_parameter<std::string>("scan_topic1", "robot1/mid360_PointCloud2");
        this->declare_parameter<std::string>("scan_topic2", "robot2/mid360_PointCloud2");
        this->declare_parameter<std::string>("odom_topic_name", "odom");
        this->declare_parameter<std::string>("pointcloud2_topic_name", "Modified_PointCloud2");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
      }

      enum class Axis
      {
        X = 0,
        Y = 1,
        Z = 2
      };

      // Pass according to the range for a given axis
      void pointcloud_range_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Axis axis, double min_range, double max_range, bool is_negative=false)
      {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        switch (axis)
        {
          case Axis::X:
            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_range, max_range);
            break;
        
          case Axis::Y:
            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_range, max_range);
            break;
          
          case Axis::Z:
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_range, max_range);
            break;
        }
        pass.setNegative(is_negative);
        pass.filter(*cloud);
      }

  

      void scan_callback1(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg){
        pcl::fromROSMsg(*scan_msg, *(points1));
        pointcloud_range_filter((points1), Axis::Z, -35, 35);

      }
      void scan_callback2(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg){
        pcl::fromROSMsg(*scan_msg, *points2);
        pointcloud_range_filter((this->points2), Axis::Z, -35, 35);

      }  
      // void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg) {
      //   // RCLCPP_INFO(this->get_logger(), "Resgistering");
      //   pcl::PointCloud<pcl::PointXYZ>::Ptr points1(new pcl::PointCloud<pcl::PointXYZ>);
      //   pcl::fromROSMsg(*scan_msg, *points1);
      //   // pass_x.setInputCloud(points1);
      //   // pass_x.filter(*points1);
      //   // pass_y.setInputCloud(points1);
      //   // pass_y.filter(*points1);
      //   // pointcloud_range_filter(points1, Axis::X, -15, 15);
      //   // pointcloud_range_filter(points1, Axis::Y, -35, 35);
      //   // pointcloud_tinker_filter(points1);
      //   // pointcloud_angle_filter(points1, Plane::XY, -M_PI/4, M_PI/4);
      //   // publish_pointcloud2(points1);
      //   pcl::PointCloud<pcl::PointXYZ>::Ptr target = voxelgrid_sampling_omp(*points1, 0.25);
      //   auto target_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*target);
        
      //   if (!reg.getInputTarget()) {
      //       RCLCPP_INFO(this->get_logger(), "No initial cloud found, set as initial");
      //       reg.setInputTarget(target_cloud);           
      //   }
      //   reg.setInputSource(target_cloud);
      //   // RCLCPP_INFO(this->get_logger(), "Begin regeister");
      //   auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      //   reg.align(*aligned);
      //   // RCLCPP_INFO(this->get_logger(), "Resgister Finished");
      //   T = T * Eigen::Isometry3d(reg.getFinalTransformation().cast<double>());
      //   reg.swapSourceAndTarget();
      //   publish_odometry();
      //   publish_tf();
      // }

      // void publish_odometry() {
      //   std::string fixed_id = "odom";

      //   nav_msgs::msg::Odometry odom_msg;

      //   odom_msg.header.frame_id = fixed_id;
      //   odom_msg.child_frame_id = "base_link";
      //   odom_msg.header.stamp = this->get_clock()->now();

      //    // Set position
      //   odom_msg.pose.pose.position.x = T.translation().x();
      //   odom_msg.pose.pose.position.y = T.translation().y();
      //   odom_msg.pose.pose.position.z = T.translation().z();
      //   odom_msg.pose.covariance = {
      //         0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
      //         0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
      //         0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
      //         0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
      //         0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
      //         0.0, 0.0, 0.0, 0.0, 0.0, 0.01
      //     };

      //   // Set orientation
      //   Eigen::Quaterniond q(T.rotation());
      //   odom_msg.pose.pose.orientation.x = q.x();
      //   odom_msg.pose.pose.orientation.y = q.y();
      //   odom_msg.pose.pose.orientation.z = q.z();
      //   odom_msg.pose.pose.orientation.w = q.w();

      //   odom_publisher->publish(odom_msg);
      // }

      // void publish_tf(){
      //   if (realtime_odometry_transform_publisher_->trylock())
      //     {
      //       auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
      //       odometry_transform_message.transforms.resize(1);
      //       odometry_transform_message.transforms.front().header.frame_id = "odom";
      //       odometry_transform_message.transforms.front().child_frame_id =  "base_link";
      //       auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      //       transform.header.stamp = this->get_clock()->now();
      //       transform.transform.translation.x = T.translation().x();
      //       transform.transform.translation.y = T.translation().y();
      //       Eigen::Quaterniond q(T.rotation());
      //       transform.transform.rotation.x = q.x();
      //       transform.transform.rotation.y = q.y();
      //       transform.transform.rotation.z = q.z();
      //       transform.transform.rotation.w = q.w();
      //       realtime_odometry_transform_publisher_->unlockAndPublish();
      //     }
      // }

      // void publish_pointcloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
      //     sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

      //     // Convert the pcl::PointCloud to the PointCloud2 message
      //     pcl::toROSMsg(*cloud, *cloud_msg);

      //     // Set the header values
      //     cloud_msg->header.stamp = this->get_clock()->now();
      //     cloud_msg->header.frame_id = "base_link"; // Change this to your frame id

      //     // Publish the message
      //     pointcloud_publisher->publish(*cloud_msg);
      //   }

      void timer_callback()
      {
        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        target = voxelgrid_sampling_omp(*(this->points1), 0.25);
        source = voxelgrid_sampling_omp(*(this->points2), 0.25);
        auto target_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*target);
        auto source_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source);
        if (!reg.getInputTarget()) {
                RCLCPP_INFO(this->get_logger(), "No initial cloud found, set as initial");
                reg.setInputTarget(target_cloud);           
        }
        reg.setInputSource(source_cloud);
        // RCLCPP_INFO(this->get_logger(), "Begin regeister");
        reg.align(*(aligned));
        // RCLCPP_INFO(this->get_logger(), "Resgister Finished");
        T = T * Eigen::Isometry3d(reg.getFinalTransformation().cast<double>());
      }
      rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  

  rclcpp::spin(std::make_shared<small_gicp::LidarOdometryNode>());
  
  rclcpp::shutdown();
  return 0;
}