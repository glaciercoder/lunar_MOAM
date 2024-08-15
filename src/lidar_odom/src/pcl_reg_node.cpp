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
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> approximate_policy;
typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;


namespace small_gicp {
class LidarOdometryNode : public rclcpp::Node
{

  public:
      double CorrespondenceRandomness;
      double MaxCorrespondenceDistance;
      double VoxelResolution;
      const int numthreads = 4;
      std::string RegistrationType;
      std::string scan_topic1;
      std::string scan_topic2;
      std::string odom_topic_name;
      std::string odom_parent;
      std::string odom_child;  
      bool publish_tf;
      
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      

      this->get_parameter("CorrespondenceRandomness", CorrespondenceRandomness);
      this->get_parameter("MaxCorrespondenceDistance", MaxCorrespondenceDistance);
      this->get_parameter("VoxelResolution", VoxelResolution);
      this->get_parameter("RegistrationType", RegistrationType);
      this->get_parameter("scan_topic1", scan_topic1);
      this->get_parameter("scan_topic2", scan_topic2);
      this->get_parameter("odom_topic_name", odom_topic_name);
      this->get_parameter("odom_parent", odom_parent);
      this->get_parameter("odom_child", odom_child);
      this->get_parameter("publish_tf", publish_tf);
      points_ready = false;

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");
      RCLCPP_INFO(this->get_logger(), "CorrespondenceRandomness: %.4f", CorrespondenceRandomness);
      RCLCPP_INFO(this->get_logger(), "MaxCorrespondenceDistance: %.4f", MaxCorrespondenceDistance);
      RCLCPP_INFO(this->get_logger(), "VoxelResolution %.4f", VoxelResolution);
      RCLCPP_INFO(this->get_logger(), "RegistrationType: %s", RegistrationType.c_str());
      RCLCPP_INFO(this->get_logger(), "scan_topic1: %s", scan_topic1.c_str());
      RCLCPP_INFO(this->get_logger(), "scan_topic2: %s", scan_topic2.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_parent: %s", odom_parent.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_child: %s", odom_child.c_str());

      // Declare registrator
      reg.setNumThreads(numthreads);
      reg.setCorrespondenceRandomness(CorrespondenceRandomness);
      reg.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
      reg.setVoxelResolution(VoxelResolution);
      reg.setRegistrationType(RegistrationType.c_str());  // or "GICP" (default = "GICP")

      // Transformation matrix
      T = Eigen::Isometry3d::Identity();
      points1.reset(new pcl::PointCloud<pcl::PointXYZ>);
      points2.reset(new pcl::PointCloud<pcl::PointXYZ>);

      // Message Filter
      scan_sub1.subscribe(this, scan_topic1, rmw_qos_profile_system_default);
      scan_sub2.subscribe(this, scan_topic2, rmw_qos_profile_system_default);
      point_sync = std::make_shared<approximate_synchronizer>(approximate_policy(10), scan_sub1 , scan_sub2);
      point_sync->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(200ms));
      point_sync->registerCallback(std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Odom publisher
      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
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

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> scan_sub1;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> scan_sub2;
    std::shared_ptr<approximate_synchronizer> point_sync;

    // Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr points1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    rclcpp::TimerBase::SharedPtr timer_;

    RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
    bool points_ready;
    pcl::PassThrough<pcl::PointXYZ> pass;

    void parameter_initilization() {
      this->declare_parameter<double>("CorrespondenceRandomness", 20);
      this->declare_parameter<double>("MaxCorrespondenceDistance", 1.0);
      this->declare_parameter<double>("VoxelResolution", 1.0);
      this->declare_parameter<std::string>("RegistrationType", "VGICP");
      this->declare_parameter<std::string>("scan_topic1", "robot0/mid360_PointCloud2");
      this->declare_parameter<std::string>("scan_topic2", "robot1/mid360_PointCloud2");
      this->declare_parameter<std::string>("odom_topic_name", "odom");
      this->declare_parameter<std::string>("odom_parent", "robot0/odom");
      this->declare_parameter<std::string>("odom_child", "robot1/odom");
    }

    // Filters 
    enum class Axis
    {
      X = 0,
      Y = 1,
      Z = 2
    };

    // Pass according to the range for a given axis
    void pointcloud_range_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Axis axis, double min_range, double max_range, bool is_negative=false)
    {
      
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



    void scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan_msg1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan_msg2){
      pcl::fromROSMsg(*scan_msg1, *points1);
      pcl::fromROSMsg(*scan_msg2, *points2);
      pointcloud_range_filter(points1, Axis::Z, 0.01, 10.0);
      pointcloud_range_filter(points2, Axis::Z, 0.01, 10.0);
      // if(! points_ready){
      //   pcl::io::savePCDFileASCII ("test_pcd1.pcd", *points1);
      //   pcl::io::savePCDFileASCII ("test_pcd2.pcd", *points2);
      // }
      points_ready = true;
    }

    void publish_relative_odometry() {

      nav_msgs::msg::Odometry odom_msg;

      odom_msg.header.frame_id = odom_parent;
      odom_msg.child_frame_id = odom_child;
      odom_msg.header.stamp = this->get_clock()->now();

        // Set position
      odom_msg.pose.pose.position.x = T.translation().x();
      odom_msg.pose.pose.position.y = T.translation().y();
      odom_msg.pose.pose.position.z = T.translation().z();
      odom_msg.pose.covariance = {
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        };

      // Set orientation
      Eigen::Quaterniond q(T.rotation());
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      odom_publisher->publish(odom_msg);
    }

    void publish_transformation(){
      if (realtime_odometry_transform_publisher_->trylock())
        {
          auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
          odometry_transform_message.transforms.resize(1);
          odometry_transform_message.transforms.front().header.frame_id = odom_parent;
          odometry_transform_message.transforms.front().child_frame_id =  odom_child;
          auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
          transform.header.stamp = this->get_clock()->now();
          transform.transform.translation.x = T.translation().x();
          transform.transform.translation.y = T.translation().y();
          Eigen::Quaterniond q(T.rotation());
          transform.transform.rotation.x = q.x();
          transform.transform.rotation.y = q.y();
          transform.transform.rotation.z = q.z();
          transform.transform.rotation.w = q.w();
          realtime_odometry_transform_publisher_->unlockAndPublish();
        }
    }

    void timer_callback()
    {
      if(points_ready){
      auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      target = voxelgrid_sampling_omp(*(this->points1), 0.25);
      source = voxelgrid_sampling_omp(*(this->points2), 0.25);
      auto target_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*target);
      auto source_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source);
      reg.setInputTarget(target_cloud);
      reg.setInputSource(source_cloud);
      // RCLCPP_INFO(this->get_logger(), "Begin regeister");
      reg.align(*(aligned));
      // RCLCPP_INFO(this->get_logger(), "Resgister Finished");
      T = T * Eigen::Isometry3d(reg.getFinalTransformation().cast<double>());
      // Convert Eigen matrix to string for logging
      // std::stringstream ss;
      // ss << T.matrix();
      // RCLCPP_INFO(this->get_logger(), "Isometry matrix:\n%s", ss.str().c_str());
      publish_relative_odometry();
      publish_transformation();
      points_ready = false;
    }
    }
      
};


}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<small_gicp::LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}