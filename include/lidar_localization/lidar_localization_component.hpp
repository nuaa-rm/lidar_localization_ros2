#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "lidar_localization/lidar_undistortion.hpp"

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit PCLLocalization(const rclcpp::NodeOptions &options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);

    void initializeParameters();
    void initializePubSub();
    void initializeRegistration();
    void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void cloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // void gnssReceived();

    tf2_ros::TransformBroadcaster broadcaster_;
    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener tflistener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
        initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
        cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr
        imu_sub_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialize_pose_;
    boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
    geometry_msgs::msg::TransformStamped transform_stamped;

    bool map_recieved_{false};
    bool initialpose_recieved_{false};
    bool set_initial_pose_{false};
    bool locate_no_map_{false};

    // parameters
    std::string global_frame_id_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string registration_method_;
    std::string point_topic;
    std::string imu_topic;
    double scan_max_range_;
    double scan_min_range_;
    double scan_period_;
    double score_threshold_;
    double ndt_resolution_;
    double ndt_step_size_;
    double transform_epsilon_;
    double voxel_leaf_size_;
    std::string pcd_path;
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_z_;
    double initial_pose_qx_;
    double initial_pose_qy_;
    double initial_pose_qz_;
    double initial_pose_qw_;
    double frequency_;
    double cloud_frequency_;
    double static_time_threshold_;
    double static_range_threshold_;
    double force_relocate_time_threshold_;
    double integral_time_;

    double last_odom_received_time_;
    bool use_imu_{false};
    bool enable_debug_{false};
    int integral_count_;

    int ndt_num_threads_;
    int skip_count_;
    int static_count_;
    geometry_msgs::msg::TransformStamped lasttransformStamped;
    pcl::PointCloud<pcl::PointXYZI>::Ptr integral_cloud_;

    // imu
    LidarUndistortion lidar_undistortion_;
};
