#pragma once

#ifndef __GLOBAL_MAPPING_NODE_HPP__
#define __GLOBAL_MAPPING_NODE_HPP__

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

#include <grid_map_core/grid_map_core.hpp>

#include "height_mapping_ros2/common/common.hpp"
#include "height_mapping_ros2/height_mapping/core/core.hpp"
#include "height_mapping_ros2/height_mapping/types/height_point.hpp"
#include "height_mapping_ros2/srv/save_map.hpp"

namespace height_mapping_ros2
{

class GlobalMappingNode : public rclcpp::Node
{
public:
    struct Config
    {
        std::string lidar_cloud_topic;
        std::string rgbd_cloud_topic;
        double map_publish_rate{10.0};
        bool remove_backward_points{false};
        bool debug_mode{false};
        std::string map_save_dir;
        std::string map_save_format{"pcd"};  // e.g., "pcd", "ply", etc.
    } cfg;

    explicit GlobalMappingNode(const rclcpp::NodeOptions &_options = rclcpp::NodeOptions());
    ~GlobalMappingNode() override = default;

    /// Load parameters from this node’s parameter server (ROS2)
    void loadConfig();

private:
    using GridIndexSet = std::unordered_set<grid_map::Index>;

    // init functions
    void initializeTimers_();
    void initializePubSubs_();
    void initializeServices_();

    // callback functions
    void lidarScanCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
    void rgbdScanCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);

    // Core logics
    PointCloudPtr<Laser> processLidarScan_(
        const PointCloudPtr<Laser> &_cloud,
        const geometry_msgs::msg::TransformStamped &_lidar2base,
        const geometry_msgs::msg::TransformStamped &_base2map);

    PointCloudPtr<Color> processRGBDScan_(
        const PointCloudPtr<Color> &_cloud,
        const geometry_msgs::msg::TransformStamped &_camera2base,
        const geometry_msgs::msg::TransformStamped &_base2map);

    void publishPointCloudMap();  // no TimerEvent in ROS2

    // Service callbacks
    void saveMap_(
        const std::shared_ptr<height_mapping_ros2::srv::SaveMap::Request> _request,
        std::shared_ptr<height_mapping_ros2::srv::SaveMap::Response> _response);

    void clearMap_(
        const std::shared_ptr<std_srvs::srv::Empty::Request> _request,
        std::shared_ptr<std_srvs::srv::Empty::Response> _response);

    // Helper functions
    bool toPclCloud_(
        const HeightMap &_map,
        const GridIndexSet &_grid_indices,
        pcl::PointCloud<height_mapping_types::HeightPoint> &_cloud);

    bool savePointCloud(
        const pcl::PointCloud<height_mapping_types::HeightPoint> &_cloud,
        const std::string &_filename);

    bool saveMapToBag_(
        const HeightMap &_map,
        const std::string &_filename);

    void toPointCloud2_(
        const HeightMap &_map,
        const std::vector<std::string> &_layers,
        const GridIndexSet &_grid_indices,
        sensor_msgs::msg::PointCloud2 &_cloud);

    void toMapRegion_(
        const HeightMap &_map,
        visualization_msgs::msg::Marker &_region);

    // Subscribers & Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidarscan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rgbdscan_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_map_region_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_rasterized_;

    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_clear_map_;
    rclcpp::Service<height_mapping_ros2::srv::SaveMap>::SharedPtr srv_save_map_;

    // Timers
    rclcpp::TimerBase::SharedPtr map_publish_timer_;

    // Core mapping object
    std::unique_ptr<height_mapping::GlobalMapper> mapper_;
    TransformOps tf_;  // ROS2 version: constructed with Node::SharedPtr in .cpp

    // State variables
    bool lidarscan_received_{false};
    bool rgbdscan_received_{false};
};

}  // namespace height_mapping_ros2

#endif // !__GLOBAL_MAPPING_NODE_HPP__
