#pragma once

#ifndef __HEIGHT_MAPPING_NODE_HPP__
#define __HEIGHT_MAPPING_NODE_HPP__

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "height_mapping_ros2/common/common.hpp"
#include "height_mapping_ros2/height_mapping/ros/config.hpp"

namespace height_mapping_ros2
{

class HeightMappingNode : public rclcpp::Node
{
public:
    struct Config
    {
        std::string lidar_cloud_topic;
        std::string rgbd_cloud_topic;
        double pose_update_rate{15.0};   // [Hz]
        double map_publish_rate{15.0};   // [Hz]
        bool remove_backward_points{false};
        bool debug_mode{false};
    } cfg;

    explicit HeightMappingNode(const rclcpp::NodeOptions &_options = rclcpp::NodeOptions());
    ~HeightMappingNode() override = default;

    // Load parameters from this node’s parameter server (ROS2)
    void loadConfig();

private:
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

    void publishRasterizedLidarScan_(const PointCloudPtr<Laser> &_scan);
    void publishRasterizedRGBDScan_(const PointCloudPtr<Color> &_scan);

    // Timers (no TimerEvent in ROS2)
    void updateMapOrigin_();
    void publishHeightMap_();

    // Subscribers & Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidarscan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rgbdscan_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidarscan_rasterized_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_rgbdscan_rasterized_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_heightmap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_rgbd_;

    // Timers
    rclcpp::TimerBase::SharedPtr pose_update_timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;

    // Core objects
    std::unique_ptr<height_mapping::HeightMapper> mapper_;
    TransformOps tf_;   // ROS2 version: constructed with Node::SharedPtr in .cpp

    // State variables
    bool lidarscan_received_{false};
    bool rgbdscan_received_{false};
};

}  // namespace height_mapping_ros

#endif  // !__HEIGHT_MAPPING_NODE_HPP__