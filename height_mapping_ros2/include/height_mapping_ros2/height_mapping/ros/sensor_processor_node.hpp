#pragma once

#ifndef __SENSOR_PROCESSOR_NODE_HPP__
#define __SENSOR_PROCESSOR_NODE_HPP__

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "height_mapping_ros2/common/common.hpp"            // keep as-is if still valid in ROS2
#include "height_mapping_ros2/height_mapping/core/core.hpp" // for Color, TransformOps, etc.

namespace height_mapping_ros2
{

class SensorProcessorNode : public rclcpp::Node
{
public:
    struct Config
    {
        std::vector<std::string> input_cloud_topics;
        std::string output_cloud_topic;
        double cloud_publish_rate{15.0};
        double downsample_resolution{0.0};
        double min_range_threshold{0.3};
        double max_range_threshold{10.0};
    } cfg;

    explicit SensorProcessorNode(const rclcpp::NodeOptions &_options = rclcpp::NodeOptions());
    ~SensorProcessorNode() override = default;

    /// Load parameters from this node’s parameter server (ROS2)
    void loadConfig();

private:
    void initializePubSubs_();

    // Synchronized callbacks (ROS2 messages)
    void syncCallback2_(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_msg1, 
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_msg2);

    void syncCallback3_(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & _msg1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & _msg2,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & _msg3);

    void transformToBaselink_(
        const PointCloudPtr<Color> &_cloud,
        PointCloudPtr<Color> &_cloud_transformed,
        const std::string &_sensor_frame);

    // Message filters types
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    using CloudSubscriber = message_filters::Subscriber<CloudMsg>;

    using SyncPolicy2 = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;
    using SyncPolicy3 = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg, CloudMsg>;

    using Synchronizer2 = message_filters::Synchronizer<SyncPolicy2>;
    using Synchronizer3 = message_filters::Synchronizer<SyncPolicy3>;

    // Subscribers and synchronizers
    std::vector<std::shared_ptr<CloudSubscriber>> sub_clouds_;
    std::shared_ptr<Synchronizer2> sync2_;
    std::shared_ptr<Synchronizer3> sync3_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_processed_;

    // Core implementation (ROS2 TransformOps uses rclcpp::Node::SharedPtr)
    TransformOps tf_;

    // pcl pointers
    PointCloudPtr<Color> cloud1_{new PointCloud<Color>};
    PointCloudPtr<Color> cloud2_{new PointCloud<Color>};
    PointCloudPtr<Color> cloud3_{new PointCloud<Color>};

    PointCloudPtr<Color> downsampled1_{new PointCloud<Color>};
    PointCloudPtr<Color> downsampled2_{new PointCloud<Color>};
    PointCloudPtr<Color> downsampled3_{new PointCloud<Color>};

    PointCloudPtr<Color> filtered1_{new PointCloud<Color>};
    PointCloudPtr<Color> filtered2_{new PointCloud<Color>};
    PointCloudPtr<Color> filtered3_{new PointCloud<Color>};

    PointCloudPtr<Color> transformed1_{new PointCloud<Color>};
    PointCloudPtr<Color> transformed2_{new PointCloud<Color>};
    PointCloudPtr<Color> transformed3_{new PointCloud<Color>};
};

}  // namespace height_mapping_ros

#endif // !__SENSOR_PROCESSOR_NODE_HPP__