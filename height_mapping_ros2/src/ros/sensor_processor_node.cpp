#include <pcl_conversions/pcl_conversions.h>

#include "height_mapping_ros2/height_mapping/ros/config.hpp"
#include "height_mapping_ros2/height_mapping/ros/sensor_processor_node.hpp"

namespace height_mapping_ros2
{
    SensorProcessorNode::SensorProcessorNode(const rclcpp::NodeOptions &_options)
        : rclcpp::Node("sensor_processor_node", _options), tf_(this->shared_from_this())   // ROS2 TransformOps uses Node::SharedPtr
    {
        // Load parameters from this node
        loadConfig();

        // Initialize subscribers, synchronizers, publisher
        initializePubSubs_();

        // TF frame IDs (ROS2 helper we converted earlier)
        frame_ids::loadFromConfig(*this);

        RCLCPP_INFO(this->get_logger(), "Sensor processor node initialized. Waiting for %zu rgb clouds...", cfg.input_cloud_topics.size());
    }

    void SensorProcessorNode::loadConfig()
    {
        // Parameters are under this node’s namespace (no private NodeHandle in ROS2)
        cfg.input_cloud_topics = this->declare_parameter<std::vector<std::string>>("input_cloud_topics", {"/sensor1/points", "/sensor2/points", "/sensor3/points"});

        cfg.output_cloud_topic = this->declare_parameter<std::string>("outputcloud_topic", "/sensor_processor/points");

        cfg.cloud_publish_rate = this->declare_parameter<double>("cloud_publish_rate", 10.0);        // [Hz]

        cfg.downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);      // [m/grid]

        cfg.min_range_threshold = this->declare_parameter<double>("min_range_threshold", 0.3);        // [m]

        cfg.max_range_threshold = this->declare_parameter<double>("max_range_threshold", 5.0);        // [m]
    }

    void SensorProcessorNode::initializePubSubs_()
    {
        // Message_filters subscribers with ROS2 node & QoS
        const int queue_size = 10;

        // Use sensor data QoS for point clouds
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

        for (const auto & topic : cfg.input_cloud_topics) {
            auto sub = std::make_shared<CloudSubscriber>(
            this->shared_from_this(), topic, qos_profile);
            sub_clouds_.push_back(sub);
        }

        if (sub_clouds_.size() == 2) {
            sync2_ = std::make_shared<Synchronizer2>(
            SyncPolicy2(queue_size), *sub_clouds_[0], *sub_clouds_[1]);
            sync2_->registerCallback(
            std::bind(&SensorProcessorNode::syncCallback2_, this, std::placeholders::_1, std::placeholders::_2));
        } else if (sub_clouds_.size() == 3) {
            sync3_ = std::make_shared<Synchronizer3>(
            SyncPolicy3(queue_size),
            *sub_clouds_[0], *sub_clouds_[1], *sub_clouds_[2]);
            sync3_->registerCallback(
            std::bind(&SensorProcessorNode::syncCallback3_, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        } else {
            RCLCPP_WARN(this->get_logger(), "SensorProcessorNode expects 2 or 3 input cloud topics, got %zu", sub_clouds_.size());
        }

        // Publisher
        pub_cloud_processed_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cfg.output_cloud_topic, rclcpp::SensorDataQoS());
    }

    void SensorProcessorNode::syncCallback2_(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_msg1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_msg2)
    {
        // Convert to PCL
        pcl::fromROSMsg(*_msg1, *cloud1_);
        pcl::fromROSMsg(*_msg2, *cloud2_);

        // Downsampling
        downsampled1_ = PointCloudOps::downsampleVoxel<Color>(cloud1_, cfg.downsample_resolution);
        downsampled2_ = PointCloudOps::downsampleVoxel<Color>(cloud2_, cfg.downsample_resolution);

        // Free memory
        cloud1_->clear();
        cloud2_->clear();

        // Transform each cloud to baselink frame
        transformToBaselink_(downsampled1_, transformed1_, _msg1->header.frame_id);
        transformToBaselink_(downsampled2_, transformed2_, _msg2->header.frame_id);

        // Free memory
        downsampled1_->clear();
        downsampled2_->clear();

        // Filter each cloud (2D range, XY plane)
        filtered1_ = PointCloudOps::filterRange2D<Color>(transformed1_, cfg.min_range_threshold, cfg.max_range_threshold);
        filtered2_ = PointCloudOps::filterRange2D<Color>(transformed2_, cfg.min_range_threshold, cfg.max_range_threshold);

        // Free memory
        transformed1_->clear();
        transformed2_->clear();

        // Merge transformed clouds
        *filtered1_ += *filtered2_;

        // Convert back to ROS2 message
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*filtered1_, msg);
        msg.header.frame_id = frame_ids::ROBOT_BASE;
        msg.header.stamp    = _msg1->header.stamp;  // Use timestamp from first message

        pub_cloud_processed_->publish(msg);

        // Free memory
        filtered1_->clear();
        filtered2_->clear();
    }

    void SensorProcessorNode::syncCallback3_(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & _msg1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & _msg2,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & _msg3)
    {
        // Convert to PCL
        pcl::fromROSMsg(*_msg1, *cloud1_);
        pcl::fromROSMsg(*_msg2, *cloud2_);
        pcl::fromROSMsg(*_msg3, *cloud3_);

        // Downsampling
        downsampled1_ = PointCloudOps::downsampleVoxel<Color>(cloud1_, cfg.downsample_resolution);
        downsampled2_ = PointCloudOps::downsampleVoxel<Color>(cloud2_, cfg.downsample_resolution);
        downsampled3_ = PointCloudOps::downsampleVoxel<Color>(cloud3_, cfg.downsample_resolution);

        // Free memory
        cloud1_->clear();
        cloud2_->clear();
        cloud3_->clear();

        // Transform each cloud to baselink frame
        transformToBaselink_(downsampled1_, transformed1_, _msg1->header.frame_id);
        transformToBaselink_(downsampled2_, transformed2_, _msg2->header.frame_id);
        transformToBaselink_(downsampled3_, transformed3_, _msg3->header.frame_id);

        // Free memory
        downsampled1_->clear();
        downsampled2_->clear();
        downsampled3_->clear();

        // Filter each cloud
        filtered1_ = PointCloudOps::filterRange2D<Color>(transformed1_, cfg.min_range_threshold, cfg.max_range_threshold);
        filtered2_ = PointCloudOps::filterRange2D<Color>(transformed2_, cfg.min_range_threshold, cfg.max_range_threshold);
        filtered3_ = PointCloudOps::filterRange2D<Color>(transformed3_, cfg.min_range_threshold, cfg.max_range_threshold);

        // Free memory
        transformed1_->clear();
        transformed2_->clear();
        transformed3_->clear();

        // Merge transformed clouds
        *filtered1_ += *filtered2_;
        *filtered1_ += *filtered3_;

        // Convert back to ROS2 message
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*filtered1_, msg);
        msg.header.frame_id = frame_ids::ROBOT_BASE;
        msg.header.stamp    = _msg1->header.stamp;

        pub_cloud_processed_->publish(msg);

        // Free memory
        filtered1_->clear();
        filtered2_->clear();
        filtered3_->clear();
    }

    void SensorProcessorNode::transformToBaselink_(
        const pcl::PointCloud<Color>::Ptr &_cloud,
        pcl::PointCloud<Color>::Ptr &_cloud_transformed,
        const std::string &_sensor_frame)
    {
        // Get transform map/base_link or robot_base/sensor_frame
        geometry_msgs::msg::TransformStamped transform;
        if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, _sensor_frame, transform)) {
            return;
        }

        // Transform point cloud
        _cloud_transformed = PointCloudOps::applyTransform<Color>(_cloud, transform);
    }

} // namespace height_mapping

// Standalone executable (optional; you can also use components)
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<height_mapping_ros2::SensorProcessorNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}