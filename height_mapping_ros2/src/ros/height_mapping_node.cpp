#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "height_mapping_ros2/height_mapping/ros/height_mapping_node.hpp"

namespace height_mapping_ros2
{

    HeightMappingNode::HeightMappingNode(const rclcpp::NodeOptions & _options)
        : rclcpp::Node("height_mapping_node", _options), tf_(this->shared_from_this())  // ROS2 TransformOps
    {
        // Load configuration from parameters (ROS2)
        loadConfig();

        // Subscribers / publishers / timers / services
        initializePubSubs_();
        initializeTimers_();
        initializeServices_();

        // TF frame IDs
        frame_ids::loadFromConfig(*this);

        // Height Mapper config (ROS2 helper you converted earlier)
        auto mapper_cfg = height_mapper::loadConfig(this->shared_from_this());
        mapper_ = std::make_unique<height_mapping::HeightMapper>(mapper_cfg);

        RCLCPP_INFO(this->get_logger(), "[HeightMappingNode] Height mapping node initialized. Waiting for scan inputs...");
    }

    void HeightMappingNode::loadConfig()
    {
        cfg.lidar_cloud_topic = this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
        cfg.rgbd_cloud_topic = this->declare_parameter<std::string>("rgbd_topic", "/camera/pointcloud/points");
        cfg.pose_update_rate = this->declare_parameter<double>("pose_update_rate", 15.0);
        cfg.map_publish_rate = this->declare_parameter<double>("map_publish_rate", 10.0);
        cfg.remove_backward_points = this->declare_parameter<bool>("remove_backward_points", false);
        cfg.debug_mode = this->declare_parameter<bool>("debug_mode", false);
    }

    void HeightMappingNode::initializePubSubs_()
    {
        // QoS profiles
        auto sensor_qos = rclcpp::SensorDataQoS();
        auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // Subscribers
        sub_lidarscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg.lidar_cloud_topic, sensor_qos,
            std::bind(&HeightMappingNode::lidarScanCallback_, this, std::placeholders::_1));

        sub_rgbdscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg.rgbd_cloud_topic, sensor_qos,
            std::bind(&HeightMappingNode::rgbdScanCallback_, this, std::placeholders::_1));

        // Publishers
        pub_lidarscan_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mapping/local/scan_rasterized_lidar", sensor_qos);

        pub_rgbdscan_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mapping/local/scan_rasterized_rgbd", sensor_qos);

        pub_heightmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/height_mapping/local/map", default_qos);

        if (cfg.debug_mode) {
            pub_debug_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mapping/local/debug_lidar", sensor_qos);
            pub_debug_rgbd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mapping/local/debug_rgbd", sensor_qos);
        }
    }

    void HeightMappingNode::initializeTimers_()
    {
        using namespace std::chrono_literals;

        auto pose_period = std::chrono::duration<double>(1.0 / cfg.pose_update_rate);
        auto map_period  = std::chrono::duration<double>(1.0 / cfg.map_publish_rate);

        pose_update_timer_ = this->create_wall_timer(pose_period, std::bind(&HeightMappingNode::updateMapOrigin_, this));

        map_publish_timer_ = this->create_wall_timer(map_period, std::bind(&HeightMappingNode::publishHeightMap_, this));
    }

    void HeightMappingNode::initializeServices_()
    {
    // TODO: port global save/clear services if needed
    // Example:
    // srv_save_map_ = this->create_service<...>(
    //   "/height_mapping/global/save_map",
    //   std::bind(&HeightMappingNode::saveMapCallback, this, _1, _2));
    }

    void HeightMappingNode::lidarScanCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
    {
        if (!lidarscan_received_) {
            frame_ids::sensor::LIDAR = _msg->header.frame_id;
            lidarscan_received_ = true;

            RCLCPP_INFO(this->get_logger(), "[HeightMappingNode] Pointcloud received! Using LiDAR scans for height mapping...");
        }

        // 1. Get transform matrices using TF tree
        geometry_msgs::msg::TransformStamped lidar2base, base2map;
        if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::LIDAR, lidar2base) ||
            !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
        {
            return;
        }

        // 2. Convert ROS2 msg to PCL data
        auto scan_raw = pcl::PointCloud<Laser>::Ptr(new pcl::PointCloud<Laser>());
        pcl::fromROSMsg(*_msg, *scan_raw);

        // 3. Preprocess scan data: ready for terrain mapping
        auto scan_preprocessed = processLidarScan_(scan_raw, lidar2base, base2map);
        if (!scan_preprocessed) {
            return;
        }

        // 4. Height mapping
        auto scan_rasterized = mapper_->heightMapping<Laser>(scan_preprocessed);

        // 5. Publish pointcloud used for mapping
        publishRasterizedLidarScan_(scan_rasterized);

        // 6. Raycasting correction: remove dynamic objects
        auto sensor2map = TransformOps::multiplyTransforms(lidar2base, base2map);
        Eigen::Vector3f sensorOrigin3D(
            sensor2map.transform.translation.x,
            sensor2map.transform.translation.y,
            sensor2map.transform.translation.z);
        mapper_->raycasting<Laser>(sensorOrigin3D, scan_preprocessed);

        // Debug: publish pointcloud that you want to see
        if (cfg.debug_mode && pub_debug_lidar_) {
            sensor_msgs::msg::PointCloud2 msg_debug;
            pcl::toROSMsg(*scan_preprocessed, msg_debug);
            msg_debug.header.frame_id = frame_ids::MAP;
            msg_debug.header.stamp    = _msg->header.stamp;
            pub_debug_lidar_->publish(msg_debug);
        }
    }

    void HeightMappingNode::rgbdScanCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
    {
        // First time receiving RGB-D cloud
        if (!rgbdscan_received_) {
            rgbdscan_received_ = true;
            RCLCPP_INFO(this->get_logger(), "[HeightMappingNode] Colored cloud received! Using RGB-D sensors for height mapping...");
        }

        // Get Transform matrices
        frame_ids::sensor::RGBD = _msg->header.frame_id;

        geometry_msgs::msg::TransformStamped camera2base, base2map;
        if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::RGBD, camera2base) ||
            !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
        {
            return;
        }

        // Prepare pointcloud
        auto scan_raw = pcl::PointCloud<Color>::Ptr(new pcl::PointCloud<Color>());
        pcl::fromROSMsg(*_msg, *scan_raw);

        // Preprocess pointcloud
        auto scan_processed = processRGBDScan_(scan_raw, camera2base, base2map);
        if (!scan_processed) {
            return;
        }

        // Mapping
        auto cloud_mapped = mapper_->heightMapping<Color>(scan_processed);

        // Publish pointcloud used for mapping
        publishRasterizedRGBDScan_(cloud_mapped);

        // Debug: publish pointcloud that you want to see
        if (cfg.debug_mode && pub_debug_rgbd_) {
            sensor_msgs::msg::PointCloud2 debugMsg;
            pcl::toROSMsg(*scan_processed, debugMsg);
            debugMsg.header.frame_id = frame_ids::MAP;
            debugMsg.header.stamp    = _msg->header.stamp;
            pub_debug_rgbd_->publish(debugMsg);
        }
    }

    PointCloudPtr<Laser> HeightMappingNode::processLidarScan_(
        const PointCloudPtr<Laser> &_cloud,
        const geometry_msgs::msg::TransformStamped &_lidar2base,
        const geometry_msgs::msg::TransformStamped &_base2map)
    {
        // 1. Filter local pointcloud
        auto cloud_base = PointCloudOps::applyTransform<Laser>(_cloud, _lidar2base);
        auto cloud_processed = PointCloudPtr<Laser>(new PointCloud<Laser>());

        mapper_->fastHeightFilter<Laser>(cloud_base, cloud_processed);

        auto range = mapper_->getHeightMap().getLength() / 2.0;  // mapping range
        cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "x", -range.x(), range.x());
        cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "y", -range.y(), range.y());

        // (Optional) Remove backward points
        if (cfg.remove_backward_points) {
            cloud_processed = PointCloudOps::filterAngle2D<Laser>(cloud_processed, -135.0, 135.0);
        }

        // 2. Transform pointcloud to map frame
        cloud_processed = PointCloudOps::applyTransform<Laser>(cloud_processed, _base2map);

        if (cloud_processed->empty()) {
            return nullptr;
        }
        return cloud_processed;
    }

    PointCloudPtr<Color> HeightMappingNode::processRGBDScan_(
        const PointCloudPtr<Color> &_cloud,
        const geometry_msgs::msg::TransformStamped &_camera2base,
        const geometry_msgs::msg::TransformStamped &_base2map)
    {
        auto cloud_base = PointCloudOps::applyTransform<Color>(_cloud, _camera2base);
        auto cloud_processed = PointCloudPtr<Color>(new PointCloud<Color>());

        mapper_->fastHeightFilter<Color>(cloud_base, cloud_processed);

        auto range = mapper_->getHeightMap().getLength() / 2.0;  // mapping range
        cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "x", -range.x(), range.x());
        cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "y", -range.y(), range.y());

        // (Optional) Remove backward points
        if (cfg.remove_backward_points) {
            cloud_processed = PointCloudOps::filterAngle2D<Color>(cloud_processed, -135.0, 135.0);
        }

        cloud_processed = PointCloudOps::applyTransform<Color>(cloud_processed, _base2map);

        if (cloud_processed->empty()) {
            return nullptr;
        }
        return cloud_processed;
    }

    void HeightMappingNode::updateMapOrigin_()
    {
        geometry_msgs::msg::TransformStamped base2map;
        if (!tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map)) {
            return;
        }

        // Update map origin
        grid_map::Position robot_position(base2map.transform.translation.x, base2map.transform.translation.y);
        mapper_->moveMapOrigin(robot_position);
    }

    void HeightMappingNode::publishHeightMap_()
    {
        auto msg_ptr = grid_map::GridMapRosConverter::toMessage(mapper_->getHeightMap());
        msg_ptr->header.frame_id = frame_ids::MAP;
        msg_ptr->header.stamp = this->now();
        pub_heightmap_->publish(*msg_ptr);
    }

    void HeightMappingNode::publishRasterizedLidarScan_(const pcl::PointCloud<Laser>::Ptr &_scan)
    {
        if (!_scan || _scan->empty()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*_scan, msg);
        msg.header.frame_id = frame_ids::MAP;
        msg.header.stamp    = this->now();
        pub_lidarscan_rasterized_->publish(msg);
    }

    void HeightMappingNode::publishRasterizedRGBDScan_(const pcl::PointCloud<Color>::Ptr & _scan)
    {
        if (!_scan || _scan->empty()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*_scan, msg);
        msg.header.frame_id = frame_ids::MAP;
        msg.header.stamp    = this->now();
        pub_rgbdscan_rasterized_->publish(msg);
    }

}  // namespace height_mapping_ros

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<height_mapping_ros2::HeightMappingNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
