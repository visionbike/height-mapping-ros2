#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "height_mapping_ros2/height_mapping/ros/global_mapping_node.hpp"
#include "height_mapping_ros2/height_mapping/ros/config.hpp"
#include "height_mapping_ros2/common/frame_ids.hpp"

namespace
{

    std::string determineSaveDirectory(const std::string &_requested_directory)
    {
        if (_requested_directory.empty()) {
            // return package share path
            std::string package_path =
            ament_index_cpp::get_package_share_directory("height_mapping_ros2");
            if (!package_path.empty() && package_path.back() != '/') {
                package_path += '/';
            }
            return package_path;
        }

        // Preprocess requested directory ("~", "*/")
        std::string directory = _requested_directory;
        if (!directory.empty() && directory[0] == '~') {
            if (const char * home = std::getenv("HOME")) {
                directory.replace(0, 1, home);
            }
        }
        
        if (!directory.empty() && directory.back() != '/') {
            directory += '/';
        }

        return directory;
    }

    std::string createFilename(const std::string &_directory, const std::string &_requested_filename)
    {
        if (_requested_filename.empty()) {
            auto now   = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::stringstream localtime;
            localtime << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
            return _directory + "elevation_" + localtime.str();
        }

        return _directory + _requested_filename;
    }

}  // namespace

namespace height_mapping_ros2
{

    GlobalMappingNode::GlobalMappingNode(const rclcpp::NodeOptions &_options)
        : rclcpp::Node("global_mapping_node", _options), tf_(this->shared_from_this())
    {
        // Configs from parameters
        loadConfig();

        // Timers / pubs / subs / services
        initializeTimers_();
        initializePubSubs_();
        initializeServices_();

        // TF frame IDs
        frame_ids::loadFromConfig(*this);

        // Global Mapper (config loader already converted to ROS2)
        auto mapper_cfg = global_mapper::loadConfig(this->shared_from_this());
        mapper_ = std::make_unique<height_mapping::GlobalMapper>(mapper_cfg);

        RCLCPP_INFO(this->get_logger(), "[GlobalMappingNode] Global mapping node initialized. Waiting for scan inputs...");
    }

    void GlobalMappingNode::loadConfig()
    {
        cfg.lidar_cloud_topic = this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
        cfg.rgbd_cloud_topic = this->declare_parameter<std::string>("rgbd_topic", "/camera/pointcloud/points");
        cfg.map_publish_rate = this->declare_parameter<double>("map_publish_rate", 10.0);
        cfg.remove_backward_points = this->declare_parameter<bool>("remove_backward_points", false);
        cfg.debug_mode = this->declare_parameter<bool>("debug_mode", false);
        cfg.map_save_format = this->declare_parameter<std::string>("map_save_format", "bag");
    }

    void GlobalMappingNode::initializeTimers_()
    {
        using namespace std::chrono_literals;

        auto period = std::chrono::duration<double>(1.0 / cfg.map_publish_rate);

        map_publish_timer_ = this->create_wall_timer(
            period,
            std::bind(&GlobalMappingNode::publishPointCloudMap, this));
    }

    void GlobalMappingNode::initializePubSubs_()
    {
        auto sensor_qos  = rclcpp::SensorDataQoS();
        auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));

        sub_lidarscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cfg.lidar_cloud_topic, sensor_qos, std::bind(&GlobalMappingNode::lidarScanCallback_, this, std::placeholders::_1));

        sub_rgbdscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cfg.rgbd_cloud_topic, sensor_qos, std::bind(&GlobalMappingNode::rgbdScanCallback_, this, std::placeholders::_1));

        pub_scan_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mapping/global/scan_rasterized", sensor_qos);

        pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mapping/global/map_cloud", default_qos);

        pub_map_region_ = this->create_publisher<visualization_msgs::msg::Marker>("/height_mapping/global/mapping_region", default_qos);
    }

    void GlobalMappingNode::initializeServices_()
    {
        // using std::placeholders::_1;
        // using std::placeholders::_2;

        // srv_save_map_ = this->create_service<height_mapping_ros2::srv::SaveMap>(
        //     "/height_mapping/global/save_map",
        //     std::bind(&GlobalMappingNode::saveMap_, this, _1, _2));

        // srv_clear_map_ = this->create_service<std_srvs::srv::Empty>(
        //     "/height_mapping/global/clear_map",
        //     std::bind(&GlobalMappingNode::clearMap_, this, _1, _2));
    }

    void GlobalMappingNode::lidarScanCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
    {
        if (!lidarscan_received_) {
            frame_ids::sensor::LIDAR = _msg->header.frame_id;
            lidarscan_received_      = true;
            RCLCPP_INFO(this->get_logger(), "[GlobalMappingNode] Pointcloud received! Using LiDAR scans for global mapping...");
        }

        // 1. Get transform matrix using tf tree
        geometry_msgs::msg::TransformStamped lidar2base, base2map;
        if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::LIDAR, lidar2base) ||
            !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
        {
            return;
        }

        // 2. Convert ROS2 msg to PCL data
        auto scan_raw = PointCloudPtr<Laser>(new PointCloud<Laser>());
        pcl::fromROSMsg(*_msg, *scan_raw);

        auto scan_processed = processLidarScan_(scan_raw, lidar2base, base2map);
        if (!scan_processed) {
            return;
        }

        auto scan_rasterized = mapper_->heightMapping<Laser>(scan_processed);

        auto sensor2map = TransformOps::multiplyTransforms(lidar2base, base2map);
        Eigen::Vector3f sensorOrigin3D(
            sensor2map.transform.translation.x,
            sensor2map.transform.translation.y,
            sensor2map.transform.translation.z);
        mapper_->raycasting<Laser>(sensorOrigin3D, scan_processed);

        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::toROSMsg(*scan_rasterized, msg_cloud);
        msg_cloud.header.frame_id = frame_ids::MAP;
        msg_cloud.header.stamp    = _msg->header.stamp;
        pub_scan_rasterized_->publish(msg_cloud);
    }

    void GlobalMappingNode::rgbdScanCallback_(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
    {
        if (!rgbdscan_received_) {
            rgbdscan_received_      = true;
            frame_ids::sensor::RGBD = _msg->header.frame_id;

            RCLCPP_INFO(this->get_logger(), "[GlobalMappingNode] Colored cloud received! Start global mapping...");
        }

        // 1. Get transform matrix using tf tree
        geometry_msgs::msg::TransformStamped camera2base, base2map;
        if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::RGBD, camera2base) ||
            !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
        {
            return;
        }

        // 2. Convert ROS2 msg to PCL data
        auto scan_raw = PointCloudPtr<Color>(new PointCloud<Color>());
        pcl::fromROSMsg(*_msg, *scan_raw);

        auto scan_processed = processRGBDScan_(scan_raw, camera2base, base2map);
        if (!scan_processed) {
            return;
        }

        auto scan_rasterized = mapper_->heightMapping<Color>(scan_processed);

        auto sensor2map = TransformOps::multiplyTransforms(camera2base, base2map);
        Eigen::Vector3f sensorOrigin3D(
            sensor2map.transform.translation.x,
            sensor2map.transform.translation.y,
            sensor2map.transform.translation.z);
        mapper_->raycasting<Color>(sensorOrigin3D, scan_processed);

        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::toROSMsg(*scan_rasterized, msg_cloud);
        msg_cloud.header.frame_id = frame_ids::MAP;
        msg_cloud.header.stamp    = _msg->header.stamp;
        pub_scan_rasterized_->publish(msg_cloud);
    }

    PointCloudPtr<Laser> GlobalMappingNode::processLidarScan_(
        const PointCloudPtr<Laser> &_cloud,
        const geometry_msgs::msg::TransformStamped &_lidar2base,
        const geometry_msgs::msg::TransformStamped &_base2map)
    {
        // 1. Filter local pointcloud
        auto cloud_base = PointCloudOps::applyTransform<Laser>(_cloud, _lidar2base);
        auto cloud_processed = PointCloudPtr<Laser>(new PointCloud<Laser>());

        mapper_->fastHeightFilter<Laser>(cloud_base, cloud_processed);
        cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "x", -10.0, 10.0);
        cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "y", -10.0, 10.0);

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

    PointCloudPtr<Color> GlobalMappingNode::processRGBDScan_(
        const PointCloudPtr<Color> &_cloud,
        const geometry_msgs::msg::TransformStamped &_camera2base,
        const geometry_msgs::msg::TransformStamped &_base2map)
    {
        auto cloud_base = PointCloudOps::applyTransform<Color>(_cloud, _camera2base);
        auto cloud_processed = PointCloudPtr<Color>(new PointCloud<Color>());

        mapper_->fastHeightFilter<Color>(cloud_base, cloud_processed);
        cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "x", -10.0, 10.0);
        cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "y", -10.0, 10.0);

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

    void GlobalMappingNode::publishPointCloudMap()
    {
        const auto &indices = mapper_->getMeasuredGridIndices();
        if (indices.empty()) {
            return;
        }

        std::vector<std::string> layers = {
            height_mapping::layers::Height::HEIGHT,
            height_mapping::layers::Height::HEIGHT_MAX,
            height_mapping::layers::Height::HEIGHT_MIN,
            height_mapping::layers::Height::HEIGHT_VARIANCE,
            height_mapping::layers::Height::NUM_MEASUREMENTS,
        };

        // Visualize global map
        sensor_msgs::msg::PointCloud2 msg_map_cloud;
        toPointCloud2_(
            mapper_->getHeightMap(),
            layers,
            indices,
            msg_map_cloud);
        pub_map_cloud_->publish(msg_map_cloud);

        // Visualize map region
        visualization_msgs::msg::Marker msg_map_region;
        toMapRegion_(mapper_->getHeightMap(), msg_map_region);
        pub_map_region_->publish(msg_map_region);
    }

void GlobalMappingNode::toPointCloud2_(
  const HeightMap & map,
  const std::vector<std::string> & layers,
  const GridIndexSet & measuredIndices,
  sensor_msgs::msg::PointCloud2 & cloud)
{
  // Setup cloud header
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp    = rclcpp::Time(map.getTimestamp());
  cloud.is_dense        = false;

  // Setup field names
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size() + 3);

  fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
  for (const auto & layer : layers) {
    if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  // Setup point field structure
  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (const auto & name : fieldNames) {
    sensor_msgs::msg::PointField field;
    field.name     = name;
    field.count    = 1;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.offset   = offset;
    cloud.fields.push_back(field);
    offset += sizeof(float);
  }

  // Initialize cloud size
  const size_t num_points = measuredIndices.size();
  cloud.height      = 1;
  cloud.width       = num_points;
  cloud.point_step  = offset;
  cloud.row_step    = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Setup point field iterators
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> iterators;
  for (const auto & name : fieldNames) {
    iterators.emplace(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t validPoints = 0;
  for (const auto & index : measuredIndices) {
    grid_map::Position3 position;
    if (!map.getPosition3(height_mapping::layers::Height::HEIGHT, index, position)) {
      continue;
    }

    for (auto & it_pair : iterators) {
      auto & fieldName = it_pair.first;
      auto & iterator  = it_pair.second;

      if (fieldName == "x") {
        *iterator = static_cast<float>(position.x());
      } else if (fieldName == "y") {
        *iterator = static_cast<float>(position.y());
      } else if (fieldName == "z") {
        *iterator = static_cast<float>(position.z());
      } else if (fieldName == "rgb") {
        *iterator = static_cast<float>(map.at("color", index));
      } else {
        *iterator = static_cast<float>(map.at(fieldName, index));
      }
      ++iterator;
    }
    ++validPoints;
  }

  // Adjust final cloud size
  cloud.width    = validPoints;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void GlobalMappingNode::toMapRegion_(
  const height_mapping::HeightMap & map,
  visualization_msgs::msg::Marker & marker)
{
  using namespace std::chrono_literals;

  marker.ns      = "height_map";
  marker.lifetime = rclcpp::Duration(0, 0);  // forever
  marker.action  = visualization_msgs::msg::Marker::ADD;
  marker.type    = visualization_msgs::msg::Marker::LINE_STRIP;

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.header.frame_id = map.getFrameId();
  marker.header.stamp    = rclcpp::Clock().now();

  float length_x_half = (map.getLength().x() - 0.5f * map.getResolution()) / 2.0f;
  float length_y_half = (map.getLength().y() - 0.5f * map.getResolution()) / 2.0f;

  marker.points.resize(5);

  marker.points[0].x = map.getPosition().x() + length_x_half;
  marker.points[0].y = map.getPosition().y() + length_y_half;
  marker.points[0].z = 0.0;

  marker.points[1].x = map.getPosition().x() + length_x_half;
  marker.points[1].y = map.getPosition().y() - length_y_half;
  marker.points[1].z = 0.0;

  marker.points[2].x = map.getPosition().x() - length_x_half;
  marker.points[2].y = map.getPosition().y() - length_y_half;
  marker.points[2].z = 0.0;

  marker.points[3].x = map.getPosition().x() - length_x_half;
  marker.points[3].y = map.getPosition().y() + length_y_half;
  marker.points[3].z = 0.0;

  marker.points[4] = marker.points[0];
}

void GlobalMappingNode::saveMap_(
  const std::shared_ptr<height_mapping_ros2::srv::SaveMap::Request> request,
  std::shared_ptr<height_mapping_ros2::srv::SaveMap::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[GlobalMappingNode] Saving map as %s...",
    cfg.map_save_format.c_str());

  if (mapper_->getMeasuredGridIndices().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[GlobalMappingNode] Height map is empty. Failed to save map.");
    response->success = false;
    return;
  }

  std::string directory = determineSaveDirectory(request->directory);
  std::string filename  = createFilename(directory, request->filename);
  const auto & format   = cfg.map_save_format;
  auto & map            = mapper_->getHeightMap();

  if (format == "pcd") {
    pcl::PointCloud<height_mapping_types::HeightPoint> cloud;
    auto & indices = mapper_->getMeasuredGridIndices();
    if (!toPclCloud_(map, indices, cloud)) {
      response->success = false;
      return;
    }

    bool success = savePointCloud(cloud, filename);
    response->success = success;
    return;
  } else if (format == "bag") {
    // TODO: implement saveMapToBag using rosbag2_cpp if you really need bag output in ROS2.
    RCLCPP_WARN(
      this->get_logger(),
      "[GlobalMappingNode] 'bag' format requested, but rosbag2 support is not implemented. "
      "Please switch to 'pcd' or implement rosbag2 saving.");
    response->success = false;
    return;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "[GlobalMappingNode] Invalid map save format: %s",
      format.c_str());
    response->success = false;
    return;
  }
}

bool GlobalMappingNode::toPclCloud_(
  const HeightMap & map,
  const GridIndexSet & grid_indices,
  pcl::PointCloud<height_mapping_types::HeightPoint> & cloud)
{
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp    = pcl_conversions::toPCL(this->now());
  cloud.points.reserve(grid_indices.size());

  try {
    for (const auto & index : grid_indices) {
      grid_map::Position3 position;
      map.getPosition3(height_mapping::layers::Height::HEIGHT, index, position);

      height_mapping_types::HeightPoint point;
      point.x                = position.x();
      point.y                = position.y();
      point.z                = position.z();
      point.height           =
        map.at(height_mapping::layers::Height::HEIGHT, index);
      point.height_min       =
        map.at(height_mapping::layers::Height::HEIGHT_MIN, index);
      point.height_max       =
        map.at(height_mapping::layers::Height::HEIGHT_MAX, index);
      point.height_variance  =
        map.at(height_mapping::layers::Height::HEIGHT_VARIANCE, index);
      point.num_measurements =
        map.at(height_mapping::layers::Height::NUM_MEASUREMENTS, index);

      cloud.points.push_back(point);
    }

    cloud.width    = cloud.points.size();
    cloud.height   = 1;
    cloud.is_dense = false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error converting height map to point cloud: %s",
      e.what());
    return false;
  }

  return true;
}

bool GlobalMappingNode::savePointCloud(
  const pcl::PointCloud<height_mapping_types::HeightPoint> & cloud,
  const std::string & filename)
{
  std::string pcd_path = filename + ".pcd";
  if (pcl::io::savePCDFileASCII(pcd_path, cloud) == -1) {
    RCLCPP_WARN(
      this->get_logger(),
      "[GlobalMappingNode] Failed to save elevation cloud to %s",
      pcd_path.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[GlobalMappingNode] Elevation cloud saved to %s",
    pcd_path.c_str());
  return true;
}

void GlobalMappingNode::clearMap_(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  mapper_->clearMap();
  RCLCPP_INFO(this->get_logger(), "[GlobalMappingNode] Map cleared.");
}

}  // namespace height_mapping_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<height_mapping_ros2::GlobalMappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
