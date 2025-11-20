#pragma once

#ifndef __FRAME_IDS_HPP__
#define __FRAME_IDS_HPP__

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace frame_ids
{

namespace sensor
{

inline std::string LIDAR;
inline std::string RGBD;

}  // namespace sensor

	inline std::string ROBOT_BASE;
	inline std::string MAP;

	// Load all frame IDs from ROS2 parameters
	inline void loadFromConfig(const rclcpp::Node::SharedPtr &_node)
	{
		ROBOT_BASE    = _node->declare_parameter<std::string>("robot", "base_link");
		MAP 		  = _node->declare_parameter<std::string>("map", "map");
		sensor::LIDAR = _node->declare_parameter<std::string>("sensor_lidar", "");
		sensor::RGBD  = _node->declare_parameter<std::string>("sensor_rgbd", "");
	}

	// Load all frame IDs from ROS2 parameters
	inline void loadFromConfig(rclcpp::Node &_node)
	{
		ROBOT_BASE    = _node.declare_parameter<std::string>("robot", "base_link");
		MAP 		  = _node.declare_parameter<std::string>("map", "map");
		sensor::LIDAR = _node.declare_parameter<std::string>("sensor_lidar", "");
		sensor::RGBD  = _node.declare_parameter<std::string>("sensor_rgbd", "");
	}

}  // namespace frame_ids

#endif // !__FRAME_IDS_HPP__