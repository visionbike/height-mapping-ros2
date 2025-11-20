#pragma once

#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <string>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include "height_mapping_ros2/height_mapping/core/core.hpp"

namespace height_mapping_ros2
{

namespace height_mapper
{

    inline height_mapping::HeightMapper::Config loadConfig(const rclcpp::Node::SharedPtr &_node)
    {
        height_mapping::HeightMapper::Config cfg;

        cfg.estimator_type  = _node->declare_parameter<std::string>("estimator_type", "StatMean");
        cfg.frame_id        = _node->declare_parameter<std::string>("frame_id", "map");
        cfg.map_length_x    = _node->declare_parameter<double>("map_length_x", 10.0);
        cfg.map_length_y    = _node->declare_parameter<double>("map_length_y", 10.0);
        cfg.grid_resolution = _node->declare_parameter<double>("grid_resolution", 0.1);
        cfg.height_min      = _node->declare_parameter<double>("min_height_threshold", -0.2);
        cfg.height_max      = _node->declare_parameter<double>("max_height_threshold", 1.5);

        return cfg;
    }

}  // namespace height_mapper

namespace global_mapper
{

    inline height_mapping::GlobalMapper::Config loadConfig(const rclcpp::Node::SharedPtr &_node)
    {
        height_mapping::GlobalMapper::Config cfg;

        cfg.estimator_type = _node->declare_parameter<std::string>("estimator_type", "StatMean");
        cfg.frame_id       = _node->declare_parameter<std::string>("frame_id", "map");
        cfg.map_length_x   = _node->declare_parameter<double>("map_length_x", 200.0);
        cfg.map_length_y   = _node->declare_parameter<double>("map_length_y", 200.0);
        cfg.grid_resolution = _node->declare_parameter<double>("grid_resolution", 0.1);
        cfg.height_min     =  _node->declare_parameter<double>("min_height_threshold", -0.2);
        cfg.height_max     = _node->declare_parameter<double>("max_height_threshold", 1.5);

        // Default save directory: /home/$USER/Downloads
        const char *user_env = std::getenv("USER");
        std::string user = (user_env && *user_env != '\0') ? std::string(user_env) : std::string("user");
        std::string default_dir = "/home/" + user + "/Downloads";

        cfg.map_save_dir = _node->declare_parameter<std::string>("map_save_dir", default_dir);

        return cfg;
    }

}  // namespace global_mapper

}  // namespace height_mapping_ros2

#endif // !__CONFIG_HPP__