#pragma once

#ifndef __TRANSFORM_OPS_HPP__
#define __TRANSFORM_OPS_HPP__

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TransformOps 
{
public:
    explicit TransformOps(const rclcpp::Node::SharedPtr &_node)
        : node_(_node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_) {}


    /**
     * @brief Attempts to lookup the transform between two frames.
     * @param target_frame The target frame.
     * @param source_frame The source frame.
     * @param transform Output variable for the transform.
     * @param time Time at which to lookup transform.
     * @return True if successful, false otherwise.
     */
    bool lookupTransform(const std::string &_frame_target, const std::string &_frame_source, geometry_msgs::msg::TransformStamped &_transform, const rclcpp::Time &_time = rclcpp::Time(0, 0, RCL_ROS_TIME), double _timeout_sec = 0.1)
    {
        try {
            // rclcpp::Duration from seconds
            _transform = tf_buffer_.lookupTransform(_frame_target, _frame_source, _time, rclcpp::Duration::from_seconds(_timeout_sec));
            return true;
        } 
        catch (const tf2::TransformException &e) 
        {
            // 1000 throttle period in ms
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "TF lookup failed (%s -> %s): %s", _frame_source.c_str(), _frame_target.c_str(), e.what());
            return false;
        }
    }

    static geometry_msgs::msg::TransformStamped multiplyTransforms(const geometry_msgs::msg::TransformStamped &_transform1, const geometry_msgs::msg::TransformStamped &_transform2)
    {
        tf2::Transform t1, t2;
        tf2::fromMsg(_transform1.transform, t1);
        tf2::fromMsg(_transform2.transform, t2);

        tf2::Transform t_multiplied = t1 * t2;

        geometry_msgs::msg::TransformStamped transform_multiplied;
        transform_multiplied.transform = tf2::toMsg(t_multiplied);
        transform_multiplied.header.frame_id = _transform2.header.frame_id;
        transform_multiplied.header.stamp = _transform2.header.stamp;
        transform_multiplied.child_frame_id = _transform1.child_frame_id;
        return transform_multiplied;
    }

private:
    // No default constructor in ROS2 (we want a node for clock & logger)
    TransformOps() = delete;
    
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif // !__TRANSFORM_OPS_HPP__