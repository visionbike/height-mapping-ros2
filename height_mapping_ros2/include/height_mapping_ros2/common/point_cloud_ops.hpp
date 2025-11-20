#pragma once

#ifndef __POINT_CLOUD_OPS_HPP__
#define __POINT_CLOUD_OPS_HPP__

#include <cmath>
#include <memory>
#include <string>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "height_mapping_core/height_map/cloud_types.hpp"

class PointCloudOps 
{
public:

	static Eigen::Affine3d toAffine3d(const geometry_msgs::msg::Transform &_t) 
	{
		Eigen::Translation3d translation(_t.translation.x, _t.translation.y, _t.translation.z);
		Eigen::Quaterniond rotation(_t.rotation.w, _t.rotation.x, _t.rotation.y, _t.rotation.z);
		return translation * rotation;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> applyTransform(const PointCloudPtr<PointT> &_input, const geometry_msgs::msg::TransformStamped &_ts) 
	{
		if (!_input || _input->header.frame_id.empty() || _input->empty()) {
			return _input;
		}

		if (_ts.child_frame_id == _ts.header.frame_id) {
			return _input;
		}

		auto affine = toAffine3d(_ts.transform);

		auto output = std::make_shared<PointCloud<PointT>>();
		pcl::transformPointCloud(*_input, *output, affine);
		output->header = _input->header;
		output->header.frame_id = _ts.header.frame_id;
		return output;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> passThrough(const PointCloudPtr<PointT> &_input, const std::string &_field, double _min_value, double _max_value, bool _invert = false) 
	{
		if (!_input || _input->empty()) {
			return _input;
		}

		auto output = std::make_shared<PointCloud<PointT>>();
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(_input);
		pass.setFilterFieldName(_field);
		pass.setFilterLimits(_min_value, _max_value);
		pass.setNegative(_invert);
		pass.filter(*output);
		output->header = _input->header;
		return output;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> filterRange2D(const PointCloudPtr<PointT> &_input, double _min_range, double _max_range) 
	{
		if (!_input || _input->empty()) {
			return _input;
		}

		auto output = std::make_shared<PointCloud<PointT>>();
		output->header = _input->header;
		
		for (const auto &pt : _input->points) {
			double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
			if (r > _min_range && r < _max_range) {
				output->points.push_back(pt);
			}
		}
		return output;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> filterRange3D(const PointCloudPtr<PointT> &_input, double _min_range, double _max_range) 
	{
		if (!_input || _input->empty()) {
			return _input;
		}

		auto output = std::make_shared<PointCloud<PointT>>();
		output->header = _input->header;

		for (const auto &pt : _input->points) {
			double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
			if (r > _min_range && r < _max_range) {
				output->points.push_back(pt);
			}
		}
		return output;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> filterAngle2D(const PointCloudPtr<PointT> &_input, double _start_deg, double _end_deg, bool _invert = false) 
	{
		if (!_input || _input->empty()) {
			return _input;
		}

		auto output = std::make_shared<PointCloud<PointT>>();
		output->header = _input->header;

		const double sRad = normalizeAngle_(_start_deg) * M_PI / 180.0;
		const double eRad = normalizeAngle_(_end_deg) * M_PI / 180.0;

		for (const auto &point : _input->points) {
			double a = std::atan2(point.y, point.x);
			bool keep = (sRad <= eRad) ? (a >= sRad && a <= eRad) : (a >= sRad || a <= eRad);
			if (_invert) { keep = !keep; }
			if (keep) { output->points.push_back(point); }
		}
		return output;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> downsampleVoxel(const PointCloudPtr<PointT> &_input, double _dx, double _dy, double _dz) 
	{
		if (!_input || _input->empty()) {
			return _input;
		}
		
		auto output = std::make_shared<PointCloud<PointT>>();
		pcl::VoxelGrid<PointT> vg;
		vg.setInputCloud(_input);
		vg.setLeafSize(_dx, _dy, _dz);
		vg.filter(*output);
		output->header = _input->header;
		return output;
	}

	template <typename PointT>
	static PointCloudPtr<PointT> downsampleVoxel(const PointCloudPtr<PointT> &_input, double _leaf_size) 
	{
		return downsampleVoxel<PointT>(_input, _leaf_size, _leaf_size, _leaf_size);
	}

private:
	static double normalizeAngle_(double _angle) 
	{
		while (_angle > 180.0) {
			_angle -= 360.0;
		}
		while (_angle < -180.0) {
			_angle += 360.0;
		}
		return _angle;
	}

	// Prevent instantiation
	PointCloudOps() = delete;
	~PointCloudOps() = delete;
	PointCloudOps(const PointCloudOps &) = delete;
	PointCloudOps &operator=(const PointCloudOps &) = delete;
};

#endif // !__POINT_CLOUD_OPS_HPP__