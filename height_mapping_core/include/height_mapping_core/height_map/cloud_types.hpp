#pragma once

#ifndef __CLOUD_TYPES_HPP__
#define __CLOUD_TYPES_HPP__

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using Point = pcl::PointXYZ;
using Laser = pcl::PointXYZI;
using Color = pcl::PointXYZRGB;

template<typename PointT> using PointCloud = pcl::PointCloud<PointT>;
template<typename PointT> using PointCloudPtr = std::shared_ptr<PointCloud<PointT>>;

#endif // !__CLOUD_TYPES_HPP__
