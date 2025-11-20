#pragma once

#ifndef __HEIGHT_POINT_HPP__
#define __HEIGHT_POINT_HPP__

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace height_mapping_types
{

	struct EIGEN_ALIGN16 HeightPoint
	{
		PCL_ADD_POINT4D;  // adds the members x,y,z and padding
		float height;
		float height_min;
		float height_max;
		float height_variance;
		float num_measurements;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	} EIGEN_ALIGN16;

}  // namespace height_mapping_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
	height_mapping_types::HeightPoint,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, height, height)
	(float, height_min, height_min)
	(float, height_max, height_max)
	(float, height_variance, height_variance)
	(float, num_measurements, num_measurements)
)

#endif // !__HEIGHT_POINT_HPP__