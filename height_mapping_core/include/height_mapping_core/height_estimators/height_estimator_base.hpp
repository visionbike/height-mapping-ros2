#pragma once

#ifndef __HEIGHT_ESTIMATOR_BASE_HPP__
#define __HEIGHT_ESTIMATOR_BASE_HPP__

#include "height_mapping_core/helper.hpp"
#include "height_mapping_core/height_map/cloud_types.hpp"
#include "height_mapping_core/height_map/height_map.hpp"

namespace height_mapping 
{

class HeightEstimatorBase 
{
public:
    using Ptr = std::shared_ptr<HeightEstimatorBase>;

    virtual ~HeightEstimatorBase() = default;

    virtual void estimate(HeightMap &_map, const PointCloud<Point> &_cloud) = 0;
    virtual void estimate(HeightMap &_map, const PointCloud<Laser> &_cloud) = 0;
    virtual void estimate(HeightMap &_map, const PointCloud<Color> &_cloud) = 0;
};
} // namespace height_mapping

#endif // !__HEIGHT_ESTIMATOR_BASE_HPP__