#pragma once

#ifndef __HEIGHT_FILTER_FAST__
#define __HEIGHT_FILTER_FAST__

#include "height_mapping_core/height_map/cloud_types.hpp"

namespace height_mapping 
{

class FastHeightFilter 
{
public:
    using Ptr = std::shared_ptr<FastHeightFilter>;

    FastHeightFilter(double _min_z, double _max_z);

    template <typename PointT>
    void filter(const PointCloudPtr<PointT> &_cloud, PointCloudPtr<PointT> &_cloud_filterd)
    {
        _cloud_filterd->clear();
        _cloud_filterd->reserve(_cloud->size());

        for (const auto &point : _cloud->points) {
            if (point.z >= min_z_ && point.z <= max_z_) {
                _cloud_filterd->points.push_back(point);
            }
        }
        _cloud_filterd->header = _cloud->header;
    }

private:
    // set min as infinite minus and max as infinite plus
    static constexpr double INF_MIN = -std::numeric_limits<double>::infinity();
    static constexpr double INF_MAX = std::numeric_limits<double>::infinity();
    double min_z_{INF_MIN}, max_z_{INF_MAX};
};

} // namespace height_mapping

#endif // !__HEIGHT_FILTER_FAST__