#pragma once

#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <iostream>
#include "height_mapping_core/height_map/cloud_types.hpp"


namespace height_mapping
{
    template <typename PointT> 
    bool hasEmptyCloud(const PointCloud<PointT> &_cloud) 
    {
        if (_cloud.empty()) {
            std::cout << "\033[33m[HeightEstimator]: Input cloud is empty! \033[0m" << std::endl;
            return true;
        }
        return false;
    }
}

#endif // !__HELPER_HPP__