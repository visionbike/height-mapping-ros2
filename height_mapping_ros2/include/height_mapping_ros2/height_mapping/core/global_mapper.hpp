#pragma once

#ifndef __GLOBAL_MAPPER_HPP__
#define __GLOBAL_MAPPER_HPP__

#include <memory>
#include <unordered_set>

    #include "height_mapping_ros2/height_mapping/core/height_mapper.hpp"

    // this is for the use of unordered_set with grid_map::Index
    namespace std 
    {
    template <> 
    struct hash<grid_map::Index> 
    {
        std::size_t operator()(const grid_map::Index &_index) const 
        {
            std::size_t h1 = std::hash<int>{}(_index[0]);
            std::size_t h2 = std::hash<int>{}(_index[1]);
            return h1 ^ (h2 << 1);
        }
    };

    template <> 
    struct equal_to<grid_map::Index> 
    {
        bool operator()(const grid_map::Index &_lhs, const grid_map::Index &_rhs) const 
        {
            return (_lhs[0] == _rhs[0]) && (_lhs[1] == _rhs[1]);
        }
    };

} // namespace std

namespace height_mapping 
{

class GlobalMapper : public HeightMapper 
{
public:
    struct Config : public HeightMapper::Config 
    {
        std::string map_save_dir;
    } cfg;

    GlobalMapper(const Config &_cfg);

    template <typename PointT>
    PointCloudPtr<PointT> heightMapping(const PointCloudPtr<PointT> &_cloud)
    {
        auto cloud_rasterized = HeightMapper::heightMapping<PointT>(_cloud);

        if (!cloud_rasterized || cloud_rasterized->empty()) {
            return cloud_rasterized;
        }

        // Save measured indices for efficiency
        recordMeasuredCells_<PointT>(getHeightMap(), *cloud_rasterized);

        return cloud_rasterized;
    }

    const std::unordered_set<grid_map::Index> &getMeasuredGridIndices() const;

private:
    template <typename PointT>
    void recordMeasuredCells_(const height_mapping::HeightMap &_map, const PointCloud<PointT> &_cloud)
    {
        grid_map::Index index_measured_cell;
        grid_map::Position position_measured_cell;

        for (const auto & point : _cloud.points) {
            position_measured_cell.x() = point.x;
            position_measured_cell.y() = point.y;

            // Skip if the point is out of the map
            if (!_map.getIndex(position_measured_cell, index_measured_cell)) {
                continue;
            }

            if (_map.isEmptyAt(index_measured_cell)) {
                continue;
            }

            measured_indices_.insert(index_measured_cell);
        }
    }

    std::unordered_set<grid_map::Index> measured_indices_;
};
} // namespace height_mapping

#endif // !__GLOBAL_MAPPER_HPP__
