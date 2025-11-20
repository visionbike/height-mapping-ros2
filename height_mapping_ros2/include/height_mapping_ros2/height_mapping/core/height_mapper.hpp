#pragma once

#ifndef __HEIGHT_MAPPER_HPP__
#define __HEIGHT_MAPPER_HPP__

#include <memory>
#include <height_mapping_core/height_mapping_core.hpp>

namespace height_mapping 
{

class HeightMapper 
{
public:
    struct Config 
    {
        std::string estimator_type;
        std::string frame_id;
        double map_length_x;
        double map_length_y;
        double grid_resolution;
        double height_min;
        double height_max;
    } cfg;

    HeightMapper(const Config &_cfg);

    /**
     * @brief Gridded height mapping using height estimator
     * @param _cloud: input pointcloud
     * @return filtered pointcloud
     */
    template <typename PointT>
    PointCloudPtr<PointT> heightMapping(const PointCloudPtr<PointT> &_cloud)
    {
        // 1. Rasterize pointcloud
        auto cloud_rasterized = cloudRasterization_<PointT>(_cloud, static_cast<float>(cfg.grid_resolution));
        if (!cloud_rasterized || cloud_rasterized->empty()) {
            std::cout << "\033[1;31m[height_mapping::HeightMapper]: Height estimation failed. Rasterized cloud is empty\033[0m\n";
            return nullptr;
        }

        // 2. Update height map
        height_estimator_->estimate(map_, *cloud_rasterized);
        return cloud_rasterized;
    }

    /*
    * Correct heightmap using raycasting
    * @param pointcloud: pointcloud for raycasting [ref: map frame]
    * @param _sensor_origin: sensor origin [ref: map frame]
    */
    template <typename PointT>
    void raycasting(const Eigen::Vector3f &_sensor_origin, const PointCloudPtr<PointT> &_cloud)
    {
        if (!_cloud || _cloud->empty()) {
            return;
        }
        raycaster_.correctHeight(map_, *_cloud, _sensor_origin);
    }

    /**
     * @brief Fast height filtering
     * @param _cloud: input pointcloud
     * @param filtered_cloud: output filtered pointcloud
     */
    template <typename PointT>
    void fastHeightFilter(const PointCloudPtr<PointT> &_cloud, PointCloudPtr<PointT> &_cloud_filtered)
    {
        height_filter_.filter<PointT>(_cloud, _cloud_filtered);
    }

    /**
     * @brief Move heightmap origin
     * @param position: new origin position in map frame
     */
    void moveMapOrigin(const grid_map::Position &_position);

    /**
     * @brief Get heightmap
     * @return heightmap
     */
    const HeightMap &getHeightMap() const;

    HeightMap &getHeightMap();

    /**
     * @brief Set heightmap origin
     * @param _position: new origin position in map frame
     */
    void setMapPosition(const grid_map::Position &_position);

    /**
     * @brief Clear heightmap
     */
    void clearMap();

private:
    void initMap_();
    void initHeightEstimator_();

    template <typename PointT>
    PointCloudPtr<PointT> cloudRasterization_(const PointCloudPtr<PointT> &_cloud, float _grid_size)
    {
        if (!_cloud || _cloud->empty()) {
            return _cloud;
        }

        // grid cell: first, second -> (x_index, y_index), point
        std::unordered_map<std::pair<int, int>, PointT, pair_hash_> grid_map_cells;
        for (const auto & point : *_cloud) {
            int x_index = static_cast<int>(std::floor(point.x / _grid_size));
            int y_index = static_cast<int>(std::floor(point.y / _grid_size));

            auto grid_key = std::make_pair(x_index, y_index);
            auto [iter, inserted] = grid_map_cells.try_emplace(grid_key, point);

            if (!inserted && point.z > iter->second.z) {
            iter->second = point;
            }
        }

        auto cloud_downsampled = std::make_shared<PointCloud<PointT>>();
        cloud_downsampled->reserve(grid_map_cells.size());
        for (const auto & grid_cell : grid_map_cells) {
            cloud_downsampled->points.emplace_back(grid_cell.second);
        }

        cloud_downsampled->header = _cloud->header;
        return cloud_downsampled;
    }

    template <typename PointT>
    PointCloudPtr<PointT> cloudRasterizationAlt_(const PointCloudPtr<PointT> &_cloud, float _grid_size)
    {
        if (!_cloud || _cloud->empty()) {
            return _cloud;
        }

        // trick for unused parameter
        (void) _grid_size;

        std::unordered_map<std::pair<int, int>, PointT, pair_hash_> grid_cells;
        grid_map::Position position_measured;
        grid_map::Index index_measured;

        for (const auto & point : *_cloud) {
            position_measured << point.x, point.y;

            if (!map_.getIndex(position_measured, index_measured)) {
                continue;  // Skip points outside map bounds
            }

            auto grid_index = std::make_pair(index_measured.x(), index_measured.y());
            auto [iter, inserted] = grid_cells.try_emplace(grid_index, point);

            if (inserted) {
                // If new point, get exact grid center position
                iter->second = point;
                map_.getPosition(index_measured, position_measured);
                iter->second.x = position_measured.x();
                iter->second.y = position_measured.y();
            } else if (point.z > iter->second.z) {
                // If higher point, update z while keeping grid center position
                iter->second = point;
                map_.getPosition(index_measured, position_measured);
                iter->second.x = position_measured.x();
                iter->second.y = position_measured.y();
            }
        }

        auto cloud_downsampled = std::make_shared<PointCloud<PointT>>();
        cloud_downsampled->reserve(grid_cells.size());
        for (const auto & grid_cell : grid_cells) {
            cloud_downsampled->points.emplace_back(grid_cell.second);
        }

        cloud_downsampled->header = _cloud->header;
        return cloud_downsampled;
    }

    struct pair_hash_
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &_p) const 
        {
            auto h1 = std::hash<T1>{}(_p.first);
            auto h2 = std::hash<T2>{}(_p.second);
            return h1 ^ h2;
        }
    };

    // Members
    HeightMap map_;
    // Height mapping objects
    FastHeightFilter height_filter_;
    HeightEstimatorBase::Ptr height_estimator_;
    HeightMapRaycaster raycaster_;
};
} // namespace height_mapping

#endif // !__HEIGHT_MAPPER_HPP__
