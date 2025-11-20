#pragma once

#ifndef __HEIGHT_MAP_RAYCASTER_HPP__
#define __HEIGHT_MAP_RAYCASTER_HPP__

#include "height_mapping_core/height_map/cloud_types.hpp"
#include "height_mapping_core/height_map/height_map.hpp"

namespace height_mapping 
{

class HeightMapRaycaster 
{
public:
    HeightMapRaycaster() = default;

    template <typename PointT>
    void correctHeight(HeightMap &_map, const PointCloud<PointT> &_cloud, const Eigen::Vector3f &_sensor_origin) 
    {

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();
        auto &matrix_num_measured    = _map.getMeasurementCountMatrix();

        _map.addLayer(layers::Scan::RAY_CASTING);
        _map.clear(layers::Scan::RAY_CASTING);
        auto &matrix_ray_casting = _map.get(layers::Scan::RAY_CASTING);

        _map.addLayer(layers::Scan::SCAN_HEIGHT);
        _map.clear(layers::Scan::SCAN_HEIGHT);
        auto &matrix_scan_height = _map.get(layers::Scan::SCAN_HEIGHT);

        const float sensor_height = _sensor_origin.z();
        grid_map::Index index_measured;
        grid_map::Position position_measured;

        // Record current scan heights to the map
        for (const auto &point : _cloud.points) {
            position_measured << point.x, point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            matrix_scan_height(index_measured(0), index_measured(1)) = point.z;
        }

        // Raycasting loop
        for (const auto &point : _cloud.points) {
            // Create ray from two points: sensor to measured point
            Eigen::Vector3f ray_dir(
                point.x - _sensor_origin.x(), 
                point.y - _sensor_origin.y(),
                point.z - _sensor_origin.z());
            float ray_length = ray_dir.norm();
            ray_dir.normalize();

            // Visibility check through ray
            float sampling_step = _map.getResolution();
            for (float t = 0; t < ray_length - sampling_step; t += sampling_step) {
                // Get ray point: starting from sensor
                Eigen::Vector3f point_on_ray = _sensor_origin + ray_dir * t;

                // Get ray point index
                grid_map::Position position_check(point_on_ray.x(), point_on_ray.y());
                grid_map::Index index_check;

                // Skip if the ray point is out of the map
                if (!_map.getIndex(position_check, index_check)) { continue; }

                // Do not erase the static obstacles
                auto &scan_height = matrix_scan_height(index_check(0), index_check(1));
                if (std::isfinite(scan_height) && scan_height > point_on_ray.z() + 0.1) { break; }

                // Get map height and variance at the ray point
                auto &map_height          = matrix_height(index_check(0), index_check(1));
                auto &map_height_max      = matrix_height_max(index_check(0), index_check(1));
                auto &map_height_variance = matrix_height_variance(index_check(0), index_check(1));
                auto &ray_height          = matrix_ray_casting(index_check(0), index_check(1));
                auto &num_points          = matrix_num_measured(index_check(0), index_check(1));

                // For visualization of traced ray
                if (!std::isfinite(ray_height)) {
                    ray_height = point_on_ray.z();
                }
                else if (ray_height > point_on_ray.z()) {
                    ray_height = point_on_ray.z();
                }

                // Update height if current height is higher than the ray point
                if (map_height > point_on_ray.z() + threshold_correction_) {
                    map_height_variance += (map_height - point_on_ray.z()); // Increase variance
                    num_points = 1;                                       // Reset num_points
                    map_height = point_on_ray.z() + threshold_correction_; // Height correction
                    // map_height_max =
                    //     point_on_ray.z() + threshold_correction_; // Update max height
                }
            }
        }
    }

private:
    float threshold_correction_{0.02f};
};

} // namespace height_mapping

#endif // !__HEIGHT_MAP_RAYCASTER_HPP__
