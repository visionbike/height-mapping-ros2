#include <unordered_map>
#include <iostream>
#include <cmath>

#include "height_mapping_ros2/height_mapping/core/height_mapper.hpp"

namespace height_mapping
{

    HeightMapper::HeightMapper(const Config &_cfg)
        : cfg{_cfg}, height_filter_{_cfg.height_min, _cfg.height_max}
    {
        initMap_();
        initHeightEstimator_();
    }

    void HeightMapper::moveMapOrigin(const grid_map::Position &_position)
    {
        map_.move(_position);
    }

    const HeightMap &HeightMapper::getHeightMap() const 
    { 
        return map_; 
    }

    HeightMap &HeightMapper::getHeightMap() 
    { 
        return map_; 
    }

    void HeightMapper::setMapPosition(const grid_map::Position &_position) 
    { 
        map_.setPosition(_position); 
    }

    void HeightMapper::clearMap() 
    { 
        map_.clearAll(); 
    }

    void HeightMapper::initMap_()
    {
        // Check parameter validity
        if (cfg.grid_resolution <= 0.0) {
            throw std::invalid_argument("[height_mapping::HeightMapper]: Grid resolution must be positive");
        }
        if (cfg.map_length_x <= 0.0 || cfg.map_length_y <= 0.0) {
            throw std::invalid_argument("[height_mapping::HeightMapper]: Map dimensions must be positive");
        }

        // Initialize map geometry
        map_.setFrameId(cfg.frame_id);
        map_.setGeometry(grid_map::Length(cfg.map_length_x, cfg.map_length_y), cfg.grid_resolution);
    }

    void HeightMapper::initHeightEstimator_()
    {
        // Set height estimator
        // - Kalman Filter
        // - Moving Average
        // - StatMean (by default)

        if (cfg.estimator_type == "KalmanFilter") {
            height_estimator_ = std::make_unique<height_mapping::KalmanEstimator>();
            std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator type --> KalmanFilter \033[0m\n";

        } else if (cfg.estimator_type == "MovingAverage") {
            height_estimator_ = std::make_unique<height_mapping::MovingAverageEstimator>();
            std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator type --> MovingAverage \033[0m\n";

        } else if (cfg.estimator_type == "StatMean") {
            height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
            std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator type --> StatisticalMeanEstimator \033[0m\n";

        } else {
            height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
            std::cout <<"\033[1;33m[height_mapping::HeightMapper] Invalid height estimator type. Set Default: StatMeanEstimator \033[0m\n";
        }
    }

    /////////////////////////////////////////////////////////////////////////////
    ///////////////// EXPLICIT INSTANTIATION OF TEMPLATE FUNCTIONS //////////////
    /////////////////////////////////////////////////////////////////////////////

    // These assume you have typedefs like:
    // using Laser = height_mapping_types::ElevationPoint; or your laser point type
    // using Color = pcl::PointXYZRGB; (for example)

    // Laser
    template PointCloudPtr<Laser> HeightMapper::heightMapping<Laser>(const PointCloudPtr<Laser> &_cloud);

    template void HeightMapper::raycasting<Laser>(const Eigen::Vector3f &_sensor_origin, const PointCloudPtr<Laser> &_cloud);
    
    template void HeightMapper::fastHeightFilter<Laser>(const PointCloudPtr<Laser> &_cloud, PointCloudPtr<Laser> &_cloud_filterd);

    template PointCloudPtr<Laser> HeightMapper::cloudRasterization_<Laser>(const PointCloudPtr<Laser> &_cloud, float _grid_size);

    template PointCloudPtr<Laser> HeightMapper::cloudRasterizationAlt_<Laser>(const PointCloudPtr<Laser> &_cloud, float _grid_size);

    // Color
    template PointCloudPtr<Color> HeightMapper::heightMapping<Color>(const PointCloudPtr<Color> &_cloud);

    template void HeightMapper::raycasting<Color>(const Eigen::Vector3f &_sensor_origin, const PointCloudPtr<Color> &_cloud);

    template void HeightMapper::fastHeightFilter<Color>(const PointCloudPtr<Color> &_cloud, PointCloudPtr<Color> &_cloud_filterd);

    template PointCloudPtr<Color> HeightMapper::cloudRasterization_<Color>(const PointCloudPtr<Color> &_cloud, float _grid_size);

    template PointCloudPtr<Color> HeightMapper::cloudRasterizationAlt_<Color>(const PointCloudPtr<Color> &_cloud, float _grid_size);

}  // namespace height_mapping
