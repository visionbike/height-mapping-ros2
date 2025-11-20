#include "height_mapping_core/height_estimators/height_estimator_moving_average.hpp"

namespace height_mapping 
{
    void MovingAverageEstimator::setMovingAverageWeight(float _alpha) 
    { 
        params_.alpha = _alpha; 
    }

    void MovingAverageEstimator::estimate(HeightMap &_map, const PointCloud<Point> &_cloud) 
    {
        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
            return;
        }

        // Prepare matrices
        auto &matrix_height       = _map.getHeightMatrix();
        auto &matrix_height_min   = _map.getHeightMinMatrix();
        auto &matrix_height_max   = _map.getHeightMaxMatrix();
        auto &matrix_num_measured = _map.getMeasurementCountMatrix();

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            // Skip if the point is out of the map
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            auto &height     = matrix_height(index_measured(0), index_measured(1));
            auto &height_min = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max = matrix_height_max(index_measured(0), index_measured(1));
            auto &num_points = matrix_num_measured(index_measured(0), index_measured(1));

            // Initialize the height and variance if it is NaN
            if (_map.isEmptyAt(index_measured)) {
                height     = new_point.z;
                height_min = new_point.z;
                height_max = new_point.z;
                num_points = 1;
                continue;
            }

            ++num_points;
            movingAveageUpdate(height, new_point.z, params_.alpha);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);
        }
    }

    void MovingAverageEstimator::estimate(HeightMap &_map, const PointCloud<Laser> &_cloud)
    {
        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
            return;
        }

        auto &matrix_height     = _map.getHeightMatrix();
        auto &matrix_height_min = _map.getHeightMinMatrix();
        auto &matrix_height_max = _map.getHeightMaxMatrix();

        _map.addLayer(layers::Sensor::Lidar::INTENSITY);
        auto &intensityMatrix = _map[layers::Sensor::Lidar::INTENSITY];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            // Skip if the point is out of the map
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            auto &height     = matrix_height(index_measured(0), index_measured(1));
            auto &intensity  = intensityMatrix(index_measured(0), index_measured(1));
            auto &height_min = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max = matrix_height_max(index_measured(0), index_measured(1));

            // Initialize the height and variance if it is NaN
            if (_map.isEmptyAt(index_measured)) {
                height    = height_min = height_max = new_point.z;
                intensity = new_point.intensity;
                continue;
            }
            movingAveageUpdate(height, new_point.z, params_.alpha);
            movingAveageUpdate(intensity, new_point.intensity, params_.alpha);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);
        }
    }

    void MovingAverageEstimator::estimate(HeightMap &_map, const PointCloud<Color> &_cloud) 
    {
        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
            return;
        }

        auto &matrix_height     = _map.getHeightMatrix();
        auto &matrix_height_min = _map.getHeightMinMatrix();
        auto &matrix_height_max = _map.getHeightMaxMatrix();

        _map.addLayer(layers::Sensor::RGBD::RED);
        _map.addLayer(layers::Sensor::RGBD::GREEN);
        _map.addLayer(layers::Sensor::RGBD::BLUE);
        _map.addLayer(layers::Sensor::RGBD::COLOR);

        auto &matrix_red   = _map[layers::Sensor::RGBD::RED];
        auto &matrix_green = _map[layers::Sensor::RGBD::GREEN];
        auto &matrix_blue  = _map[layers::Sensor::RGBD::BLUE];
        auto &matrix_color = _map[layers::Sensor::RGBD::COLOR];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            // Skip if the point is out of the map
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            auto &height     = matrix_height(index_measured(0), index_measured(1));
            auto &height_min = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max = matrix_height_max(index_measured(0), index_measured(1));
            auto &red        = matrix_red(index_measured(0), index_measured(1));
            auto &green      = matrix_green(index_measured(0), index_measured(1));
            auto &blue       = matrix_blue(index_measured(0), index_measured(1));
            auto &color      = matrix_color(index_measured(0), index_measured(1));

            // Initialize the height and variance if it is NaN
            if (_map.isEmptyAt(index_measured)) {
                height = height_min = height_max = new_point.z;
                red    = new_point.r;
                green  = new_point.g;
                blue   = new_point.b;
                grid_map::colorVectorToValue(new_point.getRGBVector3i(), color);
                continue;
            }

            // Height estimates
            movingAveageUpdate(height, new_point.z, params_.alpha);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);

            // Color estimates
            movingAveageUpdate(red, new_point.r, params_.alpha);
            movingAveageUpdate(green, new_point.g, params_.alpha);
            movingAveageUpdate(blue, new_point.b, params_.alpha);
            grid_map::colorVectorToValue(Eigen::Vector3i(red, green, blue), color);
        }
    }

    void MovingAverageEstimator::movingAveageUpdate(float &_current, float _new_value, float _alpha) 
    {
        _alpha = clamp_(_alpha, 0.1f, 0.9f);
        _current = ((1.0f - _alpha) * _current) + (_alpha * _new_value);
    }

    float MovingAverageEstimator::clamp_(float _value, float _min, float _max)
    {
        return std::min(std::max(_value, _min), _max);
    }

    float MovingAverageEstimator::calculateAdaptiveWeight_(float _current, float _new_value) const
    {
        if (!params_.weight_adaptive) { return params_.alpha; }

        const float diff = std::abs(_new_value - _current);
        const float max_expected_diff = 0.5f; // Could be made configurable

        // Reduce weight for larger differences
        return params_.alpha * std::exp(-diff / max_expected_diff);
    }

} // namespace height_mapping