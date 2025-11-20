/*
 * KalmanEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/height_estimators/height_estimator_kalman.hpp"

namespace height_mapping 
{
    void KalmanEstimator::setParameters(const Parameters &_params) 
    { 
        params_ = _params; 
    }

    void KalmanEstimator::estimate(HeightMap &_map, const PointCloud<Point> &_cloud) 
    {

        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
            return;
        }

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_min      = _map.getHeightMinMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();
        auto &matrix_num_measured    = _map.getMeasurementCountMatrix();

        _map.addLayer(layers::Confidence::CONFIDENCE, 0.0f);
        auto &matrix_confidence = _map[layers::Confidence::CONFIDENCE];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            auto &height     = matrix_height(index_measured(0), index_measured(1));
            auto &variance   = matrix_height_variance(index_measured(0), index_measured(1));
            auto &height_min = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max = matrix_height_max(index_measured(0), index_measured(1));
            auto &num_points = matrix_num_measured(index_measured(0), index_measured(1));
            auto &confidence = matrix_confidence(index_measured(0), index_measured(1));

            const Eigen::Vector3f point_vec(new_point.x, new_point.y, new_point.z);
            const float point_variance = getPointVariance_(point_vec);

            if (_map.isEmptyAt(index_measured)) {
                height     = new_point.z;
                height_min = new_point.z;
                height_max = new_point.z;
                variance   = point_variance;
                num_points = 1;
                confidence = getConfidence_(variance);
                continue;
            }

            ++num_points;
            kalmanUpdate(height, variance, new_point.z, point_variance);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);
            confidence = getConfidence_(variance);
        }
    }

    void KalmanEstimator::estimate(HeightMap &_map, const PointCloud<Laser> &_cloud) 
    {

        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
            return;
        }

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_min      = _map.getHeightMinMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();

        _map.addLayer(layers::Confidence::CONFIDENCE, 0.0f);
        auto &matrix_confidence = _map[layers::Confidence::CONFIDENCE];

        _map.addLayer(layers::Sensor::Lidar::INTENSITY);
        auto &matrix_intensity = _map[layers::Sensor::Lidar::INTENSITY];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) {
                continue;
            }

            auto &height     = matrix_height(index_measured(0), index_measured(1));
            auto &variance   = matrix_height_variance(index_measured(0), index_measured(1));
            auto &height_min = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max = matrix_height_max(index_measured(0), index_measured(1));
            auto &confidence = matrix_confidence(index_measured(0), index_measured(1));
            auto &intensity  = matrix_intensity(index_measured(0), index_measured(1));

            const Eigen::Vector3f point_vec(new_point.x, new_point.y, new_point.z);
            const float point_variance = getPointVariance_(point_vec);

            if (_map.isEmptyAt(index_measured)) {
                height     = new_point.z;
                variance   = point_variance;
                height_min = height_max = new_point.z;
                intensity  = new_point.intensity;
                confidence = getConfidence_(variance);
                continue;
            }

            kalmanUpdate(height, variance, new_point.z, point_variance);
            kalmanUpdate(intensity, variance, new_point.intensity, point_variance);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);
            confidence = getConfidence_(variance);
        }
    }

    void KalmanEstimator::estimate(HeightMap &_map, const PointCloud<Color> &_cloud) 
    {

        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \n";
            return;
        }

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();
        auto &matrix_height_min      = _map.getHeightMinMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();

        _map.addLayer(layers::Confidence::CONFIDENCE, 0.0f);
        _map.addLayer(layers::Sensor::RGBD::RED);
        _map.addLayer(layers::Sensor::RGBD::GREEN);
        _map.addLayer(layers::Sensor::RGBD::BLUE);
        _map.addLayer(layers::Sensor::RGBD::COLOR);

        auto &matrix_confidence = _map[layers::Confidence::CONFIDENCE];
        auto &matrix_red        = _map[layers::Sensor::RGBD::RED];
        auto &matrix_green      = _map[layers::Sensor::RGBD::GREEN];
        auto &matrix_blue       = _map[layers::Sensor::RGBD::BLUE];
        auto &matrix_color      = _map[layers::Sensor::RGBD::COLOR];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) {
                continue;
            }

            auto &height     = matrix_height(index_measured(0), index_measured(1));
            auto &variance   = matrix_height_variance(index_measured(0), index_measured(1));
            auto &height_min = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max = matrix_height_max(index_measured(0), index_measured(1));
            auto &confidence = matrix_confidence(index_measured(0), index_measured(1));
            auto &red        = matrix_red(index_measured(0), index_measured(1));
            auto &green      = matrix_green(index_measured(0), index_measured(1));
            auto &blue       = matrix_blue(index_measured(0), index_measured(1));
            auto &color      = matrix_color(index_measured(0), index_measured(1));

            const Eigen::Vector3f point_vec(new_point.x, new_point.y, new_point.z);
            const float point_variance = getPointVariance_(point_vec);

            if (_map.isEmptyAt(index_measured)) {
                height     = new_point.z;
                variance   = point_variance;
                height_min = height_max = new_point.z;
                red        = new_point.r;
                green      = new_point.g;
                blue       = new_point.b;
                grid_map::colorVectorToValue(new_point.getRGBVector3i(), color);
                confidence = getConfidence_(variance);
                continue;
            }

            kalmanUpdate(height, variance, new_point.z, point_variance);
            kalmanUpdate(red, variance, new_point.r, point_variance);
            kalmanUpdate(green, variance, new_point.g, point_variance);
            kalmanUpdate(blue, variance, new_point.b, point_variance);
            grid_map::colorVectorToValue(Eigen::Vector3i(red, green, blue), color);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);
            confidence = getConfidence_(variance);
        }
    }

    void KalmanEstimator::kalmanUpdate(float &_height, float &_variance, float _point_height, float _point_variance)
    {
        const float kalman_gain = _variance / (_variance + _point_variance);
        _height = _height + kalman_gain * (_point_height - _height);
        _variance = (1.0f - kalman_gain) * _variance;
    }

    float KalmanEstimator::getPointVariance_(const Eigen::Vector3f &_point) const 
    {
        if (!params_.variance_adaptive) {
            return params_.base_uncertainty;
        }

        // Distance-based uncertainty
        const float distance = _point.norm();
        float variance = params_.base_uncertainty + params_.distance_factor * distance * distance;

        // Angle-based uncertainty (if point not directly below sensor)
        if (distance > 1e-3f) {
            const float angle = std::acos(std::abs(_point.z()) / distance);
            variance += params_.angle_factor * angle * angle;
        }

        return clamp_(variance, params_.variance_min, params_.variance_max);
    }

    float KalmanEstimator::getConfidence_(float _variance) const 
    {
        return 1.0f - clamp_(_variance / params_.variance_max, 0.0f, 1.0f);
    }

    float KalmanEstimator::clamp_(float _value, float _min, float _max) 
    {
        return std::min(std::max(_value, _min), _max);
    }
} // namespace height_mapping