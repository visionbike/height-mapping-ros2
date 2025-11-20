#pragma once

#ifndef __HEIGHT_ESTIMATOR_KALMAN_HPP__
#define __HEIGHT_ESTIMATOR_KALMAN_HPP__

#include "height_mapping_core/height_estimators/height_estimator_base.hpp"

namespace height_mapping 
{

class KalmanEstimator : public HeightEstimatorBase 
{
public:
    struct Parameters 
    {
        float base_uncertainty{0.03f}; // Base measurement uncertainty
        float distance_factor{0.001f}; // How much uncertainty increases with distance
        float angle_factor{0.001f};    // How much uncertainty increases with angle
        float variance_min{0.001f};    // Minimum allowed variance
        float variance_max{1.0f};      // Maximum allowed variance
        bool variance_adaptive{true};  // Enable distance/angle-based variance
    };

    explicit KalmanEstimator() = default;
    ~KalmanEstimator() override = default;

    void setParameters(const Parameters &_params);

    void estimate(HeightMap &_map, const PointCloud<Point> &_cloud) override;
    void estimate(HeightMap &_map, const PointCloud<Laser> &_cloud) override;
    void estimate(HeightMap &_map, const PointCloud<Color> &_cloud) override;

    /**
     * Kalman filter update step
     * @param _height Current height estimate
     * @param _variance Current height variance
     * @param _point_height New height measurement
     * @param _point_variance Measurement uncertainty
     */
    static void kalmanUpdate(float &_height, float &_variance, float _point_height, float _point_variance);

private:
    /**
     * Calculate measurement variance based on point properties
     * Considers distance and angle from sensor
     */
    float getPointVariance_(const Eigen::Vector3f &_point) const; 

    /**
     * Get normalized confidence score [0,1]
     * Higher values indicate more reliable estimates
     */
    float getConfidence_(float _variance) const;
    
    static float clamp_(float _value, float _min, float _max);

    Parameters params_;
};

} // namespace height_mapping

#endif // !__HEIGHT_ESTIMATOR_KALMAN_HPP__