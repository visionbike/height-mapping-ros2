#pragma once

#ifndef __HEIGHT_ESTIMATOR_MOVING_AVERAGE_HPP__
#define __HEIGHT_ESTIMATOR_MOVING_AVERAGE_HPP__

#include "height_mapping_core/height_estimators/height_estimator_base.hpp"

namespace height_mapping 
{

class MovingAverageEstimator : public HeightEstimatorBase 
{
public:
    struct Parameters 
    {
        float alpha{0.8f};              // Moving average weight [0, 1]: closer to 1 means more weight to new measurement
        bool weight_adaptive{false};    // Enable adaptive weight based on height difference
    };

    explicit MovingAverageEstimator() = default;
    ~MovingAverageEstimator() override = default;

    void setMovingAverageWeight(float _alpha);

    void estimate(HeightMap &_map, const PointCloud<Point> &_cloud) override;
    void estimate(HeightMap &_map, const PointCloud<Laser> &_cloud) override;
    void estimate(HeightMap &_map, const PointCloud<Color> &_cloud) override;

    static void movingAveageUpdate(float &_current, float _new_value, float _alpha);

private:
    // Minimum weight for stability
    // Maximum weight to ensure adaptability
    static float clamp_(float _value, float _min, float _max);

    /**
     * Update mean with exponential moving average
     * @param current Current value
     * @param new_value New measurement
     * @param alpha Weight factor [0,1], higher means more weight to new
     * measurement
     */

    /**
     * Calculate adaptive weight based on measurement difference
     * Reduces weight for measurements that differ significantly
     */
    float calculateAdaptiveWeight_(float _current, float _new_value) const;

    Parameters params_;
};
} // namespace height_mapping

#endif // !__HEIGHT_ESTIMATOR_MOVING_AVERAGE_HPP__