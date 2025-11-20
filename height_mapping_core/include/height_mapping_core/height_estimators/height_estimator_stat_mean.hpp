#pragma once

#ifndef __HEIGHT_ESTIMATOR_STAT_MEAN_HPP__
#define __HEIGHT_ESTIMATOR_STAT_MEAN_HPP__

#include "height_mapping_core/height_estimators/height_estimator_base.hpp"

namespace height_mapping 
{
class StatMeanEstimator : public HeightEstimatorBase 
{
public:
    StatMeanEstimator() = default;
    ~StatMeanEstimator() override = default;

    void estimate(HeightMap &_map, const PointCloud<Point> &_cloud) override;
    void estimate(HeightMap &_map, const PointCloud<Laser> &_cloud) override;
    void estimate(HeightMap &_map, const PointCloud<Color> &_cloud) override;

private:
    /**
     * Updates running statistics (mean, variance) using Welford's online
     * algorithm Reference:
     * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
     */
    void updateHeightStats_(float &_height, float &_variance, float _n, float _new_height);

    /**
     * Updates running mean using numerically stable method
     * Handles potential numerical precision issues with large n
     */
    void meanFilter_(float &_mean, float _n, float _new_value);
    
    /**
     * Calculate Standard Error of the Mean (SEM)
     * SEM = σ/√n, where σ is standard deviation and n is sample size
     * Lower SEM indicates more precise estimate of the true mean
     */
    float getStandardError_(float _n, float _variance) const;

    /**
     * Calculate confidence interval using t-distribution
     * For 95% confidence level with degrees of freedom = n-1
     * Returns the half-width of the confidence interval
     */
    float getConfidenceInterval_(float _n, float _variance, float _confidence_level = 0.95f) const;

    /**
     * Calculate relative standard error (RSE) as a percentage
     * RSE = (SEM/mean) * 100
     * Useful for comparing variability between measurements
     */
    float getRelativeStandardError_(float _mean, float _n, float _variance) const;

};
} // namespace height_mapping

#endif // !__HEIGHT_ESTIMATOR_STAT_MEAN_HPP__