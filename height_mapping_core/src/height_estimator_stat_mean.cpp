#include "height_mapping_core/height_estimators/height_estimator_stat_mean.hpp"

/*
StatMeanEstimator tracks:
- elevation
- elevation_min
- elevation_max
- elevation_variance
- n_measured
- intensity (if available)
- color (if available)
*/
namespace height_mapping 
{

    void StatMeanEstimator::estimate(HeightMap &_map, const PointCloud<Point> &_cloud) 
    {
        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "\033[0;31m[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \033[0m\n";
            return;
        }

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_min      = _map.getHeightMinMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();
        auto &matrix_num_measured    = _map.getMeasurementCountMatrix();

        _map.addLayer(layers::Confidence::STANDARD_ERROR);
        _map.addLayer(layers::Confidence::CONFIDENCE_INTERVAL);

        auto &matrix_standard_error      = _map[layers::Confidence::STANDARD_ERROR];
        auto &matrix_confidence_interval = _map[layers::Confidence::CONFIDENCE_INTERVAL];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            // Skip if the point is out of the map
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            auto &height              = matrix_height(index_measured(0), index_measured(1));
            auto &height_min          = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max          = matrix_height_max(index_measured(0), index_measured(1));
            auto &variance            = matrix_height_variance(index_measured(0), index_measured(1));
            auto &num_points          = matrix_num_measured(index_measured(0), index_measured(1));
            auto &standard_error      = matrix_standard_error(index_measured(0), index_measured(1));
            auto &confidence_interval = matrix_confidence_interval(index_measured(0), index_measured(1));

            // Initialize the height and variance if empty (NaN values)
            if (_map.isEmptyAt(index_measured)) {
                height              = new_point.z;
                height_min          = new_point.z;
                height_max          = new_point.z;
                variance            = 0.0f; // Single measurement -> no variance
                num_points          = 1;
                standard_error      = 2.0f; // Assume max error
                confidence_interval = 2.0f;
                continue;
            }

            ++num_points;

            // Height estimates
            updateHeightStats_(height, variance, num_points, new_point.z);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);

            // Statistics
            standard_error      = getStandardError_(num_points, variance);
            confidence_interval = getConfidenceInterval_(num_points, variance);
        }
    }

    void StatMeanEstimator::estimate(HeightMap &_map, const PointCloud<Laser> &_cloud) 
    {
        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "\033[0;31m[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \033[0m\n";
            return;
        }

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_min      = _map.getHeightMinMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();
        auto &matrix_num_measured    = _map.getMeasurementCountMatrix();

        _map.addLayer(layers::Sensor::Lidar::INTENSITY);
        auto &intensityMatrix = _map[layers::Sensor::Lidar::INTENSITY];
        
        _map.addLayer(layers::Confidence::STANDARD_ERROR);
        _map.addLayer(layers::Confidence::CONFIDENCE_INTERVAL);

        auto &matrix_standard_error      = _map[layers::Confidence::STANDARD_ERROR];
        auto &matrix_confidence_interval = _map[layers::Confidence::CONFIDENCE_INTERVAL];

        grid_map::Index index_measured;
        grid_map::Position position_measured;

        for (const auto &new_point : _cloud) {
            // Skip if the point is out of the map
            position_measured << new_point.x, new_point.y;
            if (!_map.getIndex(position_measured, index_measured)) { continue; }

            auto &height              = matrix_height(index_measured(0), index_measured(1));
            auto &height_min          = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max          = matrix_height_max(index_measured(0), index_measured(1));
            auto &variance            = matrix_height_variance(index_measured(0), index_measured(1));
            auto &num_points          = matrix_num_measured(index_measured(0), index_measured(1));
            auto &intensity           = intensityMatrix(index_measured(0), index_measured(1));
            auto &standard_error      = matrix_standard_error(index_measured(0), index_measured(1));
            auto &confidence_interval = matrix_confidence_interval(index_measured(0), index_measured(1));

            // Initialize the height and variance if empty (NaN values)
            if (_map.isEmptyAt(index_measured)) {
                height              = new_point.z;
                height_min          = new_point.z;
                height_max          = new_point.z;
                variance            = 0.0f; // Single measurement -> no variance
                num_points          = 1;
                intensity           = new_point.intensity;
                standard_error      = 2.0f; // Assume max error
                confidence_interval = 2.0f;
                continue;
            }

            ++num_points;

            // Height estimates
            updateHeightStats_(height, variance, num_points, new_point.z);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);

            // Statistics
            standard_error      = getStandardError_(num_points, variance);
            confidence_interval = getConfidenceInterval_(num_points, variance);

            // Intensity
            meanFilter_(intensity, num_points, new_point.intensity);
        }
    }

    void StatMeanEstimator::estimate(HeightMap &_map, const PointCloud<Color> &_cloud) 
    {
        if (hasEmptyCloud(_cloud)) { return; }

        if (_cloud.header.frame_id != _map.getFrameId()) {
            std::cout << "\033[0;31m[HeightEstimator]: Frame ID mismatch - pointcloud is in a different frame! \033[0m\n";
            return;
        }

        auto &matrix_height          = _map.getHeightMatrix();
        auto &matrix_height_min      = _map.getHeightMinMatrix();
        auto &matrix_height_max      = _map.getHeightMaxMatrix();
        auto &matrix_height_variance = _map.getHeightVarianceMatrix();
        auto &matrix_num_measured    = _map.getMeasurementCountMatrix();

        _map.addLayer(layers::Confidence::STANDARD_ERROR);
        _map.addLayer(layers::Confidence::CONFIDENCE_INTERVAL);

        auto &matrix_standard_error      = _map[layers::Confidence::STANDARD_ERROR];
        auto &matrix_confidence_interval = _map[layers::Confidence::CONFIDENCE_INTERVAL];

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

            auto &height              = matrix_height(index_measured(0), index_measured(1));
            auto &height_min          = matrix_height_min(index_measured(0), index_measured(1));
            auto &height_max          = matrix_height_max(index_measured(0), index_measured(1));
            auto &variance            = matrix_height_variance(index_measured(0), index_measured(1));
            auto &num_points          = matrix_num_measured(index_measured(0), index_measured(1));
            auto &standard_error      = matrix_standard_error(index_measured(0), index_measured(1));
            auto &confidence_interval = matrix_confidence_interval(index_measured(0), index_measured(1));

            auto &red   = matrix_red(index_measured(0), index_measured(1));
            auto &green = matrix_green(index_measured(0), index_measured(1));
            auto &blue  = matrix_blue(index_measured(0), index_measured(1));
            auto &color = matrix_color(index_measured(0), index_measured(1));

            // Initialize the height and variance if empty (NaN values)
            if (_map.isEmptyAt(index_measured)) {
                height              = height_min = height_max = new_point.z;
                variance            = 0.0f; // Single measurement -> no variance
                num_points          = 1;
                red                 = new_point.r;
                green               = new_point.g;
                blue                = new_point.b;
                standard_error      = 2.0f; // Assume max error
                confidence_interval = 2.0f;
                grid_map::colorVectorToValue(new_point.getRGBVector3i(), color);
                continue;
            }

            // Update the color if it is NaN
            if (!std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue)) {
                red   = new_point.r;
                green = new_point.g;
                blue  = new_point.b;
            }

            ++num_points;

            // Height estimates
            updateHeightStats_(height, variance, num_points, new_point.z);
            height_min = std::min(height_min, new_point.z);
            height_max = std::max(height_max, new_point.z);

            // Statistics
            standard_error      = getStandardError_(num_points, variance);
            confidence_interval = getConfidenceInterval_(num_points, variance);

            // Color update
            meanFilter_(red, num_points, new_point.r);
            meanFilter_(green, num_points, new_point.g);
            meanFilter_(blue, num_points, new_point.b);
            grid_map::colorVectorToValue(Eigen::Vector3i(red, green, blue), color);
        }
    }

    void StatMeanEstimator::updateHeightStats_(float &_height, float &_variance, float _n, float _new_height)
    {
        const float delta   = (_new_height - _height);
        const float delta_n = delta / _n;

        // Update mean
        _height += delta_n;

        // Update variance using the more stable algorithm
        // M2 = M2 + delta * (new_value - updated_mean)
        // where M2 is the sum of squared differences from the mean
        _variance += delta * (_new_height - _height);

        // Convert M2 to variance
        if (_n > 1) {
            _variance = _variance / (_n - 1); // Use (n-1) for sample variance
        }
    }

    void StatMeanEstimator::meanFilter_(float &_mean, float _n, float _new_value)
    {
        _mean += (_new_value - _mean) / _n;
    }

    float StatMeanEstimator::getStandardError_(float _n, float _variance) const
    {
        if (_n < 2) {
            return std::numeric_limits<float>::infinity();
        }
        return std::sqrt(_variance / _n);
    }

    float StatMeanEstimator::getConfidenceInterval_(float _n, float _variance, float _confidence_level) const
    {
        if (_n < 2) {
            return std::numeric_limits<float>::infinity();
        }

        // t-value for 95% confidence level (could be made configurable)
        // This is simplified; ideally would use actual t-distribution quantiles
        // const float t_value = 1.96f; // Approximation using normal distribution

        const int df = static_cast<int>(_n) - 1;
        if (df <= 0) {
            return std::numeric_limits<float>::infinity();
        }

        // Supported confidence levels
        // 0.90 → 90%
        // 0.95 → 95%
        // 0.99 → 99%
        float cl = _confidence_level;
        if (cl != 0.90f && cl != 0.95f && cl != 0.99f) { cl = 0.95f; } // default

        // Simplified t-table for df up to 30 (common in mapping cells)
        // Values taken from standard Student-t tables
        static const std::unordered_map<int, std::array<float, 3>> t_table = {
            //   90%     95%     99%
            { 1, { 6.314f, 12.706f, 63.657f } },
            { 2, { 2.920f, 4.303f, 9.925f } },
            { 3, { 2.353f, 3.182f, 5.841f } },
            { 4, { 2.132f, 2.776f, 4.604f } },
            { 5, { 2.015f, 2.571f, 4.032f } },
            { 6, { 1.943f, 2.447f, 3.707f } },
            { 7, { 1.895f, 2.365f, 3.499f } },
            { 8, { 1.860f, 2.306f, 3.355f } },
            { 9, { 1.833f, 2.262f, 3.250f } },
            { 10, { 1.812f, 2.228f, 3.169f } },
            { 15, { 1.753f, 2.131f, 2.947f } },
            { 20, { 1.725f, 2.086f, 2.845f } },
            { 30, { 1.697f, 2.042f, 2.750f } }
        };

        float t_value = 1.96f; // fallback

        // Pick closest df table entry
        int best_df = df;
        if (!t_table.count(df)) {
            // find nearest df in table
            int min_diff = 999;
            for (auto &kv : t_table) {
                int diff = std::abs(kv.first - df);
                if (diff < min_diff) {
                    min_diff = diff;
                    best_df = kv.first;
                }
            }
        }

        const auto &row = t_table.at(best_df);

        if (cl == 0.90f) { t_value = row[0]; }
        else if (cl == 0.95f) { t_value = row[1]; }
        else if (cl == 0.99f) { t_value = row[2]; }

        return t_value * getStandardError_(_n, _variance);
    }

    float StatMeanEstimator::getRelativeStandardError_(float _mean, float _n, float _variance) const
    {
        if (std::abs(_mean) < 1e-6f || _n < 2) {
            return std::numeric_limits<float>::infinity();
        }
        return (getStandardError_(_n, _variance) / std::abs(_mean)) * 100.0f;
    }

} // namespace height_mapping