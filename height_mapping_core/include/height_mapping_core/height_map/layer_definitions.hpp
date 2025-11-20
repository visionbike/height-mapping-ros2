#pragma once

#ifndef __LAYER_DEFINITIONS_HPP__
#define __LAYER_DEFINITIONS_HPP__

namespace height_mapping 
{

namespace layers 
{
    struct Height 
    {
        static constexpr const char *HEIGHT           = "height";
        static constexpr const char *HEIGHT_MIN       = "height_min";
        static constexpr const char *HEIGHT_MAX       = "height_max";
        static constexpr const char *HEIGHT_VARIANCE  = "height_variance";
        static constexpr const char *NUM_MEASUREMENTS = "num_measured";
    };

    struct Scan 
    {
        static constexpr const char *RAY_CASTING = "scan/raycasting";
        static constexpr const char *SCAN_HEIGHT = "scan/scan_height";
    };

    struct Confidence 
    {
        static constexpr const char *STANDARD_ERROR      = "height/standard_error";
        static constexpr const char *CONFIDENCE_INTERVAL = "height/confidence_interval";
        static constexpr const char *CONFIDENCE          = "height/confidence";
    };

    struct Sensor 
    {
        struct Lidar 
        {
            static constexpr const char *INTENSITY = "lidar/intensity";
        };

        struct RGBD 
        {
            static constexpr const char *RED   = "rgbd/red";
            static constexpr const char *GREEN = "rgbd/green";
            static constexpr const char *BLUE  = "rgbd/blue";
            static constexpr const char *COLOR = "rgbd/color";
        };
    };

} // namespace layers

} // namespace height_mapping

#endif // !__LAYER_DEFINITIONS_HPP__