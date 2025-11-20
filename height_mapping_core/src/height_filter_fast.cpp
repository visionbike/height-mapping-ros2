#include "height_mapping_core/height_filters/height_filter_fast.hpp"

namespace height_mapping 
{

    FastHeightFilter::FastHeightFilter(double _min_z, double _max_z)
        : min_z_(_min_z), max_z_(_max_z) {}


    // Explicit instantiation for the types we use
    template void FastHeightFilter::filter<Laser>(const PointCloudPtr<Laser> &_cloud, PointCloudPtr<Laser> &_cloud_filtered);

    template void FastHeightFilter::filter<Color>(const PointCloudPtr<Color> &_cloud, PointCloudPtr<Color> &_cloud_filtered);

} // namespace height_mapping