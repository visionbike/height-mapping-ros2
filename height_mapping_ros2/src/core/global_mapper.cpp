#include "height_mapping_ros2/height_mapping/core/global_mapper.hpp"

namespace height_mapping
{

    GlobalMapper::GlobalMapper(const Config &_cfg)
        : HeightMapper(_cfg)
    {
    const auto &map = getHeightMap();
    measured_indices_.reserve(map.getSize().prod());
    }

    const std::unordered_set<grid_map::Index> &GlobalMapper::getMeasuredGridIndices() const
    {
        return measured_indices_;
    }

    //////////////////////////////////////////////////
    // Explicit instantiation of template functions //
    //////////////////////////////////////////////////

    // Laser
    template PointCloudPtr<Laser> GlobalMapper::heightMapping<Laser>(const PointCloudPtr<Laser> &_cloud);
    template void GlobalMapper::recordMeasuredCells_<Laser>(const height_mapping::HeightMap &_map, const pcl::PointCloud<Laser> &_cloud);

    // Color
    template PointCloudPtr<Color> GlobalMapper::heightMapping<Color>(const PointCloudPtr<Color> &_cloud);
    template void GlobalMapper::recordMeasuredCells_<Color>(const height_mapping::HeightMap &_map, const pcl::PointCloud<Color> &_cloud);

}  // namespace height_mapping
