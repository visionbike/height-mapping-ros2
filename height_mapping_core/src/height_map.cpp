#include "height_mapping_core/height_map/height_map.hpp"

namespace height_mapping 
{

    HeightMap::HeightMap() 
    {
        // Add core layers
        add(layers::Height::HEIGHT);
        add(layers::Height::HEIGHT_MIN);
        add(layers::Height::HEIGHT_MAX);
        add(layers::Height::HEIGHT_VARIANCE);
        add(layers::Height::NUM_MEASUREMENTS, 0.0f);
        setFrameId("map");
        setBasicLayers({
            layers::Height::HEIGHT,
            layers::Height::HEIGHT_MIN,
            layers::Height::HEIGHT_MAX});
    }

    void HeightMap::addLayer(const std::string &_layer, float _default_value) 
    {
        if (!exists(_layer)) { add(_layer, _default_value); }
    }

    void HeightMap::addBasicLayer(const std::string &_layer) 
    {
        auto basic_layers = getBasicLayers();
        basic_layers.insert(basic_layers.end(), {_layer});
        setBasicLayers(basic_layers);
    }

    void HeightMap::removeLayer(const std::string &_layer) 
    {
        if (exists(_layer)) { erase(_layer); }
    }

    bool HeightMap::hasLayer(const std::string &_layer) const 
    { 
        return exists(_layer); 
    }

    grid_map::Matrix &HeightMap::getHeightMatrix()
    {
        return get(layers::Height::HEIGHT);
    }

    grid_map::Matrix &HeightMap::getHeightMinMatrix() 
    { 
        return get(layers::Height::HEIGHT_MIN); 
    }

    grid_map::Matrix &HeightMap::getHeightMaxMatrix() 
    { 
        return get(layers::Height::HEIGHT_MAX); 
    }

    grid_map::Matrix &HeightMap::getHeightVarianceMatrix() 
    { 
        return get(layers::Height::HEIGHT_VARIANCE); 
    }

    grid_map::Matrix &HeightMap::getMeasurementCountMatrix() 
    { 
        return get(layers::Height::NUM_MEASUREMENTS); 
    }

    const grid_map::Matrix &HeightMap::getHeightMatrix() const 
    { 
        return get(layers::Height::HEIGHT); 
    }

    const grid_map::Matrix &HeightMap::getHeightMinMatrix() const 
    { 
        return get(layers::Height::HEIGHT_MIN); 
    }

    const grid_map::Matrix &HeightMap::getHeightMaxMatrix() const 
    { 
        return get(layers::Height::HEIGHT_MAX); 
    }

    const grid_map::Matrix &HeightMap::getHeightVarianceMatrix() const 
    { 
        return get(layers::Height::HEIGHT_VARIANCE); 
    }

    const grid_map::Matrix &HeightMap::getMeasurementCountMatrix() const 
    { 
        return get(layers::Height::NUM_MEASUREMENTS); 
    }

    bool HeightMap::isEmptyAt(const grid_map::Index &_index) const 
    { 
        return !isValid(_index); 
    }

    bool HeightMap::isEmptyAt(const std::string &_layer, const grid_map::Index &_index) const 
    { 
        return !exists(_layer) || !std::isfinite(at(_layer, _index)); 
    }

    float HeightMap::getMinHeight() const 
    { 
        return HeightMapMath::getMinValue(*this, layers::Height::HEIGHT); 
    }

    float HeightMap::getMaxHeight() const 
    { 
        return HeightMapMath::getMaxValue(*this, layers::Height::HEIGHT); 
    }

    bool HeightMap::hasHeightValues() const
    {
        const auto &mat = getHeightMatrix();
        auto allNaN = mat.array().unaryExpr([](float elem) { return std::isnan(elem); }).all();
        return !allNaN;
    }

    std::vector<grid_map::Position3> HeightMap::getNeighborHeights(const grid_map::Index &_index, double _radius) const
    {
        std::vector<grid_map::Position3> neighbors;
        grid_map::Position center_position;
        getPosition(_index, center_position);
        grid_map::Position3 position;
        grid_map::CircleIterator iterator(*this, center_position, _radius * 1.414);
        while (!iterator.isPastEnd()) {
            if (getPosition3(layers::Height::HEIGHT, *iterator, position)) {
                neighbors.push_back(position);
            }
            ++iterator;
        }
        return neighbors;
    }

    float HeightMapMath::getMinValue(const HeightMap &_map, const std::string &_layer) 
    {
        const auto &data = _map[_layer];
        auto fillNaNForFindingMinVal = data.array().isNaN().select(std::numeric_limits<double>::max(), data);
        return fillNaNForFindingMinVal.minCoeff();
    }

    float HeightMapMath::getMaxValue(const HeightMap &_map, const std::string &_layer) 
    {
        const auto &data = _map[_layer];
        // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
        auto fillNaNForFindingMaxVal = data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
        return fillNaNForFindingMaxVal.maxCoeff();
    }

} // namespace height_mapping