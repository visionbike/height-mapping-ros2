#pragma once

#ifndef __HEIGHT_MAP_HPP__
#define __HEIGHT_MAP_HPP__

#include <string>
#include <grid_map_core/grid_map_core.hpp>
#include "height_mapping_core/height_map/layer_definitions.hpp"

namespace height_mapping 
{

class HeightMap : public grid_map::GridMap 
{
public:
    HeightMap();

    // Layer management
    void addLayer(const std::string &_layer, float _default_value = NAN);
    void addBasicLayer(const std::string &_layer);
    void removeLayer(const std::string &_layer);
    bool hasLayer(const std::string &_layer) const;

    // Core layer accessors
    grid_map::Matrix &getHeightMatrix();
    grid_map::Matrix &getHeightMinMatrix();
    grid_map::Matrix &getHeightMaxMatrix();
    grid_map::Matrix &getHeightVarianceMatrix();
    grid_map::Matrix &getMeasurementCountMatrix();

    // Core layer accessors: const versions
    const grid_map::Matrix &getHeightMatrix() const;
    const grid_map::Matrix &getHeightMinMatrix() const;
    const grid_map::Matrix &getHeightMaxMatrix() const;
    const grid_map::Matrix &getHeightVarianceMatrix() const;
    const grid_map::Matrix &getMeasurementCountMatrix() const;

    // Cell validity checks
    bool isEmptyAt(const grid_map::Index &_index) const;
    bool isEmptyAt(const std::string &_layer, const grid_map::Index &_index) const;

    // Height statistics
    float getMinHeight() const;

    float getMaxHeight() const;
    
    bool hasHeightValues() const;

    // Basic spatial functions
    std::vector<grid_map::Position3> getNeighborHeights(const grid_map::Index &_index, double _radius) const;
    
private:
    bool is_initialized_{false};
};

class HeightMapMath 
{
public:
  static float getMinValue(const HeightMap &_map, const std::string &_layer);

  static float getMaxValue(const HeightMap &_map, const std::string &_layer);
};

} // namespace height_mapping

#endif // !__HEIGHT_MAP_HPP__