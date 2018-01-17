/* Copyright (c) 2017, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: internal_grid_map.hpp
 *   Created on: Mar, 23, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */

#ifndef INTERNAL_GRID_MAP_INTERNAL_GRID_MAP_HPP
#define INTERNAL_GRID_MAP_INTERNAL_GRID_MAP_HPP

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/core/eigen.hpp>
#include "eigen2cv.hpp"
namespace hmpl {
/**
 * This class includes gridmap data and the method to calculate corresponding
 * data.
 * lenth is ordered by x y
 * size is ordered by x y
 * x matches rows
 * y matches column
 */
class InternalGridMap {
 public:
    /**
     * Constructor
     */
    InternalGridMap();
    /**
     * Initialize the grid map with frame ID , side lengths, map resolution,
     * Map position
     * @param frame_id The frame ID of the map
     * @param length The lengths of x y sides
     * @param resolution The resolution of the map
     * @param position The position of the map frame
     */
    bool init(const std::string frame_id, const grid_map::Length& length,
              const double resolution,
              const grid_map::Position& position = grid_map::Position::Zero());

    /**
     * Just initialize the geometry of the map.
     * @param image
     * @param resolution
     * @param position
     * @return
     */
    bool initializeFromImage(const cv::Mat& image, const double resolution,
                             const grid_map::Position& position);

    /**
     *
     * @param image
     * @param alpha_threshold
     * @return
     */
    bool addObstacleLayerFromImage(const cv::Mat& image,
                                   const double alpha_threshold = 0.5);
    /**
     * Update the distance map in eigen routine
     * @return true if succeed
     */
    bool updateDistanceLayer();
   
    /**
     * Update distance map in opencv routine
     * @return
     */
    bool updateDistanceLayerCV();

    /**
     *
     * @param index
     * @return
     */
    float getObstacleDistance(const grid_map::Index& index) const;

    /**
     *
     * @param pos
     * @return
     */
    float getObstacleDistance(const grid_map::Position& pos) const;

    ~InternalGridMap();

    // syntax sugar
    // obstacle map layer ID
    std::string obs;
    // distance map layer ID
    std::string dis;
    //

    /// The grid map container
    grid_map::GridMap maps;

    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;
};

}  // namespace internal_grid_map

#endif  // INTERNAL_GRID_MAP_INTERNAL_GRID_MAP_HPP
