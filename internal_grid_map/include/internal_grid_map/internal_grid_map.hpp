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
#include "internal_grid_map/eigen2cv.hpp"
#include "internal_grid_map/exploration_transform_vis.h"

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

    bool updateInflatedLayer(int inflation_radius_map_cells);

    bool updateDeflatedLayer(int deflation_radius_map_cells);
    /**
     * add a exploration_transform layer to grid map, calculate and update
     * explore transform of free cells from goal points using flood-fill algorithm
     * @param goal_points
     * @param lethal_dist min minimal safety distance  !use cell unit
     * @param penalty_dist threshold value used to represent uncomfortableness  !use cell unit
     * @return
     */
    bool updateExplorationTransform(const std::vector<grid_map::Index>& goal_points,
                                    const float lethal_dist,
                                    const float penalty_dist,
                                    float alpha = 1.0,
                                    bool use_cell_danger = true);
    /**
     * calculate explore transform value of free neighberhood cells of current cell,
     * then push the vaild cells into flood-fill queue.
     * dangerous_cost = penalty_dist - dist2obstacle
     * explore transform value = curr_cost + travel_cost + dangerous_cost^2
     * @param idx_x
     * @param idx_y
     * @param curr_val
     * @param add_cost interval travel path cost  !use cell unit
     * @param lethal_dist minimal distance to obstacle for updating explore  !use cell unit
     * @param penalty_dist  !use cell unit
     * @param point_queue
     * @param p_alpha factor of discomfort because of obstacles
     */
    void touchExplorationCell(const int idx_x, const int idx_y, const float curr_val,
                              const float add_cost, const float lethal_dist, const float penalty_dist,
                              std::queue<grid_map::Index> &point_queue,float p_alpha = 1.0);

    /**
      *
      * @param index
      * @return
      */
    float getExplorationTFValue(const grid_map::Index& index) const;

    /**
     *
     * @param pos
     * @return
     */
    float getExplorationTFValue(const grid_map::Position& pos) const;
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
    // inflated map layer ID
    std::string inflate;
    // deflated map layer ID
    std::string deflate;
    // explore transform map layer ID
    std::string explore_transform;

    // independent publisher class for showing explore transform gradient map
    boost::shared_ptr<hmpl::ExplorationTransformVis> vis_;

    /// The grid map container
    grid_map::GridMap maps;

    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;

};

}  // namespace internal_grid_map

#endif  // INTERNAL_GRID_MAP_INTERNAL_GRID_MAP_HPP
