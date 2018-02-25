//
// Created by kevin on 1/11/18.
//

#ifndef PATH_TRANSFORM_PATH_PLANNING_HPP
#define PATH_TRANSFORM_PATH_PLANNING_HPP
// Grid Map
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <algorithm>
#include <utility>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <opt_utils/opt_utils.hpp>

namespace grid_map_path_planning{
/**
* find dense path using steepest ascend gradient method in
* exploration transform map
* @param grid_map
* @param start_pose
* @param path
* @param occupancy_layer
* @param dist_trans_layer used to decide whether to reserve this sampling cell
* @param expl_trans_layer
* @return
*/
bool findPathOfCompleteCoverage(grid_map::GridMap &grid_map, const hmpl::Pose2D &start_pose,
                                std::vector<geometry_msgs::PoseStamped> &path,
                                const std::string occupancy_layer = "occupancy",
                                const std::string dist_trans_layer = "distance_transform",
                                const std::string expl_trans_layer = "exploration_transform");
/**
 * find dense path using steepest descent gradient method in
 * exploration transform map. then refine path by sampling rule
 * @param grid_map
 * @param start_pose
 * @param path
 * @param occupancy_layer
 * @param dist_trans_layer used to decide whether to reserve this sampling cell
 * @param expl_trans_layer
 * @return
 */
bool findPathExplorationTransform(grid_map::GridMap &grid_map, const hmpl::Pose2D &start_pose,
                                  std::vector<geometry_msgs::PoseStamped> &path,
                                  const std::string occupancy_layer = "occupancy",
                                  const std::string dist_trans_layer = "distance_transform",
                                  const std::string expl_trans_layer = "exploration_transform");
// judge start point is whether within obstacle zone or not.
// if within obstacle zone, start searching vaild start point in the vicinity of original start point
bool adjustStartPoseIfOccupied(const grid_map::GridMap &grid_map, const hmpl::Pose2D &start_pose,
                               hmpl::Pose2D &revised_start_pose,
                               const std::string occupancy_layer = "occupancy",
                               const std::string dist_trans_layer = "distance_transform",
                               const std::string expl_trans_layer = "exploration_transform");

/**
 * find safety cell meeting the requirement of distance from obstacle
 * @param grid_map
 * @param start_index
 * @param adjusted_index
 * @param allowed_distance_from_start search scope limitation, not used now
 * @param required_final_distance at least safety distance
 * @param desired_final_distance most safety distance
 * @param occupancy_layer
 * @param dist_trans_layer
 * @param expl_trans_layer
 * @return
 */
bool findValidClosePoseExplorationTransform(const grid_map::GridMap &grid_map, const grid_map::Index &start_index,
                                            grid_map::Index &adjusted_index,
                                            const float allowed_distance_from_start = 3.0,
                                            const float required_final_distance = 6.0,
                                            const float desired_final_distance = 12.0,
                                            const std::string occupancy_layer = "occupancy",
                                            const std::string dist_trans_layer = "distance_transform",
                                            const std::string expl_trans_layer = "exploration_transform");
// sample the dense path according to the rule:
// sample more in "dangerous" position, sample less in "open" position
bool shortCutPath(grid_map::GridMap &grid_map, const std::vector<grid_map::Index> &path_in,
                  std::vector<grid_map::Index> &path_out, const std::string dist_trans_layer = "distance_transform",
                  const std::string expl_trans_layer = "exploration_transform");

/**
 * find most safety neighberhood cell with maximum distance transform value
 * @param dist_trans_map
 * @param current_point
 * @param idx_x
 * @param idx_y
 * @param highest_val
 * @param highest_index
 */
void touchDistanceField(const grid_map::Matrix &dist_trans_map, const grid_map::Index &current_point, const int idx_x,
                   const int idx_y, float &highest_val, grid_map::Index &highest_index);

// calculate descending gradient value
void touchGradientCell(const grid_map::Matrix &expl_trans_map, const grid_map::Index &current_point, const int idx_x,
                  const int idx_y, float &lowest_val, grid_map::Index &lowest_index);
// calculate ascending gradient value
void touchInverseGradientCell(const std::vector<std::pair<int, int> > &visited, const grid_map::Matrix &expl_trans_map, const grid_map::Index &current_point, const int idx_x,
                              const int idx_y, float &lowest_val, grid_map::Index &lowest_index);
// judge cell is whether close to obstacle cell or not
// return false meaning that need to reserve this cell in result path
bool shortCutValid(const grid_map::GridMap &grid_map, const grid_map::Matrix &dist_trans_map,
                   const grid_map::Index &start_point, const grid_map::Index &end_point);
size_t size_x_lim, size_y_lim;
} // namespace grid_map_path_planning

#endif //PATH_TRANSFORM_PATH_PLANNING_HPP
