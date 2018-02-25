/* Copyright (c) 2017, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: internal_grid_map.cpp
 *   Created on: Mar, 23, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include "internal_grid_map/internal_grid_map.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

namespace hmpl {

using grid_map::GridMapCvConverter;
// for now, just keep two layers : obstacle layer and distance map layer
// later on, the gradient or hessian map may be added in.
InternalGridMap::InternalGridMap()
    : obs("obstacle"),
      dis("distance"),
      inflate("inflated_obstacle"),
      deflate("deflated_obstacle"),
      explore_transform("explre_transform"),
      maps(grid_map::GridMap(std::vector<std::string>{obs, dis, inflate, deflate, explore_transform})) {}

InternalGridMap::~InternalGridMap() {}
bool InternalGridMap::init(const std::string frame_id,
                           const grid_map::Length &length,
                           const double resolution,
                           const grid_map::Position &position) {
    this->maps.setFrameId(frame_id);
    this->maps.setGeometry(length, resolution, position);
}

bool InternalGridMap::initializeFromImage(const cv::Mat &image,
                                          const double resolution,
                                          const grid_map::Position &position) {
    return grid_map::GridMapCvConverter::initializeFromImage(
            image, resolution, this->maps, position);
}

// this method only handle gray image,
// 0 refers to obstacles and 255 refers to freespace
bool InternalGridMap::addObstacleLayerFromImage(const cv::Mat &image,
                                                const double alpha_threshold) {
    return GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
            image, this->obs, this->maps, OCCUPY, FREE, alpha_threshold);
}

bool InternalGridMap::updateDistanceLayer() {
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
            this->maps.get(this->obs).cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(this->maps.get(this->dis))
            , CV_DIST_L2, CV_DIST_MASK_PRECISE);
    this->maps.get(this->dis) *= this->maps.getResolution();
    return true;
}

bool InternalGridMap::updateDistanceLayerCV() {
    cv::Mat obs_f, obs_u;
    obs_f = hmpl::eigen2cv(this->maps.get(this->obs));
    obs_f.convertTo(obs_u, CV_8UC1);
    cv::distanceTransform(obs_u, eigen2cv(this->maps.get(this->dis)), CV_DIST_L2, CV_DIST_MASK_PRECISE);
    this->maps.get(this->dis) *= this->maps.getResolution();
}

bool InternalGridMap::updateInflatedLayer(int inflation_radius_map_cells) {
    if (!this->maps.exists(this->obs))
        return false;
    cv::Mat obs_f, obs_u, inflated_f;
    obs_f = hmpl::eigen2cv(this->maps.get(this->obs));
    obs_f.convertTo(obs_u, CV_8UC1);
    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = inflation_radius_map_cells;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );
    obs_u = cv::Scalar::all(255) - obs_u;
    cv::dilate(obs_u, inflated_f, element);
    inflated_f = cv::Scalar::all(255) - inflated_f;
    grid_map::Matrix grid_data;
    cv::cv2eigen(inflated_f, grid_data);
    this->maps.add(this->inflate, grid_data);

//    cv::Mat mat;
//    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(this->maps, this->inflate, CV_8UC1, 0, 255, mat);
//    cv::namedWindow("inflated_map");
//    cv::imshow("inflated_map", mat);
//    cv::waitKey(-1);

}

bool InternalGridMap::updateDeflatedLayer( int deflation_radius_map_cells) {
    if (!this->maps.exists(this->obs))
        return  false;
    cv::Mat obs_f, obs_u, deflated_u;
    eigen2cv(this->maps.get(this->obs), obs_f);
    obs_f.convertTo(obs_u, CV_8UC1);
    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = deflation_radius_map_cells;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );
    obs_u = cv::Scalar::all(255) - obs_u;
    cv::erode(obs_u, deflated_u, element);
    deflated_u = cv::Scalar::all(255) - deflated_u;
    grid_map::Matrix grid_data;
    cv::cv2eigen(deflated_u, grid_data);
    this->maps.add(this->deflate, grid_data);
}

bool InternalGridMap::updateExplorationTransform(const std::vector<grid_map::Index>& goal_points,
                                                 const float lethal_dist,
                                                 const float penalty_dist,
                                                 float alpha,
                                                 bool use_cell_danger) {
    if (!this->maps.exists(obs))
        return false;

    if (!this->maps.exists(dis))
        return false;

    this->maps.add(this->explore_transform, std::numeric_limits<float>::max());
    grid_map::Matrix& expl_layer (this->maps[explore_transform]);

    std::queue<grid_map::Index> point_queue;

    for (size_t i = 0; i < goal_points.size(); ++i){
        const grid_map::Index& point = goal_points[i];
        expl_layer(point(0), point(1)) = 0.0;
        point_queue.push(point);
    }

    size_t size_x_lim = this->maps.getSize()(0) -1;
    size_t size_y_lim = this->maps.getSize()(1) -1;

    float adjacent_dist = 1;
    float diagonal_dist = sqrt(2);
    // don't consider discomfort
    if(!use_cell_danger) {
        alpha = 0;
    }
    //std::cout << "pq size:" << point_queue.size() << "\n";

    while (point_queue.size()){
        grid_map::Index point (point_queue.front());
        point_queue.pop();

        //Reject points near border here early as to not require checks later
        if (point(0) < 1 || point(0) >= size_x_lim ||
            point(1) < 1 || point(1) >= size_y_lim){
            continue;
        }

        float current_val = expl_layer(point(0), point(1));

        touchExplorationCell(point(0)-1,
                             point(1)-1,
                             current_val,
                             diagonal_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0),
                             point(1)-1,
                             current_val,
                             adjacent_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0)+1,
                             point(1)-1,
                             current_val,
                             diagonal_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0)-1,
                             point(1),
                             current_val,
                             adjacent_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0)+1,
                             point(1),
                             current_val,
                             adjacent_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0)-1,
                             point(1)+1,
                             current_val,
                             diagonal_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0),
                             point(1)+1,
                             current_val,
                             adjacent_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);

        touchExplorationCell(point(0)+1,
                             point(1)+1,
                             current_val,
                             diagonal_dist,
                             lethal_dist,
                             penalty_dist,
                             point_queue,
                             alpha);
    }
    return true;

}

void InternalGridMap::touchExplorationCell(const int idx_x, const int idx_y, const float curr_val,
                                           const float add_cost, const float lethal_dist,
                                           const float penalty_dist, std::queue<grid_map::Index> &point_queue,
                                           float p_alpha) {

    //If not free at cell, return right away
    if (this->maps[obs](idx_x, idx_y) != FREE)
        return;

    float dist = this->maps[dis](idx_x, idx_y) / this->maps.getResolution();

    if (dist < lethal_dist)
        return;

    float cost = curr_val + add_cost;

    if (dist < penalty_dist){
        float add_cost = (penalty_dist - dist);
        cost += p_alpha * add_cost * add_cost;
    }

    if (this->maps[explore_transform](idx_x, idx_y) > cost){
        this->maps[explore_transform](idx_x, idx_y) = cost;
        point_queue.push(grid_map::Index(idx_x, idx_y));
    }
}


float InternalGridMap::getObstacleDistance(const grid_map::Index &index) const {
    return this->maps.at(this->dis, index);
}

float InternalGridMap::getObstacleDistance(
        const grid_map::Position &pos) const {
    return this->maps.atPosition(this->dis, pos);
}

float InternalGridMap::getExplorationTFValue(const grid_map::Index& index) const {
    return this->maps.at(this->explore_transform, index);
}

float InternalGridMap::getExplorationTFValue(const grid_map::Position& pos) const {
    return this->maps.atPosition(this->explore_transform, pos);
}

}  // namespace hmpl
