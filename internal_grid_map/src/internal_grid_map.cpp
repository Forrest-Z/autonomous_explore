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
      maps(grid_map::GridMap(std::vector<std::string>{obs, dis})) {}

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

    cv::distanceTransform(eigen2cv(binary), eigen2cv(this->maps.get(this->dis)),
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
    this->maps.get(this->dis) *= this->maps.getResolution();
    return true;
}

bool InternalGridMap::updateDistanceLayerCV() {
    cv::Mat obs_f = eigen2cv(this->maps.get(this->obs));
    cv::Mat obs_u;
    obs_f.convertTo(obs_u, CV_8UC1);
    cv::distanceTransform(obs_u, eigen2cv(this->maps.get(this->dis)),
                               CV_DIST_L2, CV_DIST_MASK_PRECISE);
    this->maps.get(this->dis) *= this->maps.getResolution();
}

float InternalGridMap::getObstacleDistance(const grid_map::Index &index) const {
    return this->maps.at(this->dis, index);
}

float InternalGridMap::getObstacleDistance(
        const grid_map::Position &pos) const {
    return this->maps.atPosition(this->dis, pos);
}

}  // namespace internal_grid_map
