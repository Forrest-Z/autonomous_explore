/* Copyright (c) 2017, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: internal_grid_map_test.cpp
 *   Created on: Mar, 23, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */

#include "internal_grid_map/internal_grid_map.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <opencv2/core/eigen.hpp>
#include <vector>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(InternalGridMapTest, initializeFromImage) {
    hmpl::InternalGridMap igm;
    int rows, cols;
    rows = 5;
    cols = 4;
    double resolution = 0.1;
    cv::Mat image(rows, cols, CV_8UC1, 255);
    image.at<unsigned char>(1, 1) = 0;
    image.at<unsigned char>(4, 3) = 0;
    igm.initializeFromImage(image, resolution, grid_map::Position::Zero());
    EXPECT_EQ(rows, igm.maps.getSize()(0));
    EXPECT_EQ(cols, igm.maps.getSize()(1));
}

TEST(InternalGridMapTest, addObstacleLayerFromImage) {
    hmpl::InternalGridMap igm;
    int rows, cols;
    rows = 4;
    cols = 3;
    double resolution = 0.1;
    cv::Mat image(rows, cols, CV_8UC1, 255);
    image.at<unsigned char>(1, 1) = 0;
    image.at<unsigned char>(3, 2) = 0;
    igm.initializeFromImage(image, resolution, grid_map::Position::Zero());
    igm.addObstacleLayerFromImage(image, 0.5);

    EXPECT_FLOAT_EQ(0, igm.maps.at(igm.obs, grid_map::Index(1, 1)));
    EXPECT_FLOAT_EQ(0, igm.maps.at(igm.obs, grid_map::Index(3, 2)));
}

TEST(InternalGridMapTest, updateDistanceLayer) {
    hmpl::InternalGridMap igm;
    int rows, cols;
    rows = 4;
    cols = 3;
    double resolution = 0.1;
    cv::Mat image(rows, cols, CV_8UC1, 255);
    image.at<unsigned char>(1, 1) = 0;
    image.at<unsigned char>(3, 2) = 0;
    igm.initializeFromImage(image, resolution, grid_map::Position::Zero());
    igm.addObstacleLayerFromImage(image, 0.5);
    igm.updateDistanceLayerCV();
    using grid_map::Index;

    EXPECT_FLOAT_EQ(sqrt(2.0) * resolution,
                    igm.getObstacleDistance(Index(0, 0)));
    EXPECT_FLOAT_EQ(1 * resolution, igm.getObstacleDistance(Index(0, 1)));
    EXPECT_FLOAT_EQ(sqrt(2.0) * resolution,
                    igm.getObstacleDistance(Index(0, 2)));

    EXPECT_FLOAT_EQ(1 * resolution, igm.getObstacleDistance(Index(1, 0)));
    EXPECT_FLOAT_EQ(0 * resolution, igm.getObstacleDistance(Index(1, 1)));
    EXPECT_FLOAT_EQ(1 * resolution, igm.getObstacleDistance(Index(1, 2)));

    EXPECT_FLOAT_EQ(sqrt(2.0) * resolution,
                    igm.getObstacleDistance(Index(2, 0)));
    EXPECT_FLOAT_EQ(1 * resolution, igm.getObstacleDistance(Index(2, 1)));
    EXPECT_FLOAT_EQ(1 * resolution, igm.getObstacleDistance(Index(2, 2)));

    EXPECT_FLOAT_EQ(2.0 * resolution, igm.getObstacleDistance(Index(3, 0)));
    EXPECT_FLOAT_EQ(1 * resolution, igm.getObstacleDistance(Index(3, 1)));
    EXPECT_FLOAT_EQ(0 * resolution, igm.getObstacleDistance(Index(3, 2)));
}

TEST(InternalGridMapTest, updateExplorationTransformLayer) {
    hmpl::InternalGridMap igm;
    int rows, cols;
    rows = 5;
    cols = 4;
    double resolution = 0.1;
    cv::Mat image(rows, cols, CV_8UC1, 255);
    image.at<unsigned char>(1, 1) = 0;
    image.at<unsigned char>(3, 2) = 0;
    igm.initializeFromImage(image, resolution, grid_map::Position::Zero());
    igm.addObstacleLayerFromImage(image, 0.5);
    igm.updateDistanceLayerCV();
    std::vector<grid_map::Index> goal;
    goal.push_back(grid_map::Index(3, 0));
    goal.push_back(grid_map::Index(2, 1));
    double lethal_dist = 0;
    double penalty_dist = 5;
    igm.updateExplorationTransform(goal, lethal_dist, penalty_dist); // ! : use cell unit
    using grid_map::Index;
    double inf = std::numeric_limits<float>::max();
    double value_3_0, value_2_1;
    value_3_0 = value_2_1 = 0;
    double value_1_1, value_3_2;
    value_1_1 = value_3_2 = inf;
    double value_0_0 = inf;
    double value_1_0 = value_2_1 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(1, 0)) / resolution , 2);

    double value_1_2 = value_2_1 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(1, 2)) / resolution , 2);
        double value_0_1 = value_1_2 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(0, 1)) / resolution , 2);
        double value_0_2 = value_1_2 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(0, 2)) / resolution , 2);
        double value_0_3 = value_1_2 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(0, 3)) / resolution , 2);
        double value_1_3 = value_1_2 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(1, 3)) / resolution , 2);

    double value_2_0 = value_2_1 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(2, 0)) / resolution , 2);

    double value_2_2 = value_2_1 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(2, 2)) / resolution , 2);
        double value_2_3 = value_2_2 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(2, 3)) / resolution , 2);
        double value_3_3 = value_2_2 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(3, 3)) / resolution , 2);

    double value_3_1 = value_2_1 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(3, 1)) / resolution , 2);
        double value_4_0 = value_3_1 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(4, 0)) / resolution , 2);
        double value_4_1 = value_3_1 + 1 + pow(penalty_dist - igm.getObstacleDistance(Index(4, 1)) / resolution , 2);
        double vaule_4_2 = value_3_1 + sqrt(2) + pow(penalty_dist - igm.getObstacleDistance(Index(4, 2)) / resolution , 2);

    double value_4_3 = inf;
    EXPECT_FLOAT_EQ(value_0_0, igm.getExplorationTFValue(Index(0, 0)));
    EXPECT_FLOAT_EQ(value_0_1, igm.getExplorationTFValue(Index(0, 1)));
    EXPECT_FLOAT_EQ(value_0_2, igm.getExplorationTFValue(Index(0, 2)));
    EXPECT_FLOAT_EQ(value_0_3, igm.getExplorationTFValue(Index(0, 3)));


    EXPECT_FLOAT_EQ(value_1_0, igm.getExplorationTFValue(Index(1, 0)));
    EXPECT_FLOAT_EQ(value_1_1, igm.getExplorationTFValue(Index(1, 1)));
    EXPECT_FLOAT_EQ(value_1_2, igm.getExplorationTFValue(Index(1, 2)));
    EXPECT_FLOAT_EQ(value_1_3, igm.getExplorationTFValue(Index(1, 3)));


    EXPECT_FLOAT_EQ(value_2_0, igm.getExplorationTFValue(Index(2, 0)));
    EXPECT_FLOAT_EQ(value_2_1, igm.getExplorationTFValue(Index(2, 1)));
    EXPECT_FLOAT_EQ(value_2_2, igm.getExplorationTFValue(Index(2, 2)));
    EXPECT_FLOAT_EQ(value_2_3, igm.getExplorationTFValue(Index(2, 3)));

    EXPECT_FLOAT_EQ(value_3_0, igm.getExplorationTFValue(Index(3, 0)));
    EXPECT_FLOAT_EQ(value_3_1, igm.getExplorationTFValue(Index(3, 1)));
    EXPECT_FLOAT_EQ(value_3_2, igm.getExplorationTFValue(Index(3, 2)));
    EXPECT_FLOAT_EQ(value_3_3, igm.getExplorationTFValue(Index(3, 3)));

    EXPECT_FLOAT_EQ(value_4_0, igm.getExplorationTFValue(Index(4, 0)));
    EXPECT_FLOAT_EQ(value_4_1, igm.getExplorationTFValue(Index(4, 1)));
    EXPECT_FLOAT_EQ(vaule_4_2, igm.getExplorationTFValue(Index(4, 2)));
    EXPECT_FLOAT_EQ(value_4_3, igm.getExplorationTFValue(Index(4, 3)));
}