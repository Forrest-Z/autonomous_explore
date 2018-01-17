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
    igm.updateDistanceLayer();
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
