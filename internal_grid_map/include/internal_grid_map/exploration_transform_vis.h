//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef INTERNAL_GRID_MAP_EXPLORATION_TRANSFORM_VIS_H___
#define INTERNAL_GRID_MAP_EXPLORATION_TRANSFORM_VIS_H___

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace hmpl {

    class ExplorationTransformVis {
    public:
        ExplorationTransformVis(const std::string &topic_name) {
            ros::NodeHandle pnh("~");
            exploration_transform_pointcloud_pub_ = pnh.advertise<sensor_msgs::PointCloud>( topic_name, 2, false);
        }

        virtual ~ExplorationTransformVis() {}

        void publishVisOnDemand(const grid_map::GridMap &gridmap, const std::string &explore_layer) {
            if (exploration_transform_pointcloud_pub_.getNumSubscribers() > 0) {
                unsigned int size_x = gridmap.getSize()(0);
                unsigned int size_y = gridmap.getSize()(1);

                float max = 0;
                double inf = std::numeric_limits<float>::max();
                const grid_map::Matrix &expl_layer(gridmap[explore_layer]);

                for (size_t x = 0; x < size_x; ++x) {
                    for (size_t y = 0; y < size_y; ++y) {
                        if ((expl_layer(x, y) < inf) && (expl_layer(x, y) > max)) {
                            max = expl_layer(x, y);
                        }
                    }
                }

                sensor_msgs::PointCloud cloud;
                cloud.header.frame_id = gridmap.getFrameId();
                cloud.header.stamp = ros::Time::now();

                geometry_msgs::Point32 point;
                grid_map::Position pose;

                for (size_t x = 0; x < size_x; ++x) {
                    for (size_t y = 0; y < size_y; ++y) {
                        if (expl_layer(x, y) < inf) {
                            grid_map::Index index(x, y);
                            gridmap.getPosition(index, pose);
                            point.x = pose(0);
                            point.y = pose(1);
                            point.z = expl_layer(x, y) / max;

                            cloud.points.push_back(point);
                        }
                    }
                }
                exploration_transform_pointcloud_pub_.publish(cloud);
            }
        }

    protected:

        ros::Publisher exploration_transform_pointcloud_pub_;
    };

} // namespace hmpl

#endif // INTERNAL_GRID_MAP_EXPLORATION_TRANSFORM_VIS_H___
