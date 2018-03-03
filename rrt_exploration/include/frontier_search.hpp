//
// Created by kevin on 2/28/18.
//

#ifndef RRT_EXPLORATION_FRONTIER_SEARCH_HPP
#define RRT_EXPLORATION_FRONTIER_SEARCH_HPP

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "functions.h"
#include <boost/foreach.hpp>

namespace frontier_exploration{
/**
 * @brief Thread-safe implementation of a frontier-search task for an input costmap.
 */
    class Frontier {
    public:
        int size;
        float min_distance;
        geometry_msgs::Point initial;
        geometry_msgs::Point centroid;
        geometry_msgs::Point middle;
    };

    class FrontierSearch{

    public:

        bool nearestCell(unsigned int &result, unsigned int start, int val);
        std::vector<unsigned int> nhood4(unsigned int idx, unsigned int width, unsigned int height);
        std::vector<unsigned int> nhood8(unsigned int idx, unsigned int width, unsigned int height);
        void indexToReal(const nav_msgs::OccupancyGrid& map, const size_t index, float &x, float &y);

        /**
         * @brief Constructor for search task
         * @param costmap Reference to costmap data to search.
         */
        FrontierSearch(nav_msgs::OccupancyGrid &mapData);

        /**
         * @brief Runs search implementation, outward from the start position
         * @param position Initial position to search from
         * @return List of frontiers, if any
         */
        std::list<Frontier> searchFrom(std::vector<float> position);



    protected:

        /**
         * @brief Starting from an initial cell, build a frontier from valid adjacent cells
         * @param initial_cell Index of cell to start frontier building
         * @param reference Reference index to calculate position from
         * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
         * @return
         */
        Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag);

        /**
         * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
         * @param idx Index of candidate cell
         * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
         * @return
         */
        bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);

    private:

        nav_msgs::OccupancyGrid& map_;
        std::vector<signed char> map_data_;
        unsigned int size_x_ , size_y_;
        float resolution_, Xstarty_, Xstartx_;
    };

}

#endif //RRT_EXPLORATION_FRONTIER_SEARCH_HPP
