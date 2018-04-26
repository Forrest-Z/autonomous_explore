#ifndef EXPLORATION_EXPLORATION_PLANNER_H
#define EXPLORATION_EXPLORATION_PLANNER_H

#include <ros/ros.h>

#include "coverage_exploration/ExplorationPlannerTypes.hpp"
#include "coverage_exploration/Config.hpp"
#include "coverage_exploration/pose_handler.h"
#include "coverage_exploration/VisMarker.h"
#include "coverage_exploration/Color.h"
#include "coverage_exploration/PoseWrap.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <chrono>


namespace exploration {
    /**
     * Used to store data for each exploration point. 
     * After all expl. points have been processed the overallValue
     * can be calculated which is used to sort the points.
     */
    struct ExplorationPoint {

        ExplorationPoint(Pose expl_pose, unsigned int num_expl_cells, double ang_dist, double robot_point_dist,
                         double worst_driveability, bool edge_found) : explPose(expl_pose),
                                                                       numberOfExploredCells(num_expl_cells),
                                                                       angularDistance(ang_dist),
                                                                       robotPointDistance(robot_point_dist),
                                                                       worstDriveability(worst_driveability),
                                                                       edgeFound(edge_found), expl_value(0.0),
                                                                       ang_value(0.0), dist_value(0.0),
                                                                       driveability_value(0.0), edge_value(0.0) {
        }

        bool operator<(const ExplorationPoint &rhs) const {
            return overallValue < rhs.overallValue;
        }

        /**
         * Calculates the overall value of this exploratin point.
         * If max_explored_cells or max_robot_goal_dist is 0, the explored cells
         * or the the distance of the goal point is ignored.
         * The bigger the better.
         */
        double calculateOverallValue(Weights &weights, double max_explored_cells, double max_robot_goal_dist) {
            if (max_explored_cells != 0) {
                expl_value = weights.explCells * (numberOfExploredCells / max_explored_cells);
            }
            ang_value = weights.angDist * (1 - (angularDistance / M_PI));
            if (max_robot_goal_dist != 0) {
                dist_value = weights.robotGoalDist * (1 - (robotPointDistance / max_robot_goal_dist));
            }
            driveability_value = weights.driveability * worstDriveability;
            edge_value = edgeFound ? weights.edgeDetected : 0;
            overallValue = expl_value + ang_value + dist_value + driveability_value + edge_value;
            return overallValue;
        }

        double overallValue;
        Pose explPose;
        unsigned int numberOfExploredCells;
        double angularDistance;
        double robotPointDistance;
        double worstDriveability;
        // Is set to true if an edge has been searched and found or if 
        // the point is close to the robot and no edge detection has been required.
        // Otherwise far away edge points would win against close points for 
        // which an edge detection has not been required.
        bool edgeFound;

        double expl_value;
        double ang_value;
        double dist_value;
        double driveability_value;
        double edge_value;
    };

    class Planner {
    public:
        Planner(Config config = Config());

        ~Planner();

        /** callback function every time a new occupied map is received! */
        void updateCycle(const nav_msgs::OccupancyGridConstPtr &map);

        /** Returns the result of the last operation */
        Status getStatus() const { return mStatus; }

        /** Returns the result message of the last operation */
        const char *getStatusMessage() const { return mStatusMessage; }

        /** Returns a list of reachable frontier cells ordered by distance to 'start' */
        PointList getFrontierCells(GridMap *map, GridPoint start, bool stopAtFirst = false);

        /** Returns a set of connected frontiers ordered by distance to 'start' */
        FrontierList getFrontiers(GridMap *map, GridPoint start);

        /** Returns the number of reachable frontier cells after detection */
        unsigned int getFrontierCellCount() const { return mFrontierCellCount; }

        /** Initializes a new coverage map from a given grid map
         *  All cells marked 'free' in the map are marked 'uncovered' in the coverage map */
        void initCoverageMap(GridMap *map);

        /** Set a cell to a given value (used to add new obstacles) */
        void setCoverageMap(PointList points, char value = 1);

        /** Cover all cells within sensor footprint from the given pose. Uses the 'willBeExplored' function */
        void addReading(Pose p);

        /** Get all cells that are still uncovered */
        PointList getUnexploredCells() const;

        /**
         * Add a sensor footprint to the internal sensor field 
         * Polygon has to be defined in grid coordinates.
         */
        void addSensor(Polygon p);

        /** Coordinate-transgform the given polygon to pose */
        Polygon transformPolygon(const Polygon &polygon, Pose pose);

        /** Get reachable Frontiers in coverage map from given pose **/
        FrontierList getCoverageFrontiers(Pose start);

        /** (Experimental!)
         * Currently returns nearest frontier cell in coverage map */
        Pose getCoverageTarget(Pose start);

        const GridMap &getCoverageMap() const;

        /**
         * Uses the exploration map to calculate the orientation of the goal point
         * which should be orientated orthogonally to the known-unknown-border.
         * \param goal_point Requires grid coordinates.
         * \return If the passed point does not lies close to one of the found edges false is returned.
         */
        bool calculateGoalOrientation(struct Pose goal_point, double &orientation, bool show_debug = false);

        /**
         * Get the point with least angular difference to robotpose. Uses compare-function for sorting.
         * If calculate_worst_driveability is set to true the robot length and with have to be specified as well.
         * In this case obstacle poses are ignored and the worst driveability is used
         * within the cost calculations.
         * \param robotPose Expects local coordinates in meter.
         */
        std::vector<Pose> getCheapest(std::vector<Pose> &pts, Pose &robotPose, bool calculate_worst_driveability = false,
                    double robot_length_x = 0, double robot_width_y = 0);

        /**
         * Clears the complete map / sets everything to UNKNOWN.
         * Receiving the next trav map will reset all OBSTACLES again.
         */
        inline void clearCoverageMap() {
            mCoverageMap->clearData();
        }

    private:

        void publishFootPrint(const geometry_msgs::Pose &pose, const std::string &frame);

        void updateMap();

        void generateGoals();

        bool isFree(Pose pos);

        void publishMarkerArray(const std::vector<Pose> &array, std::vector<int> type);

        Config mConfig;

        PointList getNeighbors(GridPoint p, bool diagonal = false) const;

        PointList getFrontier(GridMap *map, GridMap *plan, GridPoint start);

        bool isFrontierCell(GridMap *map, GridPoint point) const;

        bool pointInPolygon(FloatPoint point, Polygon polygon) const;

        bool isVisible(FloatPoint point, Pose pose) const;

        SensorField transformSensorField(Pose pose);

        /** returns all cells within the sensor footprint from the given pose */
        PointList willBeExplored(Pose p);

        Status mStatus;
        char mStatusMessage[500];

        unsigned int mFrontierCellCount;
        char mFrontierCount;

        GridMap *mCoverageMap;

        SensorField mSensorField;

        /**
         * Maps the passed angle to [0, 2*PI)
         */
        double map0to2pi(double angle_rad);


        bool included(FloatPoint point, std::vector<Polygon> &regions);


        /**
         * Max distant sensor polygon point in grid coordinates.
         */
        double mMaxDistantSensorPoint;

        /**
         * Counts the black(0) pixels starting at 'pose' using the contained direction.
         * vec_len_px defines the final length of the vector. E.g. vec_len_px == 10
         * means that vec from 1 to 10 are used to count the black pixels.
         * TODO: Currently a pixel may be counted twice, but this should not be a problem
         * because we do not need accurate results, just a tendency.
         */
        int countBlackPixels(cv::Mat mat, struct Pose pose, int vec_len_px = 10);


        bool initialized_;

        double xinit_;  //!< Map x position at initialization
        double yinit_;  //!< Map y position at initialization
        Pose current_pose_;
        Pose map_origin_;
        double map_solution_;
        unsigned int map_width_;
        unsigned int map_height_;
        std::string frame_name_;
        double border_thickness_;
        int obs_threshold_;
        double polygon_height_;
        double polygon_base_length_;
        double base2camera_length_;
        double left_base2camera_length_;


        std::string local_map_frame_name_, global_map_frame_name_;
        tfhandler::PoseHandler pose_hander_;
        ros::NodeHandle nh_;
        ros::NodeHandlePtr n_;
        ros::Publisher cover_map_pub_;
        ros::Publisher mMarkerPub_;
        ros::Publisher vehicle_footprint_pub_;


    };
}

#endif
