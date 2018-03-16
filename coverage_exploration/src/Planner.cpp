#include "coverage_exploration/Planner.hpp"

#include <map>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <algorithm> // sort
#include <list>


using namespace exploration;

typedef std::multimap<int, GridPoint> Queue;
typedef std::pair<int, GridPoint> Entry;

//#define DEBUG
#define CV_Dector
Planner::Planner(Config config) {
    mFrontierCellCount = 0;
    mFrontierCount = 0;
    mCoverageMap = nullptr;
    mMaxDistantSensorPoint = -1;
    mConfig = config;
    // todo push into mconfig parameters
    initialized_ = false;

    cover_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("cover_map", 1, true);
    mMarkerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("frontier_marker", 2);

}

Planner::~Planner() {
    if (mCoverageMap)
        delete mCoverageMap;
}

PointList Planner::getNeighbors(GridPoint p, bool diagonal) const {
    PointList neighbors;
    GridPoint n;

    n.x = p.x - 1;
    n.y = p.y;
    neighbors.push_back(n);
    n.x = p.x;
    n.y = p.y - 1;
    neighbors.push_back(n);
    n.x = p.x + 1;
    n.y = p.y;
    neighbors.push_back(n);
    n.x = p.x;
    n.y = p.y + 1;
    neighbors.push_back(n);

    if (diagonal) {
        n.x = p.x - 1;
        n.y = p.y - 1;
        neighbors.push_back(n);
        n.x = p.x - 1;
        n.y = p.y + 1;
        neighbors.push_back(n);
        n.x = p.x + 1;
        n.y = p.y - 1;
        neighbors.push_back(n);
        n.x = p.x + 1;
        n.y = p.y + 1;
        neighbors.push_back(n);
    }
    return neighbors;
}

bool Planner::isFrontierCell(GridMap *map, GridPoint point) const {
    char c = 0;
    if (map->getData(point, c) && c != VISIBLE)
        return false;

    PointList neighbors = getNeighbors(point, true);
    for (PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++) {
        char c = 0;
        if (map->getData(*cell, c) && c == UNKNOWN) {
            return true;
        }
    }

    return false;
}

PointList Planner::getFrontierCells(GridMap *map, GridPoint start, bool stopAtFirst) {
    // Initialization
    mFrontierCellCount = 0;
    GridMap plan = GridMap(map->getWidth(), map->getHeight());
    PointList result;

    // Initialize the queue with the robot position
    Queue queue;
    queue.insert(Entry(0, start));
    plan.setData(start, 0);

    // Do full search with weightless Dijkstra-Algorithm
    while (!queue.empty()) {
        // Get the nearest cell from the queue
        Queue::iterator next = queue.begin();
        int distance = next->first;
        GridPoint point = next->second;
        queue.erase(next);
        bool foundFrontier = false;

        // Add all adjacent cells
        PointList neighbors = getNeighbors(point);
        for (PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++) {
            char c = 0;
            if (map->getData(*cell, c) && c == UNKNOWN) {
                foundFrontier = true;
                continue;
            }

            if (map->getData(*cell, c) && c == VISIBLE && plan.getData(*cell, c) && c == UNKNOWN) {
                queue.insert(Entry(distance + 1, *cell));
                plan.setData(*cell, 0);
            }
        }

        if (foundFrontier) {
            GridPoint frontier = point;
            frontier.distance = distance;
            result.push_back(frontier);
            mFrontierCellCount++;
            if (stopAtFirst) break;
        }
    }

    // Set result message and return the point list
    if (result.size() > 0) {
        mStatus = SUCCESS;
        sprintf(mStatusMessage, "Found %d reachable frontier cells.", (int) result.size());
    } else {
        mStatus = NO_GOAL;
        sprintf(mStatusMessage, "No reachable frontier cells found.");
    }
    return result;
}

FrontierList Planner::getFrontiers(GridMap *map, GridPoint start) {
    // Initialization
    mFrontierCellCount = 0;
    mFrontierCount = 0;
    GridMap plan = GridMap(map->getWidth(), map->getHeight());
    FrontierList result;

    // Initialize the queue with the robot position
    Queue queue;
    queue.insert(Entry(0, start));
    // UNKNOWN: no visit
    // OBSTACLE: frontier
    // VISIBLE：visited
    plan.setData(start, VISIBLE);

    // Do full search with weightless Dijkstra-Algorithm
    while (!queue.empty()) {
        // Get the nearest cell from the queue
        Queue::iterator next = queue.begin();
        int distance = next->first;
        GridPoint point = next->second;
        queue.erase(next);

        // Add neighbors
        bool isFrontier = false;
        PointList neighbors = getNeighbors(point, false);
        char c = 0;
        for (PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++) {
            if (map->getData(*cell, c) && c == UNKNOWN) {
                plan.setData(*cell, OBSTACLE);
                isFrontier = true;
                break;
            }
            if (map->getData(*cell, c) && c == VISIBLE && plan.getData(*cell, c) && c == UNKNOWN) {
                queue.insert(Entry(distance + 1, *cell));
                plan.setData(*cell, VISIBLE);
            }
        }

        if (isFrontier) {
            result.push_back(getFrontier(map, &plan, point));
        }
    }

    // Set result message and return the point list
    if (result.size() > 0) {
        mStatus = SUCCESS;
        sprintf(mStatusMessage, "Found %d frontiers with %d frontier cells.", mFrontierCount, mFrontierCellCount);
    } else {
        mStatus = NO_GOAL;
        sprintf(mStatusMessage, "No reachable frontiers found.");
    }
    return result;
}

PointList Planner::getFrontier(GridMap *map, GridMap *plan, GridPoint start) {
    // Mark the cell as "already added to a frontier" by setting
    // the value in the plan to 2(OBSTACLE).
    PointList frontier;
    mFrontierCount++;

    // Initialize the queue with the first frontier cell
    Queue queue;
    queue.insert(Entry(0, start));
    plan->setData(start, OBSTACLE);

    // Do full search with weightless Dijkstra-Algorithm
    while (!queue.empty()) {
        // Get the nearest cell from the queue
        Queue::iterator next = queue.begin();
        int distance = next->first;
        GridPoint point = next->second;
        queue.erase(next);

        // Add it to the frontier
        frontier.push_back(point);
        mFrontierCellCount++;

        // Add all adjacent frontier cells to the queue
        PointList neighbors = getNeighbors(point, true);
        char c = 0;
        for (PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++) {
            // new and vaild adjacent frontier cells
            if (plan->getData(*cell, c) && c != OBSTACLE && isFrontierCell(map, *cell)) {
                plan->setData(*cell, OBSTACLE);
                queue.insert(Entry(distance + 1, *cell));
            }
        }
    }

    return frontier;
}

void Planner::initCoverageMap(GridMap *map) {
    if (mCoverageMap) delete mCoverageMap;

    unsigned int width = map->getWidth();
    unsigned int height = map->getHeight();
    mCoverageMap = new GridMap(width, height);

    memcpy(mCoverageMap->getData(), map->getData(), sizeof(char) * width * height);
}

void Planner::setCoverageMap(PointList points, char value) {
    PointList::iterator i;
    for (i = points.begin(); i < points.end(); i++) {
        mCoverageMap->setData(*i, value);
    }
}

void Planner::addReading(Pose p) {
    PointList points = willBeExplored(p);
    // deprecated!
//    clearVehicleBodyArea(p);  // vehicle body current cover area is visible, unnecessary to search!
    for (PointList::iterator i = points.begin(); i < points.end(); ++i) {
        mCoverageMap->setData(*i, VISIBLE);
    }
    ROS_DEBUG("add %d cell into new visible label!", points.size());
}

PointList Planner::willBeExplored(Pose p) {
    auto start = std::chrono::system_clock::now();

    // todo use lookup table ,faster!
    SensorField transformedSF = transformSensorField(p);

    // Rasterize transformed SensorField
    SensorField::iterator sensor;
    Polygon::iterator point;
    FloatPoint min, max, current;
    PointList result;
    for (sensor = transformedSF.begin(); sensor < transformedSF.end(); sensor++) {
        // Determine bounding box for efficiency
        min = max = *(sensor->begin());
        for (point = sensor->begin() + 1; point < sensor->end(); point++) {
            if (point->x < min.x) min.x = point->x;
            if (point->x > max.x) max.x = point->x;
            if (point->y < min.y) min.y = point->y;
            if (point->y > max.y) max.y = point->y;
        }

        // Rasterize polygon
        for (int y = min.y; y <= max.y; y++) {
            for (int x = min.x; x <= max.x; x++) {
                current.x = x;
                current.y = y;
                GridPoint gp;
                gp.x = x;
                gp.y = y;
                char c = 0;

                if (mCoverageMap->getData(gp, c) && c == UNKNOWN && pointInPolygon(current, *sensor) &&
                    isVisible(current, p)) {
                    result.push_back(gp);
                }
            }
        }
    }
    auto end = std::chrono::system_clock::now();
    auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//    ROS_INFO("Imagine sensor area msec: %lf", msec);
    return result;
}


// http://alienryderflex.com/polygon/
bool Planner::pointInPolygon(FloatPoint point, Polygon polygon) const {
    int j = polygon.size() - 1;
    bool oddNodes = false;

    for (unsigned int i = 0; i < polygon.size(); i++) {
        if ((polygon[i].y < point.y && polygon[j].y >= point.y) ||
            (polygon[j].y < point.y && polygon[i].y >= point.y)) {
            if (polygon[i].x +
                (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) * (polygon[j].x - polygon[i].x) < point.x) {
                oddNodes = !oddNodes;
            }
        }
        j = i;
    }
    return oddNodes;
}

bool Planner::isVisible(FloatPoint point, Pose pose) const {
    double x = pose.x;
    double y = pose.y;

    double delta_x = point.x - pose.x;
    double delta_y = point.y - pose.y;
    double delta = sqrt((delta_x * delta_x) + (delta_y * delta_y));
    int step = delta;
    double step_x = delta_x / step;
    double step_y = delta_y / step;
    char c = 0;

    for (int i = 0; i < step; i++) {
        GridPoint p;
        p.x = x;
        p.y = y;
        // not to consider point in previous scanned area
        // todo consider obstcle in line
        if (mCoverageMap->getData(p, c) && c == OBSTACLE)
            return false;

        x += step_x;
        y += step_y;
    }
    return true;
}

SensorField Planner::transformSensorField(Pose pose) {
    SensorField::const_iterator sensor;
    SensorField field;
    for (sensor = mSensorField.begin(); sensor < mSensorField.end(); sensor++) {
        field.push_back(transformPolygon(*sensor, pose));
    }
    return field;
}

Polygon Planner::transformPolygon(const Polygon &polygon, Pose pose) {
    Polygon polyTransform;
    Polygon::const_iterator p;
    for (p = polygon.begin(); p < polygon.end(); p++) {
        double x_ = (p->x * cos(-pose.theta)) + (p->y * sin(-pose.theta));
        double y_ = -(p->x * sin(-pose.theta)) + (p->y * cos(-pose.theta));
        polyTransform.push_back(FloatPoint(pose.x + x_, pose.y + y_));
    }
    return polyTransform;
}

PointList Planner::getUnexploredCells() const {
    PointList result;
    GridPoint p;
    char c = 0;

    for (unsigned int y = 0; y < mCoverageMap->getHeight(); y++) {
        for (unsigned int x = 0; x < mCoverageMap->getWidth(); x++) {
            p.x = x;
            p.y = y;
            if (mCoverageMap->getData(p, c) && c == UNKNOWN) {
                result.push_back(p);
            }
        }
    }
    return result;
}

void Planner::addSensor(Polygon p) {
    mSensorField.push_back(p);

    // Find max distant polygon point of all added sensors.
    // Used to determine which type of orientation calculation shuld be used. 
    std::vector<FloatPoint>::iterator it = p.begin();
    double len;
    for (; it != p.end(); it++) {
        len = sqrt(pow(it->x, 2) + pow(it->y, 2));
        std::cout << "len " << len << " x " << it->x << " y " << it->y << std::endl;
        if (len > mMaxDistantSensorPoint) {
            mMaxDistantSensorPoint = len;
        }
    }
    std::cout << "Max len " << len << std::endl;
}

Pose Planner::getCoverageTarget(Pose start) {
    GridPoint startPoint;
    startPoint.x = start.x;
    startPoint.y = start.y;
    PointList fCells = getFrontierCells(mCoverageMap, startPoint);
    PointList::const_iterator p;
    Pose target;
    for (p = fCells.begin(); p < fCells.end(); p++) {
        target.x = p->x;
        target.y = p->y;
        std::cout << "possible goals: " << target.x << "/" << target.y;
        if (p->distance > 20 && p->distance < 30) break;
    }
    return target;
}

bool Planner::calculateGoalOrientation(struct Pose goal_point, double &orientation, bool show_debug) {
    int num_cells_radius = 12;
    bool ret = false;

    // Defines opencv image (area around the goal point).
    int min_x = goal_point.x - num_cells_radius;
    int max_x = goal_point.x + num_cells_radius;
    int min_y = goal_point.y - num_cells_radius;
    int max_y = goal_point.y + num_cells_radius;
    if (min_x < 0)
        min_x = 0;
    if (min_y < 0)
        min_y = 0;
    if (max_x > (int) mCoverageMap->getWidth())
        max_x = mCoverageMap->getWidth();
    if (max_y > (int) mCoverageMap->getHeight())
        max_y = mCoverageMap->getHeight();

    int width_cv = max_x - min_x;
    int height_cv = max_y - min_y;
    if (width_cv <= 0 || height_cv <= 0) {
        ROS_WARN("Opencv image size is too small (%d, %d), orientation 0 will be returned");
        return false;
    }

    // Calculate goal position within the OpenCV image.
    struct Pose goal_point_cv;
    goal_point_cv.x = goal_point.x - min_x;
    goal_point_cv.y = goal_point.y - min_y;
    ROS_DEBUG("Calculate orientation for exploration point (%4.2f, %4.2f), opencv pixel (%d, %d)", goal_point.x,
              goal_point.y, goal_point_cv.x, goal_point_cv.y);

    // Copy image from the exploration map to the opencv image.
    // TODO Treat OBSTACLEs as unknown?
    cv::Mat mat_src(height_cv, width_cv, CV_8UC1, cv::Scalar(0));
    char *coverage_map = mCoverageMap->getData();
    int c = 0;
    int x_cv = 0;
    for (int x = min_x; x < max_x; ++x, ++x_cv) {
        int y_cv = 0;
        for (int y = min_y; y < max_y; ++y, ++y_cv) {
            c = (int) coverage_map[y * mCoverageMap->getWidth() + x];
            if (c == VISIBLE || c == EXPLORED) {
                mat_src.at<uchar>(y_cv, x_cv) = 255;
            } else { // OBSTACLE or UNKNOWN
                mat_src.at<uchar>(y_cv, x_cv) = 0;
            }
        }
    }

    cv::Mat mat_canny, mat_canny_bgr;
    Canny(mat_src, mat_canny, 50, 200);
    std::vector<cv::Vec2f> lines;
    // Resolution: 1 px and 180/32 degree.
    HoughLines(mat_canny, lines, 1, CV_PI / 32, 10);

    if (show_debug) {
        cvtColor(mat_canny, mat_canny_bgr, CV_GRAY2BGR);
    }

    // Examines the found edges, choose the best one if available.
    cv::Point lowest_dist_pt1, lowest_dist_pt2;
    double shortest_dist = std::numeric_limits<double>::max();
    double expl_point_orientation = 0;
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));

        // Determine the exploration point which lies closest to the found edge.
        double dist_diff_px = fabs((goal_point_cv.x * a + goal_point_cv.y * b) - rho);
        if (dist_diff_px < shortest_dist) {
            lowest_dist_pt1 = pt1;
            lowest_dist_pt2 = pt2;
            shortest_dist = dist_diff_px;
            expl_point_orientation = theta;
        }
        if (show_debug) {
            line(mat_canny_bgr, pt1, pt2, cv::Scalar(0, 0, 64), 1, 8/*CV_AA*/);
        }
    }

    // Display windows.
    cv::Size size(300, 300);//the dst image size,e.g.100x100
    cv::Mat mat_src_resize, mat_canny_resize;
    resize(mat_src, mat_src_resize, size);
    resize(mat_canny_bgr, mat_canny_resize, size);


    imshow("source", mat_src_resize);
    imshow("detected lines", mat_canny_resize);

    cv::waitKey(-1);


    // Calculate correct orientation (theta or theta+M_PI) by counting black pixels(UNKNOWN).
    // So, the direction with more black pixels is the direction the exploration should look at.
    int num_pixels_to_check = 10;
    goal_point_cv.theta = expl_point_orientation;
    int count_theta = countBlackPixels(mat_src, goal_point_cv, num_pixels_to_check);
    goal_point_cv.theta = expl_point_orientation + M_PI;
    int count_theta_pi = countBlackPixels(mat_src, goal_point_cv, num_pixels_to_check);
    if (abs(count_theta - count_theta_pi) >= 4) { // Difference is big enough.
        if (count_theta_pi > count_theta) { // Use orientation with more black / unknown pixels.
            expl_point_orientation = expl_point_orientation + M_PI;
        }

        // Draw line closest to the exploration point.
        if (shortest_dist <= 2.0) {
            ROS_DEBUG("Found edge lies close enough to the exploration point (%4.2f pixels)", shortest_dist);
            if (show_debug) {
                line(mat_canny_bgr, lowest_dist_pt1, lowest_dist_pt2, cv::Scalar(0, 0, 200), 1, 8/*CV_AA*/);
            }
            orientation = expl_point_orientation;
            ret = true;
        }

        if (show_debug) {
            // Draw cross goal point.
            cv::Point pt_e1, pt_e2;
            pt_e1.x = goal_point_cv.x - 2;
            pt_e1.y = goal_point_cv.y;
            pt_e2.x = goal_point_cv.x + 2;
            pt_e2.y = goal_point_cv.y;
            line(mat_canny_bgr, pt_e1, pt_e2, cv::Scalar(0, 128, 0), 1, 8/*CV_AA*/);
            pt_e1.x = goal_point_cv.x;
            pt_e1.y = goal_point_cv.y - 2;
            pt_e2.x = goal_point_cv.x;
            pt_e2.y = goal_point_cv.y + 2;
            line(mat_canny_bgr, pt_e1, pt_e2, cv::Scalar(0, 128, 0), 1, 8/*CV_AA*/);

            // Display windows.
            cv::Size size(300, 300);//the dst image size,e.g.100x100
            cv::Mat mat_src_resize, mat_canny_resize;
            resize(mat_src, mat_src_resize, size);
            resize(mat_canny_bgr, mat_canny_resize, size);


            imshow("source", mat_src_resize);
            imshow("detected lines", mat_canny_resize);

            cv::waitKey();
        }
    }
    return ret;
}

std::vector<Pose> Planner::getCheapest(std::vector<Pose> &pts, Pose &roboPose,
                     bool calculate_worst_driveability, double robot_length_x, double robot_width_y) {

    if (calculate_worst_driveability) {
        assert(robot_length_x > 0 && robot_width_y > 0);
    }

    bool visualize_debug_infos = true;
    std::vector<Pose> goals;
    goals.reserve(pts.size());

    // The for-loop is used for calculating the angular differences
    // experimental: dividing the number of cells that will be explored 
    // at the given point by the angDifference
    int too_close_counter = 0;
    int no_new_cell_counter = 0;
    int touch_obstacle = 0;
    int outside_of_the_map = 0;
    //int touch_difficult_region = 0;
    bool robot_pos_grid_available = false;

    if (roboPose.x > 0 && roboPose.x < map_width_ &&
            roboPose.y > 0 && roboPose.y < map_height_) {

        robot_pos_grid_available = true;
    } else {
        ROS_WARN("Robot position (%4.2f, %4.2f) lies outside of the map", roboPose.x, roboPose.y);
    }

    double max_robot_goal_dist = 0;
    double max_explored_cells = 0;
    std::list<ExplorationPoint> expl_point_list;
    for (std::vector<Pose>::const_iterator i = pts.begin(); i != pts.end(); ++i) {
        Pose givenPoint;
        if ((*i).x > 0 && (*i).x < map_width_ &&
            (*i).y > 0 && (*i).y < map_height_) {
            givenPoint.x = (*i).x;
            givenPoint.y = (*i).y;
        } else { // Should not happen.
            outside_of_the_map++;
            continue;
        }

        // If the distance between the robot and the exploration point exceeds the sensor range
        // (simply a far-away-goal) we use the edge detection to calculate the orientation
        // of the goal pose, otherwise the orientation of the vector between robot and goal is used.
        bool use_calculate_goal_orientation = false;
        double dist_robo_goal_grid = 0.0;
        if (robot_pos_grid_available && mMaxDistantSensorPoint > 0) {
            dist_robo_goal_grid = sqrt(
                    pow((int) givenPoint.x - (int) roboPose.x, 2) + pow((int) givenPoint.y - (int) roboPose.y, 2));
            if (dist_robo_goal_grid > mMaxDistantSensorPoint) {
                use_calculate_goal_orientation = true;
            }
        }

        // Calculate angle of exploregoal-vector and map it to 0-2*pi radian.
        double rotationOfPoint = 0.0;

        // If the exploration point lies close to the robot or if no matching edge 
        // can be found the old orientation calculation is used.
        bool edge_found = false;
        if (!(use_calculate_goal_orientation /*&&
              calculateGoalOrientation(givenPoint, rotationOfPoint, visualize_debug_infos)*/)) {
            rotationOfPoint = atan2(i->y - roboPose.y, i->x - roboPose.x);
            ROS_DEBUG("Goal orientation has been calculated using the current robot position");
        } else {
            ROS_DEBUG("Goal orientation has been calculated using edge detection");
            edge_found = true;
            rotationOfPoint = i->theta; //[o,2pi]
        }
        rotationOfPoint = map0to2pi(rotationOfPoint);
        givenPoint.theta = rotationOfPoint;

        // Calculate angular distance, will be [0,2*PI)
        double yaw = map0to2pi(roboPose.theta);
        double angularDistance = std::fabs(yaw - rotationOfPoint);
        // Robot will use shortest turning distance!
        if (angularDistance > M_PI) {
            angularDistance = 2 * M_PI - angularDistance;
        }

        // Calculate the distance between the robot and the goalPose. 
        double robotToPointDistance = (Eigen::Vector3d(givenPoint.x, givenPoint.y, 0) - Eigen::Vector3d(roboPose.x, roboPose.y, 0)).norm();
        if (robotToPointDistance > max_robot_goal_dist) {
            max_robot_goal_dist = robotToPointDistance;
        }
        // Goal poses which are too close to the robot are discarded.
        if (robotToPointDistance < mConfig.min_goal_distance || robotToPointDistance == 0) {
            too_close_counter++;
            continue;
        }


        // Ignore ecploration point if it is no real exploration point.
        unsigned numberOfExploredCells = willBeExplored(givenPoint).size();
        if (numberOfExploredCells > max_explored_cells) {
            max_explored_cells = numberOfExploredCells;
        }
        if (numberOfExploredCells <= 0) {
            no_new_cell_counter++;
            continue;
        }

        // Requests the worst driveability at the goal pose using the width/length of the robot.
        // If the goal pose rectangle touches an obstacle the goal point is ignored.
        double worst_driveability = 1.0;
        if (calculate_worst_driveability) {
            // todo judge vehicle body collision checking
            //robot_length_x,robot_width_y
            if(!isFree((*i))) {
                worst_driveability = 0;
                touch_obstacle++;
                continue;
            }
        }

        // Create exploration point and add it to a list to be sorted as soon
        // as max_robot_goal_dist and max_explored_cells are known.
        ExplorationPoint expl_point(givenPoint, numberOfExploredCells, angularDistance, robotToPointDistance,
                                    worst_driveability, !use_calculate_goal_orientation ||
                                                        edge_found); // See documentation in ExplorationPoint.edgeFound.
        expl_point_list.push_back(expl_point);
    }

    ROS_INFO("%d exploration points are uses: %d touches an obstacle, %d are too close to the robot, %d leads to no new cells, %d lies outside of the map",
            pts.size(), touch_obstacle, too_close_counter, no_new_cell_counter,
            outside_of_the_map);

    std::list<ExplorationPoint>::iterator it = expl_point_list.begin();
    double max_value = 0;
    double min_value = std::numeric_limits<double>::max();
    double value = 0;
    for (; it != expl_point_list.end(); it++) {
        value = it->calculateOverallValue(mConfig.weights, max_explored_cells, max_robot_goal_dist);
        if (value > max_value)
            max_value = value;
        if (value < min_value)
            min_value = value;
    }
    // Higher values are better, so the optimal goal is the last one.
    expl_point_list.sort();

    // The list is processed from back to front, so the first element in the goal vector is the best one.
    //TODO ADD Marker showing
    Pose expl_rbs;
    ROS_DEBUG("Exploration point list:\n");
    for (auto rit = expl_point_list.crbegin(); rit != expl_point_list.crend(); ++rit) {
        expl_rbs = rit->explPose;
        goals.push_back(expl_rbs);
        ROS_DEBUG(
                "Point(%4.2f, %4.2f, %4.2f) values: overall(%4.2f), expl(%4.2f), ang(%4.2f), dist(%4.2f), driveability(%4.2f), edge(%4.2f)\n",
                rit->explPose.x, rit->explPose.y, rit->explPose.theta, rit->overallValue,
                rit->expl_value, rit->ang_value, rit->dist_value, rit->driveability_value, rit->edge_value);
    }

    if (goals.empty()) {
        ROS_INFO( "did not find any target, propably stuck in an obstacle.");
    }

    return goals;
}

FrontierList Planner::getCoverageFrontiers(Pose start) {
    GridPoint startPoint;
    startPoint.x = start.x;
    startPoint.y = start.y;
    return getFrontiers(mCoverageMap, startPoint);
}

bool Planner::isFree(Pose pos) {

    unsigned int xCenter;
    unsigned int yCenter;

    xCenter = pos.x;
    yCenter = pos.y;
    int radius = ceil(mConfig.vehicle_length / 2 / map_solution_);

    int vx[2], vy[2];
    for(int x = xCenter - radius; x <= xCenter; ++x)
        for(int y = yCenter - radius; y <= yCenter; ++y)
            if((x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) <= radius*radius)
            {
                vx[0] = x;
                vy[0] = y;
                vx[1] = xCenter - (x - xCenter);
                vy[1] = yCenter - (y - yCenter);

                char value;
                for(int i=0; i<2; ++i)
                    for(int j=0; j<2; ++j)
                    {
                        GridPoint point(vx[i], vy[j], 0);
                        if(mCoverageMap->getData(point, value) && value == OBSTACLE) {
                            return false;
                        }
                    }
            }
    return true;

    /*
    // Define the robot as rectangle
    static double left   = -1.0 * mConfig.base2back;
    static double right  = mConfig.vehicle_length - mConfig.base2back;
    static double top    = mConfig.vehicle_width / 2.0;
    static double bottom = -1.0 * mConfig.vehicle_width / 2.0;
    static double resolution = map_solution_;

    // Coordinate of base_link in OccupancyGrid frame
    double base_x     = pos.x * resolution;
    double base_y     = pos.y * resolution;
    double base_theta = map0to2pi(pos.theta);

    // Calculate cos and sin in advance
    double cos_theta = std::cos(base_theta);
    double sin_theta = std::sin(base_theta);

    for (double x = left; x < right; x += resolution) {
        for (double y = top; y > bottom; y -= resolution) {
            // 2D point rotation
            // todo not compact, exisit single cell unprocessed
            int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
            int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

            GridPoint point(index_x, index_y, 0);
            mCoverageMap->setData(point, VISIBLE);
        }
    }*/

}

const exploration::GridMap &Planner::getCoverageMap() const {
    return *mCoverageMap;
}


// PRIVATE
double Planner::map0to2pi(double angle_rad) {
    double pi2 = 2 * M_PI;
    while (angle_rad >= pi2) {
        angle_rad -= pi2;
    }
    while (angle_rad < 0) {
        angle_rad += pi2;
    }
    return angle_rad;
}

bool Planner::included(FloatPoint point, std::vector<Polygon> &regions) {
    for (int i = 0; i < regions.size(); i++) {
        for (int j = 0; j < regions[i].size(); j++) {
            if (regions[i][j].x == point.x && regions[i][j].y == point.y) {
                return true;
            }
        }
    }
    return false;
}



int Planner::countBlackPixels(cv::Mat mat, struct Pose pose, int vec_len_px) {
    Eigen::Vector2d start(pose.x, pose.y);
    double orientation = pose.theta;
    int count_black = 0;

    for (int i = 1; i <= vec_len_px; i++) {
        Eigen::Vector2d vec(i, 0.0);

        Eigen::Rotation2D<double> rot2(orientation);
        vec = rot2 * vec;
        vec += start;
        if (vec[0] < 0 || vec[0] >= mat.cols || vec[1] < 0 || vec[1] >= mat.rows) {
            return count_black;
        }
        if (mat.at<uchar>(vec[1], vec[0]) == 0) {
            count_black++;
        }
    }
    return count_black;
}


void Planner::updateCycle(const nav_msgs::OccupancyGridConstPtr &map) {
    auto start = std::chrono::system_clock::now();
    // After a map has been received
    if(initialized_) {
        auto start = std::chrono::system_clock::now();
        updateMap();
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        ROS_INFO("updatd sensor area msec: %lf", msec);
        // Generate goal will only produce goals if the operation
        generateGoals();
    }

    GridPoint point;
    if(!initialized_) {
        map_solution_ = map->info.resolution;
        map_origin_.x = map->info.origin.position.x;
        map_origin_.y = map->info.origin.position.y;
        map_origin_.theta = tf::getYaw(map->info.origin.orientation);
        map_width_ = map->info.width;
        map_height_ = map->info.height;
        GridMap map_temp(map_width_, map_height_);
        for(size_t y = 0; y < map_height_; y++) {
            point.y = y;
            for(size_t x = 0; x < map_width_; x++) {
                char value = UNKNOWN;
                point.x = x;
                unsigned int index = y * map_height_ + x;
                // todo set threshold = 80
                if(map->data[index] > 80) {
                    value = OBSTACLE;
                }
                map_temp.setData(point, value);
            }
        }
        // create coverage map
        initCoverageMap(&map_temp);

        // Initialize saved positions.
        tf::StampedTransform transform;
        transform = pose_hander_.lookupPose("/odom", "/base_link");
        xinit_ = transform.getOrigin().x();
        yinit_ = transform.getOrigin().y();

        // add initialized faked camera polygon(triangle)
        const double polygon_height = 10;
        const double polygon_base_length = 8.4;
        const double base2camera_length = 3.8;
        Polygon poly;
        FloatPoint first_pt, second_pt, third_pt, fourth_pt;  // point is in ogm
        // front camera
        first_pt.x =  int(base2camera_length / map_solution_);
        first_pt.y = 0;
        second_pt.x = first_pt.x + polygon_height / map_solution_;
        second_pt.y = first_pt.y + polygon_base_length / 2 / map_solution_;
        third_pt.x = second_pt.x;
        third_pt.y = first_pt.y - polygon_base_length / 2 / map_solution_;
        poly.clear();
        poly.push_back(first_pt);
        poly.push_back(second_pt);
        poly.push_back(third_pt);
        addSensor(poly);

        // left camera
        first_pt.x =  int(base2camera_length / map_solution_);
        first_pt.y = mConfig.vehicle_width / 2 / map_solution_;
        second_pt.x = first_pt.x - polygon_base_length / 2 / map_solution_;
        second_pt.y = first_pt.y + polygon_height / map_solution_;
        third_pt.x = first_pt.x + polygon_base_length / 2 / map_solution_;
        third_pt.y = second_pt.y;
        poly.clear();
        poly.push_back(first_pt);
        poly.push_back(second_pt);
        poly.push_back(third_pt);
        addSensor(poly);

        // right camera
        first_pt.x =  int(base2camera_length / map_solution_);
        first_pt.y = -mConfig.vehicle_width / 2 / map_solution_;
        second_pt.x = first_pt.x - polygon_base_length / 2 / map_solution_;
        second_pt.y = first_pt.y - polygon_height / map_solution_;
        third_pt.x = first_pt.x + polygon_base_length / 2 / map_solution_;
        third_pt.y = second_pt.y;
        poly.clear();
        poly.push_back(first_pt);
        poly.push_back(second_pt);
        poly.push_back(third_pt);
        addSensor(poly);

        // vehicle body
        first_pt.x =  int(-mConfig.base2back/ map_solution_);
        first_pt.y = mConfig.vehicle_width / 2 / map_solution_;
        second_pt.x = (mConfig.vehicle_length - mConfig.base2back)/ map_solution_;
        second_pt.y = first_pt.y;
        third_pt.x = second_pt.x;
        third_pt.y = -mConfig.vehicle_width / 2 / map_solution_;
        fourth_pt.x = first_pt.x;
        fourth_pt.y = -mConfig.vehicle_width / 2 / map_solution_;
        poly.clear();
        poly.push_back(first_pt);
        poly.push_back(second_pt);
        poly.push_back(third_pt);
        poly.push_back(fourth_pt);
        addSensor(poly);

        ROS_DEBUG("Planner initialization complete");
        initialized_ = true;
    }
    //todo consider situation where map size change
    //update coverage map use traverse map
    {
        auto start = std::chrono::system_clock::now();
        bool isObstacle;
        PointList obstacles;
        PointList obstacleToUnknown;
        char value = 0;
        const GridMap &c_map = getCoverageMap();
        unsigned int width = map->info.width;
        unsigned int height = map->info.height;
        for(size_t y = 0; y < height; y++) {
            point.y = y;
            for(size_t x = 0; x < width; x++) {
                point.x = x;
                isObstacle = false;
                unsigned int index = y * height + x;
                if(map->data[index] > 80) {
                    obstacles.push_back(point);
                    isObstacle = true;
                }

                if(!isObstacle && c_map.getData(point, value) && value == OBSTACLE) {
                    obstacleToUnknown.push_back(point);
                }
            }
        }
        setCoverageMap(obstacles, OBSTACLE);
        setCoverageMap(obstacleToUnknown, UNKNOWN);
        ROS_DEBUG("Obstalces have been updated!");
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//        ROS_INFO("Obstalces update msec: %lf", msec);
    }

    // publish cover map
    {
        nav_msgs::OccupancyGrid occ_map;
        unsigned map_width = map->info.width;
        unsigned map_height = map->info.height;
        occ_map.header.frame_id = map->header.frame_id;
        occ_map.info = map->info;
        occ_map.data.assign(map_width * map_height, -1);

        for (size_t y = 0; y < map_height; y++) {
            point.y = y;
            for (size_t x = 0; x < map_width; x++) {
                char value = UNKNOWN;
                point.x = x;
                unsigned int index = y * map_height + x;
                if (mCoverageMap->getData(point, value) && value == OBSTACLE) {
                    occ_map.data[index] = 100;
                } else if (mCoverageMap->getData(point, value) && value == UNKNOWN) {
                    occ_map.data[index] = -1;
                } else if (mCoverageMap->getData(point, value) && value == VISIBLE) {
                    occ_map.data[index] = 0;
                } else;;
            }
        }
        cover_map_pub_.publish(occ_map);
    }

    auto end = std::chrono::system_clock::now();
    auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    ROS_INFO("update cycle msec: %lf", msec);

}

void Planner::updateMap() {
    ROS_DEBUG("Updating map");

    // Initialize saved positions.
    tf::StampedTransform transform;
    transform = pose_hander_.lookupPose("/odom", "/base_link");
    current_pose_.x = transform.getOrigin().x() - xinit_;
    current_pose_.y = transform.getOrigin().y() - yinit_;
    current_pose_.theta = tf::getYaw(transform.getRotation()); //[-pi, pi]

    // from odom relative pose to ogm
    Pose current_ogm_pose;
    current_ogm_pose.x = (current_pose_.x - map_origin_.x) / map_solution_;
    current_ogm_pose.y = (current_pose_.y - map_origin_.y) / map_solution_;
    current_ogm_pose.theta = tf::getYaw(transform.getRotation());;

    current_pose_ = current_ogm_pose;

    // mark unknown area(not include visible) within faked camera polygon  with VISIBLE
    addReading(current_ogm_pose);
}

void Planner::generateGoals() {
    ROS_DEBUG("generateGoals");
    auto start = std::chrono::system_clock::now();
    FrontierList goals_tmp = getCoverageFrontiers(current_pose_);
    auto end = std::chrono::system_clock::now();
    auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    ROS_INFO("search frontier cost msec: %lf", msec);
    for(unsigned int i=0; i<goals_tmp.size(); i++) {
        // todo filter small frontier (single cell like)
        if(goals_tmp[i].size() < 25) {
            goals_tmp.erase(goals_tmp.begin() + i);
            i--;
        }
    }

    for(unsigned int i=0; i<goals_tmp.size(); i++) {
        ROS_INFO("(%d)Generates %d frontier goals", i, goals_tmp[i].size());
    }

#ifdef DEBUG
    std::vector<Pose> frontier_array_centroid;
    std::vector<int> type_array;
    int type = 0, max = 0;
    BOOST_FOREACH(PointList frontier, static_cast<std::vector<PointList> &&>(goals_tmp))  {
                    BOOST_FOREACH(GridPoint point, frontier) {
                                    Pose pos;
                                    pos.x = point.x;
                                    pos.y = point.y;
                                    frontier_array_centroid.push_back(pos);
                                    type_array.push_back(type);
                                }
                    type++;
                }

    publishMarkerArray(frontier_array_centroid, type_array);

#endif

    std::vector<Pose> goals, final_goals;
    Pose vec;
    for(FrontierList::const_iterator frIt = goals_tmp.begin(); frIt != goals_tmp.end(); ++frIt) {
        for(PointList::const_iterator pointIt = frIt->begin(); pointIt != frIt->end(); ++pointIt) {
            vec = Pose(pointIt->x, pointIt->y, 0);
            goals.push_back(vec);
        }
    }

#ifdef CV_Dector
    std::vector<Pose> goals_cv;
    std::vector<int> type_array;
    // Copy image from the exploration map to the opencv image.
    // TODO differentiate circle and line contour use opencv
    // use bfs-dector results, only use cv extract line
    if(0) {
        cv::Mat mat_src(map_height_, map_width_, CV_8UC1, cv::Scalar(0));
        for(std::vector<Pose>::const_iterator it = goals.begin(); it != goals.end(); ++it) {
            int x_cv = (int)it->x;
            int y_cv = (int)it->y;
            mat_src.at<uchar>(y_cv, x_cv) = 255;
        }

        cv::Mat  mat_canny_bgr;
        cv::RNG rng( 0xFFFFFFFF );
        Color col;
        cvtColor(mat_src, mat_canny_bgr, CV_GRAY2BGR);
        std::vector<cv::Vec4f> lines;
        // Resolution: 1 px and 180/32 degree. last two para is important!
        HoughLinesP(mat_src, lines, 1, CV_PI / 32, 5, 20, 10);
        Pose vec;
        for( size_t i = 0; i < lines.size(); i++ ) {
            cv::Vec4i l = lines[i];
            line( mat_canny_bgr, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), col.randomColor(rng), 1, 8/*CV_AA*/);
            vec.x = (l[0] + l[2]) / 2;
            vec.y = (l[1] + l[3]) / 2;
            vec.theta = std::atan2(l[3] - l[1], l[2] - l[0]);

            goals_cv.push_back(vec);
            int type = i;
            type_array.push_back(type);
        }




        ROS_INFO("LINE number : %d", lines.size());

        cv::imshow("source", mat_src);
        cv::imshow("detected lines", mat_canny_bgr);

        cv::waitKey(1);
    }
    if(1) {
        cv::Mat mat_src(map_height_, map_width_, CV_8UC1, cv::Scalar(0));
        char *coverage_map = mCoverageMap->getData();
        int c = 0;
        int x_cv = 0;
        for (int x = 0; x < map_width_; ++x, ++x_cv) {
            int y_cv = 0;
            for (int y = 0; y < map_height_; ++y, ++y_cv) {
                c = (int) coverage_map[y * mCoverageMap->getWidth() + x];
                if (c == VISIBLE) {
                    mat_src.at<uchar>(y_cv, x_cv) = 255;
                } else { // OBSTACLE or UNKNOWN
                    mat_src.at<uchar>(y_cv, x_cv) = 0;
                }
            }
        }
        cv::Mat mat_canny, mat_canny_bgr;
        cv::RNG rng( 0xFFFFFFFF );
        Color col;
        Canny(mat_src, mat_canny, 50, 200);
        cvtColor(mat_canny, mat_canny_bgr, CV_GRAY2BGR);
        std::vector<cv::Vec4f> lines;
        // Resolution: 1 px and 180/32 degree. last two para is important!
        HoughLinesP(mat_canny, lines, 1, CV_PI / 32, 10, 20, 5);
        Pose vec;
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            line( mat_canny_bgr, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), col.randomColor(rng), 1, 8/*CV_AA*/);
            vec.x = (l[0] + l[2]) / 2;
            vec.y = (l[1] + l[3]) / 2;
            vec.theta = std::atan2(l[3] - l[1], l[2] - l[0]);
            {
                int num_pixels_to_check = 10;
                double expl_point_orientation = vec.theta;
                int count_theta = countBlackPixels(mat_src, vec, num_pixels_to_check);
                vec.theta = expl_point_orientation + M_PI;
                int count_theta_pi = countBlackPixels(mat_src, vec, num_pixels_to_check);
                if (abs(count_theta - count_theta_pi) >= 4) { // Difference is big enough.
                    if (count_theta_pi > count_theta) { // Use orientation with more black / unknown pixels.
                        expl_point_orientation = expl_point_orientation + M_PI;
                    }
                }
                vec.theta = expl_point_orientation;  //[0, 2pi]
            }

            goals_cv.push_back(vec);
            int type = i;
            type_array.push_back(type);
        }

        ROS_INFO("LINE number : %d", lines.size());
        cv::imshow("source", mat_src);
        cv::imshow("detected lines", mat_canny_bgr);

        cv::waitKey(1);
    }



#endif

//    publishMarkerArray(goals_cv, type_array );
    auto start0 = std::chrono::system_clock::now();
    final_goals = getCheapest(goals_cv, current_pose_, true, mConfig.vehicle_length, mConfig.vehicle_width);
    ROS_INFO("Got %d final goals", final_goals.size());
    auto end0 = std::chrono::system_clock::now();
    auto msec0 = std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count() / 1000.0;
    ROS_INFO("find cheapest frontier cost msec: %lf", msec0);

    if(type_array.size() > 0) {
        type_array[0] = -2;
    }
    publishMarkerArray(final_goals, type_array );


    if(final_goals.size() == 0) {
        ROS_INFO("No new goals found, state is set to EXPLORATION_DONE");
        printf("No new goals found, state is set to EXPLORATION_DONE\n");
//        state(EXPLORATION_DONE);
    } else {
        printf("State back to running");
//        state(RUNNING);
    }

}

void Planner::publishMarkerArray(const std::vector<Pose> &array, std::vector<int> type) {
    visualization_msgs::MarkerArray markersMsg;
    for (int i = 0; i < array.size(); i++) {
        double x, y;
        x = array[i].x * map_solution_ + map_origin_.x;
        y = array[i].y * map_solution_ + map_origin_.y;
        PoseWrap pose(x, y);

        VisMarker marker;
        /*
          type:
            1: free area
            2: frontier
        */
        type[i] = type[i] % 4;
        if (type[i] == 0) {
            Color color(0, 0, 0.7);
            marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
        } else if (type[i] == 1) {
            Color color(0, 0.7, 0);
            marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
        } else if (type[i] == 2) {
            Color color(0.7, 0, 0);
            marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
        } else if (type[i] == 3) {
            Color color(0, 1.0, 0);
            marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
        } else if (type[i] == -1) {
            Color color(0, 1.0, 0);
            marker.setParams("frontier", pose.getPose(), 0.15, color.getColor(), i, 0.6);
        } else if (type[i] == -2) {
            Color color(0, 1.0, 0);
            marker.setParams("frontier", pose.getPose(), 0.75, color.getColor(), i);
        } else {
            ROS_WARN("TYPE INDEX ERROR!");
        }

        markersMsg.markers.push_back(marker.getMarker());
    }
    mMarkerPub_.publish(markersMsg);
}


