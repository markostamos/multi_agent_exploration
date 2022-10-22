#ifndef FRONTIER_GENERATION_H
#define FRONTIER_GENERATION_H
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <geometry_msgs/PointStamped.h>
class FrontierGeneration
{
private:
    nav_msgs::OccupancyGrid map_;
    octomap::OcTree *octree_;
    std::vector<geometry_msgs::Point> blacklisted_pts_;
    geometry_msgs::Point exploration_center_;

    std::vector<geometry_msgs::Point> location_;
    int exploration_radius_;

public:
    FrontierGeneration() : map_(nav_msgs::OccupancyGrid()),
                           exploration_radius_(100),
                           exploration_center_(geometry_msgs::Point()),
                           octree_(nullptr)

    {
    }

    /**
     * @brief Find frontiers in the current 2d OccupancyGrid map.
     *
     * @param frontiers Pointer to the vector of frontiers to be filled.
     * @param obstacle_threshold Distance from obstacle that is considered an obstacle.
     */
    void getFrontiers(std::vector<geometry_msgs::Point> *frontiers, float obstacle_threshold);

    /**
     * @brief Find frontiers in the current 3d Octomap.
     *
     * @param frontiers Pointer to the vector of frontiers to be filled.
     */
    void get3DFrontiers(std::vector<geometry_msgs::Point> *frontiers);

    /**
     * @brief Filters a vector of frontiers using the DBSCAN algorithm. Only the cluster centers are kept
     *
     * @param frontiers Vector of frontiers to be filtered.
     * @param min_pts Minimum number of points in a cluster.
     * @param epsilon Maximum distance between points in a cluster.
     */
    void filterFrontiersDBSCAN(std::vector<geometry_msgs::Point> *frontiers, int min_points, float epsilon);

    /**
     * @brief Updates the current 2d Occupancy grid map that the frontier generation algorithm is working on.
     *
     * @param map
     */
    void updateMap(const nav_msgs::OccupancyGrid &map);

    /**
     * @brief Updates the current 3d Octomap that the frontier generation algorithm is working on.
     *
     * @param octomap
     */
    void updateOctomap(const octomap_msgs::Octomap octomap);

    /**
     * @brief Updates the current location of the agents. Location is used for the bounding box calculating 3d frontiers.
     *
     * @param location
     * @param index
     */
    void updateLocation(const geometry_msgs::PointStamped &location, int index);

    /**
     * @brief Set the 2d Exploration Area object. Frontiers outside of this area are ignored.
     *
     * @param center
     * @param radius
     */
    void setExplorationArea(const geometry_msgs::Point &center, const int radius);

    /**
     * @brief Get the current exploration area.
     *
     * @param exploration_center
     * @param radius
     */
    void getExplorationArea(geometry_msgs::Point *exploration_center, int *radius);

    /**
     * @brief Adds a point to the blacklist, ignoring it and its neighbourhood in further frontier generation attempts.
     *
     * @param pt
     */
    void addToBlacklist(const geometry_msgs::Point &pt);

private:
    /**
     * @brief Checks if a point is considered  a frontier in the 2D occupancy grid map
     *
     * @param i index of the point in the map
     * @param j index of the point in the map
     */
    inline bool isFrontier(int i, int j) const;

    /**
     * @brief Checks if a point is near an obstacle in the 2D occupancy grid map
     *
     * @param i index of the point in the map
     * @param j index of the point in the map
     * @param threshold distance from obstacle that is also considered an obstacle (in meters).
     */
    bool isNearObstacle(int i, int j, float threshold) const;

    /**
     * @brief Checks if a point is inside the exploration area.
     *
     * @param i index of the point in the map
     * @param j index of the point in the map
     */
    bool isInExplorationArea(const int i, const int j) const;

    /**
     * @brief Checks if a point is in the blacklist.
     *
     * @param pt
     */
    bool isBlacklisted(const geometry_msgs::Point &pt) const;

    /**
     * @brief Get the Neighbors of a point in the list of points provided.
     *
     * @param frontiers List of points to search in.
     * @param i Index of the point to search for its neighbors.
     * @param epsilon Maximum distance between points to be considered neighbors.
     * @return std::vector<int> List of indices of the neighbors.
     */
    std::vector<int> getNeighbors(const std::vector<geometry_msgs::Point> &frontiers, int i, float epsilon);
};
#endif // FRONTIER_GENERATION_H
