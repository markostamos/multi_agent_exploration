#ifndef FRONTIER_GENERATION_H
#define FRONTIER_GENERATION_H
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
/* TODO: ADD documentation */

class FrontierGeneration
{
private:
    nav_msgs::OccupancyGrid map_;
    std::vector<geometry_msgs::Point> blacklisted_pts_;
    geometry_msgs::Point exploration_center_;
    int exploration_radius_;

public:
    FrontierGeneration() : map_(nav_msgs::OccupancyGrid()),
                           exploration_radius_(100),
                           exploration_center_(geometry_msgs::Point())

    {
    }

    void getFrontiers(std::vector<geometry_msgs::Point> *frontiers, float obstacle_threshold);
    void filterFrontiersDBSCAN(std::vector<geometry_msgs::Point> *frontiers, int min_points, float epsilon);
    void updateMap(const nav_msgs::OccupancyGrid &map);
    void setExplorationArea(const geometry_msgs::Point &center, const int radius);
    void getExplorationArea(geometry_msgs::Point *exploration_center, int *radius);
    void addToBlacklist(const geometry_msgs::Point &pt);

private:
    inline bool isFrontier(int i, int j) const;
    bool isNearObstacle(int i, int j, float threshold) const;
    bool isInExplorationArea(const int i, const int j) const;
    bool isBlacklisted(const geometry_msgs::Point &pt) const;

    std::vector<int> getNeighbors(const std::vector<geometry_msgs::Point> &frontiers, int i, float epsilon);
};
#endif // FRONTIER_GENERATION_H
