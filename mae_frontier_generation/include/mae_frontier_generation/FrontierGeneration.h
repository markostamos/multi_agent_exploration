#ifndef FRONTIER_GENERATION_H
#define FRONTIER_GENERATION_H
#include <mae_frontier_generation/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <mae_utils/utils.h>
/* TODO: ADD documentation */

class FrontierGeneration
{
private:
    nav_msgs::OccupancyGrid map_;

public:
    FrontierGeneration() : map_(nav_msgs::OccupancyGrid())

    {
    }

    void getFrontiers(std::vector<geometry_msgs::Point> *frontiers, float obstacle_threshold);
    void filterFrontiersDBSCAN(std::vector<geometry_msgs::Point> *frontiers, int min_points, float epsilon);
    void updateMap(const nav_msgs::OccupancyGrid &map);

private:
    bool isFrontier(int i, int j);
    bool isNearObstacle(int i, int j, float threshold);

    std::vector<int> getNeighbors(const std::vector<geometry_msgs::Point> &frontiers, int i, float epsilon);
};
#endif // FRONTIER_GENERATION_H
