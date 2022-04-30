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
    nav_msgs::OccupancyGrid costmap_;

    int obj_threshold_;

public:
    FrontierGeneration() : map_(nav_msgs::OccupancyGrid())
    {
        obj_threshold_ = 20;
    }

    void getFrontiers(std::vector<geometry_msgs::Point> *frontiers);

    void updateMap(const nav_msgs::OccupancyGrid &map);
    void updateCostMap(const nav_msgs::OccupancyGrid &costmap);

private:
    bool isFrontier(int i, int j);
    bool isNearObstacle(int i, int j);
};
#endif // FRONTIER_GENERATION_H
