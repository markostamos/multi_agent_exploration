#ifndef FRONTIER_GENERATION_H
#define FRONTIER_GENERATION_H
#include <mae_frontier_generation/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
/* TODO: ADD documentation */
class FrontierGeneration
{
private:
    nav_msgs::OccupancyGrid map_;

public:
    FrontierGeneration() : map_(nav_msgs::OccupancyGrid())
    {
    }

    void getFrontiers(std::vector<geometry_msgs::Point> *frontiers);
    /* TODO: Check whether map has actually changed (speed increase) */
    void updateMap(const nav_msgs::OccupancyGrid &map);

private:
    bool isFrontier(int i, int j);
};
#endif // FRONTIER_GENERATION_H
