#include <mae_frontier_generation/FrontierGeneration.h>
#include <ros/ros.h>
bool FrontierGeneration::isFrontier(int i, int j)
{

    if ((i > 0 && map_.data[i - 1 + j * map_.info.width] == -1) ||
        (i < map_.info.width - 1 && map_.data[i + 1 + j * map_.info.width] == -1) ||
        (j > 0 && map_.data[i + (j - 1) * map_.info.width] == -1) ||
        (j < map_.info.height - 1 && map_.data[i + (j + 1) * map_.info.width] == -1))
    {
        return true;
    }
    return false;
}

void FrontierGeneration::updateMap(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;
}
void FrontierGeneration::updateCostMap(const nav_msgs::OccupancyGrid &costmap)
{
    costmap_ = costmap;
}

void FrontierGeneration::getFrontiers(std::vector<geometry_msgs::Point> *frontiers)
{
    if (map_.info.width == 0)
        return;
    frontiers->reserve(map_.info.height * map_.info.width);
    for (int i = 0; i < map_.info.width; i++)
    {
        for (int j = 0; j < map_.info.height; j++)
        {
            if (map_.data[i + j * map_.info.width] == 0 && FrontierGeneration::isFrontier(i, j) && !FrontierGeneration::isNearObstacle(i, j))
            {
                frontiers->push_back(pointFrom2DMapIndex(i, j, map_));
            }
        }
    }
    // ROS_WARN_STREAM(frontiers->size());
}

bool FrontierGeneration::isNearObstacle(int i, int j)
{
    int n = 5;
    for (int x = i - n; x <= i + n; x++)
    {
        for (int y = j - n; y <= j + n; y++)
        {
            if (x >= 0 && x < map_.info.width && y >= 0 && y < map_.info.height)
            {
                if (map_.data[x + y * map_.info.width] > 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}