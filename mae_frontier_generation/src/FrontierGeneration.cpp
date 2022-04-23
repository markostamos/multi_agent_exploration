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

void FrontierGeneration::getFrontiers(std::vector<geometry_msgs::Point> *frontiers)
{

    frontiers->reserve(map_.info.height * map_.info.width);
    for (int i = 0; i < map_.info.width; i++)
    {
        for (int j = 0; j < map_.info.height; j++)
        {
            if (map_.data[i + j * map_.info.width] == 0 && FrontierGeneration::isFrontier(i, j))
            {
                frontiers->push_back(pointFrom2DMapIndex(i, j, map_));
                // ROS_WARN_STREAM(frontiers->size());
            }
        }
    }
}
