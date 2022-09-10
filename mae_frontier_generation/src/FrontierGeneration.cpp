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

void FrontierGeneration::getFrontiers(std::vector<geometry_msgs::Point> *frontiers, float threshold)
{
    if (map_.info.width == 0)
        return;
    frontiers->reserve(map_.info.height * map_.info.width);
    for (int i = 0; i < map_.info.width; i++)
    {
        for (int j = 0; j < map_.info.height; j++)
        {
            if (map_.data[i + j * map_.info.width] == 0 &&
                isFrontier(i, j) &&
                !isBlacklisted(i, j) &&
                isInExplorationArea(i, j) &&
                !isNearObstacle(i, j, threshold))
            {
                frontiers->push_back(pointFrom2DMapIndex(i, j, map_));
            }
        }
    }
}

bool FrontierGeneration::isNearObstacle(int i, int j, float threshold)
{
    // Objects are considered obstacles if they are within a certain distance of the robot
    // to avoid collisions
    int n = (int)(threshold / map_.info.resolution) + 1;

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

void FrontierGeneration::filterFrontiersDBSCAN(std::vector<geometry_msgs::Point> *frontiers, int min_points, float epsilon)
{

    std::vector<int> labels(frontiers->size(), -1);
    std::vector<std::vector<int>> clusters;
    int cluster_id = -1;
    for (int i = 0; i < frontiers->size(); i++)
    {
        if (labels[i] != -1)
            continue;
        std::vector<int> neighbors = getNeighbors(*frontiers, i, epsilon);
        if (neighbors.size() < min_points)
        {
            labels[i] = -2;
            continue;
        }
        cluster_id++;
        clusters.push_back({});
        labels[i] = cluster_id;
        clusters[cluster_id].push_back(i);
        for (int j = 0; j < neighbors.size(); j++)
        {
            if (labels[neighbors[j]] == -2)
            {
                labels[neighbors[j]] = cluster_id;
                clusters[cluster_id].push_back(neighbors[j]);
            }
            if (labels[neighbors[j]] != -1)
                continue;
            labels[neighbors[j]] = cluster_id;
            clusters[cluster_id].push_back(neighbors[j]);
            std::vector<int> new_neighbors = getNeighbors(*frontiers, neighbors[j], epsilon);
            if (new_neighbors.size() >= min_points)
            {
                for (int k = 0; k < new_neighbors.size(); k++)
                {
                    if (std::find(neighbors.begin(), neighbors.end(), new_neighbors[k]) == neighbors.end())
                    {
                        neighbors.push_back(new_neighbors[k]);
                    }
                }
            }
        }
    }

    // calculate center of mass of each cluster
    std::vector<geometry_msgs::Point> new_frontiers_centers;

    for (auto cluster : clusters)
    {
        geometry_msgs::Point center;
        center.x = 0;
        center.y = 0;
        for (auto i : cluster)
        {
            center.x += frontiers->operator[](i).x;
            center.y += frontiers->operator[](i).y;
        }
        center.x /= cluster.size();
        center.y /= cluster.size();
        if (!isBlacklisted(center))
        {
            new_frontiers_centers.push_back(center);
        }
    }

    *frontiers = new_frontiers_centers;
}

std::vector<int> FrontierGeneration::getNeighbors(const std::vector<geometry_msgs::Point> &frontiers, int i, float epsilon)
{
    std::vector<int> neighbors;
    for (int j = 0; j < frontiers.size(); j++)
    {
        if (i == j)
            continue;
        if (dist2D(frontiers[i], frontiers[j]) < epsilon)
        {
            neighbors.push_back(j);
        }
    }
    return neighbors;
}

void FrontierGeneration::setExplorationArea(const geometry_msgs::Point &center, const int radius)
{
    exploration_center_ = center;
    exploration_radius_ = radius;
}

bool FrontierGeneration::isInExplorationArea(const int i, const int j)
{
    return dist2D(pointFrom2DMapIndex(i, j, map_), exploration_center_) < exploration_radius_;
}

bool FrontierGeneration::isBlacklisted(const int i, const int j)
{
    for (auto pt : blacklisted_pts_)
    {
        if (dist2D(pointFrom2DMapIndex(i, j, map_), pt) < 0.5)
        {
            return true;
        }
    }
    return false;
}

bool FrontierGeneration::isBlacklisted(const geometry_msgs::Point &pt)
{
    for (auto blacklisted_pt : blacklisted_pts_)
    {
        if (dist2D(blacklisted_pt, pt) < 0.5)
        {
            return true;
        }
    }
    return false;
}

void FrontierGeneration::addToBlacklist(const geometry_msgs::Point &pt)
{
    blacklisted_pts_.push_back(pt);
}

void FrontierGeneration::getExplorationArea(geometry_msgs::Point *exploration_center = nullptr, int *exploration_radius = nullptr)
{
    *exploration_center = exploration_center_;
    *exploration_radius = exploration_radius_;
}
