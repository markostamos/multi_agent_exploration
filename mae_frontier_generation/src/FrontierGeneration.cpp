#include <mae_frontier_generation/FrontierGeneration.h>
#include <mae_utils/utils.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <geometry_msgs/PointStamped.h>
inline bool FrontierGeneration::isFrontier(int i, int j) const
{

    return ((i > 0 && map_.data[i - 1 + j * map_.info.width] == -1) ||
            (i < map_.info.width - 1 && map_.data[i + 1 + j * map_.info.width] == -1) ||
            (j > 0 && map_.data[i + (j - 1) * map_.info.width] == -1) ||
            (j < map_.info.height - 1 && map_.data[i + (j + 1) * map_.info.width] == -1));
}

void FrontierGeneration::updateMap(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;
}

void FrontierGeneration::updateLocation(const geometry_msgs::PointStamped &location, int index)
{
    if (index == location_.size())
    {
        location_.push_back(location.point);
    }
    else
    {
        location_[index] = location.point;
    }
}
void FrontierGeneration::updateOctomap(const octomap_msgs::Octomap octomap)
{
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(octomap);
    if (tree)
    {
        delete octree_;
        octree_ = dynamic_cast<octomap::OcTree *>(tree);
        if (octree_ == nullptr)
        {
            ROS_ERROR_STREAM("octree is null");
        }
    }
}

void FrontierGeneration::get3DFrontiers(std::vector<geometry_msgs::Point> *frontiers)
{

    if (octree_ == nullptr)
    {
        return;
    }

    for (const auto &loc : location_)
    {

        float xy_range = 20;
        float z_range = 10;

        octomap::point3d min(loc.x - xy_range, loc.y - xy_range, loc.z + 1);
        octomap::point3d max(loc.x + xy_range, loc.y + xy_range, loc.z + z_range);

        octomap::point3d_list unknown_cells;
        octree_->getUnknownLeafCenters(unknown_cells, min, max, 13);

        // LAMBDA FUNCTION TO CHECK IF A POINT SHOULD BE PUSHED TO FRONTIERS
        auto check_point = [&](const octomap::point3d &pt, int range)
        {
            octomap::OcTreeKey key;
            if (octree_->coordToKeyChecked(pt, key))
            {
                // check key neighbors for occupied
                for (int dx = -range; dx <= range; dx++)
                {
                    for (int dy = -range; dy <= range; dy++)
                    {
                        for (int dz = -range; dz <= range; dz++)
                        {
                            octomap::OcTreeKey nkey = key;
                            nkey[0] += dx;
                            nkey[1] += dy;
                            nkey[2] += dz;
                            octomap::OcTreeNode *node = octree_->search(nkey, 16);
                            if (node != nullptr && octree_->isNodeOccupied(node))
                                return true;
                        }
                    }
                }
            }
            return false;
        };

        for (octomap::point3d_list::iterator it = unknown_cells.begin(); it != unknown_cells.end(); ++it)
        {

            if (check_point(*it, 5))
            {
                geometry_msgs::Point pt;
                pt.x = it->x();
                pt.y = it->y();
                pt.z = it->z();
                frontiers->push_back(pt);
            }
        }
    }
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
                !isBlacklisted(pointFrom2DMapIndex(i, j, map_)) &&
                isInExplorationArea(i, j) &&
                !isNearObstacle(i, j, threshold))
            {
                frontiers->push_back(pointFrom2DMapIndex(i, j, map_));
            }
        }
    }
}

bool FrontierGeneration::isNearObstacle(int i, int j, float threshold) const
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
    geometry_msgs::Point center;
    for (const auto &cluster : clusters)
    {
        center.x = 0;
        center.y = 0;
        center.z = 0;
        for (const auto &i : cluster)
        {
            center.x += frontiers->operator[](i).x;
            center.y += frontiers->operator[](i).y;
            center.z += frontiers->operator[](i).z;
        }
        center.x /= cluster.size();
        center.y /= cluster.size();
        center.z /= cluster.size();
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
        if (dist2D(frontiers[i], frontiers[j]) < epsilon && i != j)
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

bool FrontierGeneration::isInExplorationArea(const int i, const int j) const
{
    return dist2D(pointFrom2DMapIndex(i, j, map_), exploration_center_) < exploration_radius_;
}

bool FrontierGeneration::isBlacklisted(const geometry_msgs::Point &pt) const
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
