
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
class Planner3D
{
public:
    Planner3D();
    nav_msgs::Path calculatePath(geometry_msgs::Point start, geometry_msgs::Point goal, float timeout_s);

    nav_msgs::Path navPathFromOmplPath(const ompl::geometric::PathGeometric &omplPath);

    void updateOctree(const octomap_msgs::Octomap &octomap);

private:
    bool isStateValid(const ompl::base::State *state);

private:
    ompl::base::RealVectorBounds bounds_;
    octomap::OcTree *octree_;
};