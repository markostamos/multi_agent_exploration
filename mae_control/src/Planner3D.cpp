#include <mae_control/Planner3D.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <iostream>
#include <ros/ros.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

Planner3D::Planner3D() : bounds_(3), octree_(nullptr)

{
    ROS_WARN_STREAM("Planner3D constructor");
    // set bounds

    bounds_.setLow(0, -10);
    bounds_.setHigh(0, 10);
    bounds_.setLow(1, -10);
    bounds_.setHigh(1, 10);
    bounds_.setLow(2, 0);
    bounds_.setHigh(2, 10);
}

bool Planner3D::isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    // extract the first component of the position
    double x = pos->values[0];
    // extract the second component of the position
    double y = pos->values[1];
    // extract the third component of the position
    double z = pos->values[2];

    // search for node in xyz
    octomap::OcTreeNode *node = octree_->search(x, y, z, 15);
    if (node != nullptr)
    {
        // check if node is occupied
        if (octree_->isNodeOccupied(node))
        {
            return false;
        }
    }

    return true;
}

nav_msgs::Path Planner3D::calculatePath(geometry_msgs::Point start, geometry_msgs::Point goal, float timeout_s)
{
    nav_msgs::Path path;

    auto space(std::make_shared<ob::SE3StateSpace>());
    space->setLongestValidSegmentFraction(0.01);

    // get dynamic bounds from start and end positions
    bounds_.setLow(0, std::min(start.x, goal.x) - 3);
    bounds_.setHigh(0, std::max(start.x, goal.x) + 3);
    bounds_.setLow(1, std::min(start.y, goal.y) - 3);
    bounds_.setHigh(1, std::max(start.y, goal.y) + 3);
    bounds_.setLow(2, std::min(start.z, goal.z));
    bounds_.setHigh(2, std::max(start.z, goal.z) + 3);

    space->setBounds(bounds_);
    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([&](const ob::State *state)
                               { return isStateValid(state); });
    /*  ss.setStateValidityCheckingResolution(0.01); */
    ob::ScopedState<ob::SE3StateSpace> start_state(space);
    start_state->setX(start.x);
    start_state->setY(start.y);
    start_state->setZ(start.z);
    start_state->rotation().setIdentity();

    ob::ScopedState<ob::SE3StateSpace> goal_state(space);
    goal_state->setX(goal.x);
    goal_state->setY(goal.y);
    goal_state->setZ(goal.z);
    goal_state->rotation().setIdentity();

    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
    // set planner range to 0.01
    ss.getPlanner()->as<og::RRTConnect>()->setRange(0.1);
    ss.setStartAndGoalStates(start_state, goal_state);

    ob::PlannerStatus solved = ss.solve(timeout_s);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();

        og::PathGeometric path_geom = ss.getSolutionPath();
        path_geom.interpolate();
        path = navPathFromOmplPath(path_geom);
    }
    else
        std::cout << "No solution found" << std::endl;
    return path;
}

nav_msgs::Path Planner3D::navPathFromOmplPath(const og::PathGeometric &omplPath)
{
    nav_msgs::Path path;

    path.header.frame_id = "world";

    for (int i = 0; i < omplPath.getStateCount(); i++)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = omplPath.getState(i)->as<ob::SE3StateSpace::StateType>()->getX();
        pose_msg.pose.position.y = omplPath.getState(i)->as<ob::SE3StateSpace::StateType>()->getY();
        pose_msg.pose.position.z = omplPath.getState(i)->as<ob::SE3StateSpace::StateType>()->getZ();
        pose_msg.header.frame_id = "world";
        pose_msg.header.stamp = ros::Time::now();

        path.poses.push_back(pose_msg);
    }

    return path;
}

void Planner3D::updateOctree(const octomap_msgs::Octomap &octomap)
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