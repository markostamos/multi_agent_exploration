#include <ros/ros.h>

#include <mae_control/Planner3D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
class MoveBase3DNode
{
public:
    ros::NodeHandle nh_;
    ros::Subscriber location_subscriber_;
    ros::Subscriber goal_subsrciber_;
    ros::Subscriber octomap_subscriber_;
    ros::Publisher path_publisher_;
    geometry_msgs::Point location_;
    Planner3D planner_3d_;

public:
    MoveBase3DNode(const ros::NodeHandle &nh) : nh_(nh)
    {
        location_subscriber_ = nh_.subscribe("/start_in", 1, &MoveBase3DNode::subLocationCallback, this);
        goal_subsrciber_ = nh_.subscribe("/goal_in", 1, &MoveBase3DNode::subGoalCallback, this);
        path_publisher_ = nh_.advertise<nav_msgs::Path>("/path_out", 1);
        octomap_subscriber_ = nh_.subscribe("/octomap_full", 1, &MoveBase3DNode::subOctomapCallback, this);
    }

    void subLocationCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        location_ = msg->point;
    }
    void subGoalCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        geometry_msgs::Point goal = *msg;

        nav_msgs::Path path = planner_3d_.calculatePath(location_, goal, 1.0);
        path_publisher_.publish(path);
    }
    void subOctomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        planner_3d_.updateOctree(*msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_3d_node");
    ros::NodeHandle nh;
    MoveBase3DNode move_base_3d_node(nh);
    ros::spin();
}