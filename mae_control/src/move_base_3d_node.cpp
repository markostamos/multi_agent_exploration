#include <ros/ros.h>

#include <mae_control/Planner3D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <mae_control/MoveBase3DAction.h>
#include <actionlib/server/simple_action_server.h>
#include <mutex>

typedef actionlib::SimpleActionServer<mae_control::MoveBase3DAction> Server;
class MoveBase3DNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber location_subscriber_;
    ros::Subscriber goal_subsrciber_;
    ros::Subscriber octomap_subscriber_;
    ros::Publisher path_publisher_;
    ros::Publisher goalpoint_publisher_;
    geometry_msgs::Point location_;
    Planner3D planner_3d_;

    Server server_;
    std::mutex mutex_;

public:
    MoveBase3DNode(const ros::NodeHandle &nh) : nh_(nh),
                                                server_(nh, "move_base_3d", boost::bind(&MoveBase3DNode::execute, this, _1), false)
    {
        location_subscriber_ = nh_.subscribe("/start_in", 1, &MoveBase3DNode::subLocationCallback, this);
        goal_subsrciber_ = nh_.subscribe("/drone1/move_base_3d_simple/goal", 1, &MoveBase3DNode::subGoalCallback, this);
        path_publisher_ = nh_.advertise<nav_msgs::Path>("/path_out", 1);
        octomap_subscriber_ = nh_.subscribe("/octomap_full", 1, &MoveBase3DNode::subOctomapCallback, this);
        goalpoint_publisher_ = nh_.advertise<geometry_msgs::Point>("/drone1/command/position", 1);

        server_.start();
    }

    void subLocationCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        location_ = msg->point;
    }
    void subGoalCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        geometry_msgs::Point goal = *msg;
        nav_msgs::Path path = planner_3d_.calculatePath(location_, goal, 0.5);
        if (path.poses.size() == 0)
            return;
        path_publisher_.publish(path);
        for (int i = 0; i < path.poses.size(); i++)
        {
            goalpoint_publisher_.publish(path.poses[i].pose.position);
            ros::Duration(0.03).sleep();
        }
    }
    void subOctomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        planner_3d_.updateOctree(*msg);
    }

    void execute(const mae_control::MoveBase3DGoalConstPtr &goal)
    {
        if (!server_.isActive() || server_.isPreemptRequested())
            return;
        geometry_msgs::Point goal_point = goal->target.position;

        float distance = std::numeric_limits<float>::max();

        int timeout = 0;

        while (distance > 0.2 && timeout < 10)
        {
            mutex_.lock();
            nav_msgs::Path path = planner_3d_.calculatePath(location_, goal_point, 0.5);
            mutex_.unlock();
            ROS_WARN_STREAM("PATH CALCULATED");
            if (path.poses.size() == 0)
            {
                server_.setAborted();
                ROS_WARN("[MoveBase3D] No path found to target");
                return;
            }

            ROS_WARN_STREAM("PATH CHECKED");

            path_publisher_.publish(path);
            ROS_WARN_STREAM("PATH PUBLISHED");
            for (int i = 0; i < path.poses.size(); i++)
            {
                goalpoint_publisher_.publish(path.poses[i].pose.position);
                ros::Duration(0.03).sleep();
            }

            mutex_.lock();
            distance = sqrt(pow(location_.x - goal_point.x, 2) + pow(location_.y - goal_point.y, 2) + pow(location_.z - goal_point.z, 2));
            mutex_.unlock();
            timeout++;
        }
        if (timeout >= 10)
        {
            server_.setAborted();
            ROS_WARN("[MoveBase3D] Timeout");
        }
        else
        {
            server_.setSucceeded(mae_control::MoveBase3DResult());
            ROS_INFO("[MoveBase3D] Goal reached");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_3d_node");
    ros::NodeHandle nh;
    MoveBase3DNode move_base_3d_node(nh);
    /* ros::MultiThreadedSpinner spinner(2);
    spinner.spin(); */
    ros::spin();
    return 0;
}