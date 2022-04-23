#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <mae_frontier_generation/utils.h>
#include <mae_frontier_generation/FrontierGeneration.h>
/*
    TODO: find a way to reduce the number of points
    TODO: simple greedy exploration algorithm
    TODO: Hough line transform
    TODO: Parameters and remapping of topics via launchf file
    TODO: github separate package with link
    TODO: Comments @brief @param @return doxygen

*/
class FrontierGenerationNode
{

private:
    ros::Subscriber map_sub_;
    ros::Publisher frontiers_viz_pub_;
    ros::Publisher frontiers_pub_;
    nav_msgs::OccupancyGrid map_;
    std::string ns_;
    ros::NodeHandle nh_;
    ros::Timer get_frontiers_timer_;
    FrontierGeneration FrontierGeneration_;

    ros::Publisher test;

public:
    FrontierGenerationNode(const ros::NodeHandle &nh) : nh_(nh),
                                                        ns_(nh.getNamespace().c_str())
    {
        FrontierGeneration_ = FrontierGeneration();
        map_sub_ = nh_.subscribe(ns_ + "/projected_map", 10, &FrontierGenerationNode::storeMap, this);
        frontiers_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(ns_ + "/frontiers_viz", 100);
        frontiers_pub_ = nh_.advertise<geometry_msgs::Point>(ns_ + "/frontiers", 100);
        test = nh_.advertise<geometry_msgs::Point>("/test", 100);
        get_frontiers_timer_ = nh_.createTimer(ros::Duration(0.5), &FrontierGenerationNode::publishFrontiers, this);
    }

private:
    void storeMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        FrontierGeneration_.updateMap(*msg);
    }

    void publishFrontiers(const ros::TimerEvent &event)
    {

        std::vector<geometry_msgs::Point> frontiers;
        FrontierGeneration_.getFrontiers(&frontiers);
        frontiers_viz_pub_.publish(createMarkerMsg(frontiers));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_generation");
    ros::NodeHandle nh;
    FrontierGenerationNode fg = FrontierGenerationNode(nh);
    ros::spin();
}