#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <mae_frontier_generation/FrontierGeneration.h>
#include <mae_utils/PointArray.h>
#include <mae_utils/utils.h>
class FrontierGenerationNode
{

private:
    std::string ns_;
    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;
    ros::Subscriber costmap_sub_;
    ros::Publisher frontiers_viz_pub_;
    ros::Publisher frontiers_pub_;
    ros::Timer get_frontiers_timer_;

    FrontierGeneration FrontierGeneration_;

public:
    FrontierGenerationNode(const ros::NodeHandle &nh) : nh_(nh),
                                                        ns_(nh.getNamespace().c_str()),
                                                        FrontierGeneration_(FrontierGeneration())
    {

        map_sub_ = nh_.subscribe(ns_ + "/projected_map", 10, &FrontierGenerationNode::storeMap, this);
        costmap_sub_ = nh_.subscribe("/drone1/drone1_move_base/global_costmap/costmap", 10, &FrontierGenerationNode::storeCostMap, this);

        frontiers_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(ns_ + "/frontiers_viz", 100);
        frontiers_pub_ = nh_.advertise<mae_utils::PointArray>(ns_ + "/frontiers", 100);
        get_frontiers_timer_ = nh_.createTimer(ros::Duration(0.05), &FrontierGenerationNode::publishFrontiers, this);
    }

private:
    void storeMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        FrontierGeneration_.updateMap(*msg);
    }
    void storeCostMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        FrontierGeneration_.updateCostMap(*msg);
    }

    void publishFrontiers(const ros::TimerEvent &event)
    {

        std::vector<geometry_msgs::Point> frontiers;
        FrontierGeneration_.getFrontiers(&frontiers);
        frontiers_viz_pub_.publish(createMarkerMsg(frontiers));
        frontiers_pub_.publish(createPointArrayMsg(frontiers));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_generation");
    ros::NodeHandle nh;
    FrontierGenerationNode fg = FrontierGenerationNode(nh);
    ros::spin();
}