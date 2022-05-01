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
    ros::Publisher frontiers_viz_pub_;
    ros::Publisher frontiers_pub_;

    ros::Timer get_frontiers_timer_;

    // params
    float update_rate_;
    float obstacle_threshold_;
    float viz_scale_;
    // DBSCAN params
    bool filter_frontiers_;
    float min_pts_;
    float epsilon_;

    FrontierGeneration FrontierGeneration_;

public:
    FrontierGenerationNode(const ros::NodeHandle &nh) : nh_(nh),
                                                        ns_(nh.getNamespace().c_str()),
                                                        FrontierGeneration_(FrontierGeneration())
    {
        map_sub_ = nh_.subscribe("/map_in", 10, &FrontierGenerationNode::storeMap, this);
        frontiers_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/frontiers_viz_out", 10);
        frontiers_pub_ = nh_.advertise<mae_utils::PointArray>("/frontiers_out", 10);

        nh.param<float>("update_rate", update_rate_, 10);
        nh.param<bool>("filter_frontiers", filter_frontiers_, true);
        nh.param<float>("min_pts", min_pts_, 1);
        nh.param<float>("epsilon", epsilon_, 0.5);
        nh.param<float>("obstacle_padding", obstacle_threshold_, 0.4);
        nh.param<float>("viz_scale", viz_scale_, 0.5);

        get_frontiers_timer_ = nh_.createTimer(ros::Duration(1 / update_rate_), &FrontierGenerationNode::publishFrontiers, this);
    }

private:
    void storeMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        FrontierGeneration_.updateMap(*msg);
    }

    void publishFrontiers(const ros::TimerEvent &event)
    {
        std::vector<geometry_msgs::Point> frontiers;
        FrontierGeneration_.getFrontiers(&frontiers, obstacle_threshold_);
        if (filter_frontiers_)
        {
            FrontierGeneration_.filterFrontiersDBSCAN(&frontiers, min_pts_, epsilon_);
        }

        frontiers_viz_pub_.publish(createMarkerMsg(frontiers, viz_scale_));
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