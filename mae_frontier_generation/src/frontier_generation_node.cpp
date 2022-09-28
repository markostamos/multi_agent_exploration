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
    ros::Subscriber blacklisted_pt_sub_;
    ros::Publisher frontiers_viz_pub_;
    ros::Publisher frontiers_pub_;
    ros::Publisher exploration_area_pub_;

    ros::Timer get_frontiers_timer_;
    ros::Timer get_exploration_area_timer_;

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
        blacklisted_pt_sub_ = nh_.subscribe("/blacklist_pt_in", 10, &FrontierGenerationNode::storeBlacklistedPt, this);
        frontiers_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/frontiers_viz_out", 10);
        frontiers_pub_ = nh_.advertise<mae_utils::PointArray>("/frontiers_out", 10);
        exploration_area_pub_ = nh_.advertise<visualization_msgs::Marker>("/exploration_area_out", 10);
        nh.param<float>("update_rate", update_rate_, 2);
        nh.param<bool>("filter_frontiers", filter_frontiers_, true);
        nh.param<float>("min_pts", min_pts_, 1);
        nh.param<float>("epsilon", epsilon_, 0.5);
        nh.param<float>("obstacle_padding", obstacle_threshold_, 0.4);
        nh.param<float>("viz_scale", viz_scale_, 0.5);

        get_frontiers_timer_ = nh_.createTimer(ros::Duration(1 / update_rate_), &FrontierGenerationNode::publishFrontiers, this);
        get_exploration_area_timer_ = nh_.createTimer(ros::Duration(1), &FrontierGenerationNode::publishExplorationArea, this);
    }

private:
    void storeMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        FrontierGeneration_.updateMap(*msg);
    }

    void publishFrontiers(const ros::TimerEvent &event)
    {
        // TODO: fix this
        static int last_size = 0;
        std::vector<geometry_msgs::Point> frontiers;
        FrontierGeneration_.getFrontiers(&frontiers, obstacle_threshold_);
        if (filter_frontiers_)
        {
            FrontierGeneration_.filterFrontiersDBSCAN(&frontiers, min_pts_, epsilon_);
        }
        frontiers_pub_.publish(createPointArrayMsg(frontiers));
        frontiers_viz_pub_.publish(createMarkerMsg(frontiers, viz_scale_));
    }
    void publishExplorationArea(const ros::TimerEvent &event)
    {
        geometry_msgs::Point center;
        int radius;
        FrontierGeneration_.getExplorationArea(&center, &radius);

        visualization_msgs::Marker msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        msg.ns = "exploration_area";
        msg.id = 0;
        msg.type = visualization_msgs::Marker::CYLINDER;
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.position = center;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 2 * radius;
        msg.scale.y = 2 * radius;
        msg.scale.z = 0.001;
        msg.color.a = 0.1;
        msg.color.r = 0.0;
        msg.color.g = 0.2;
        msg.color.b = 1.0;
        exploration_area_pub_.publish(msg);
    }
    void storeBlacklistedPt(const geometry_msgs::Point::ConstPtr &msg)
    {
        FrontierGeneration_.addToBlacklist(*msg);
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_generation");
    ros::NodeHandle nh;
    FrontierGenerationNode fg = FrontierGenerationNode(nh);
    ros::spin();
}