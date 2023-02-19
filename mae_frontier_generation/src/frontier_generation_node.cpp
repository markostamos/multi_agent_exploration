#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <mae_frontier_generation/FrontierGeneration.h>
#include <mae_utils/PointArray.h>
#include <mae_utils/utils.h>
#include <octomap_msgs/Octomap.h>

class FrontierGenerationNode
{

private:
    std::string ns_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber map_sub_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber blacklisted_pt_sub_;
    std::vector<ros::Subscriber> location_sub_;
    ros::Publisher frontiers_viz_pub_;
    ros::Publisher unfiltered_frontiers_viz_pub_;
    ros::Publisher frontiers_pub_;

    ros::Timer publish_frontiers_;

    // params
    float update_rate_;
    float obstacle_threshold_;
    int num_agents_;
    bool generate_3d_frontiers_;
    bool filter_frontiers_;

    FrontierGeneration FrontierGeneration_;

public:
    FrontierGenerationNode(const ros::NodeHandle &nh, ros::NodeHandle nh_private_) : nh_(nh),
                                                                                     nh_private_(nh_private_),
                                                                                     ns_(nh.getNamespace().c_str()),
                                                                                     FrontierGeneration_(FrontierGeneration())
    {

        nh_private_.param<float>("update_rate", update_rate_, 1);
        nh_private_.param<bool>("filter_frontiers", filter_frontiers_, true);
        nh_private_.param<float>("obstacle_padding", obstacle_threshold_, 0.4);
        nh_private_.param<bool>("frontiers_3d", generate_3d_frontiers_, false);
        nh_private_.param<int>("num_agents", num_agents_, 1);

        octomap_sub_ = nh_.subscribe("/octomap_full_in", 1, &FrontierGenerationNode::storeOctomap, this);
        map_sub_ = nh_.subscribe("/map_in", 1, &FrontierGenerationNode::storeMap, this);

        /*   for (int i = 0; i < num_agents_; i++)
          {
              location_sub_.push_back(nh_.subscribe<geometry_msgs::PointStamped>("/location_in" + std::to_string(i + 1),
                                                                                 1,
                                                                                 boost::bind(&FrontierGenerationNode::setLocation, this, _1, i)));
          } */

        blacklisted_pt_sub_ = nh_.subscribe("/blacklist_pt_in", 10, &FrontierGenerationNode::storeBlacklistedPt, this);
        frontiers_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/frontiers_viz_out", 10);
        unfiltered_frontiers_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/unfiltered_frontiers_viz_out", 10);
        frontiers_pub_ = nh_.advertise<mae_utils::PointArray>("/frontiers_out", 10);

        publish_frontiers_ = nh_.createTimer(ros::Duration(1 / update_rate_), &FrontierGenerationNode::publishFrontiers, this);
    }

private:
    void storeMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        FrontierGeneration_.updateMap(*msg);
    }

    void storeOctomap(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        FrontierGeneration_.updateOctomap(*msg);
    }
    void setLocation(const geometry_msgs::PointStamped::ConstPtr &msg, const int index)
    {
        FrontierGeneration_.updateLocation(*msg, index);
    }

    void publishFrontiers(const ros::TimerEvent &event)
    {
        std::vector<geometry_msgs::Point> frontiers;
        std::vector<geometry_msgs::Point> frontiers_3d;

        FrontierGeneration_.getFrontiers(&frontiers, obstacle_threshold_);
        if (!frontiers.empty())
            unfiltered_frontiers_viz_pub_.publish(createMarkerMsg(frontiers, 0.2, {255, 255, 0}));
        if (filter_frontiers_)
            FrontierGeneration_.filterFrontiersDBSCAN(&frontiers, 1, 0.5);

        if (generate_3d_frontiers_)
        {
            FrontierGeneration_.get3DFrontiers(&frontiers_3d);
            FrontierGeneration_.filterFrontiersDBSCAN(&frontiers_3d, 1, 2);

            frontiers.insert(frontiers.end(), frontiers_3d.begin(), frontiers_3d.end());
        }

        frontiers_pub_.publish(createPointArrayMsg(frontiers));
        frontiers_viz_pub_.publish(createMarkerMsg(frontiers, 0.5, {255, 255, 255}));
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
    ros::NodeHandle private_nh("~");
    FrontierGenerationNode fg = FrontierGenerationNode(nh, private_nh);
    ros::spin();
}