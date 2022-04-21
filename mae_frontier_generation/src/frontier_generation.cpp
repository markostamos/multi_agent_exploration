#include <ros/ros.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
class FrontierGeneration
{

private:
    int counter;
    ros::Publisher pub;
    ros::Subscriber number_subscriber;
    ros::ServiceServer reset_service;

public:
    FrontierGeneration(ros::NodeHandle *nh)
    {
        cv::Mat frame;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    FrontierGeneration fg = FrontierGeneration(&nh);
    ros::spin();
}