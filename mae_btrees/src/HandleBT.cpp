#include <mae_btrees/HandleBT.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mae_btrees/conditions.h>
#include <mae_btrees/actions.h>
#include <mae_btrees/ros_comm.h>
extern RosComm state;

HandleBT::HandleBT(ros::NodeHandle &nh) : nh_(nh)
{

    initRosComm(nh_);

    pose_subscriber_ = nh_.subscribe(state.ns + "/ground_truth/odometry", 100, &HandleBT::subPoseCallback, this);
    frontier_subscriber_ = nh_.subscribe(state.ns + "/frontiers", 100, &HandleBT::subFrontierCallback, this);
    plan_subscriber_ = nh_.subscribe(state.ns + "/plan", 100, &HandleBT::subPlanCallback, this);
    task_subscriber_ = nh_.subscribe(state.ns + "/task", 100, &HandleBT::subTaskCallback, this);
    active_task_publisher_ = nh_.advertise<std_msgs::String>(state.ns + "/active_task", 1);
    lidar_readings_subscriber_ = nh_.subscribe(state.ns + "/velodyne_points", 100, &HandleBT::subLidarReadingsCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.5), &HandleBT::pubActiveTaskCallback, this);
    waitForConnection();
}

void HandleBT::createTree(std::string path)
{
    BT::BehaviorTreeFactory factory;

    // REGISTER ACTIONS
    factory.registerNodeType<GoTo>("GoTo");
    factory.registerNodeType<SetLocation>("SetLocation");
    factory.registerNodeType<TakeOff>("TakeOff");
    factory.registerNodeType<Land>("Land");
    factory.registerNodeType<SetNextTargetGreedy>("SetNextTargetGreedy");
    factory.registerNodeType<SetNextTargetFromPlan>("SetNextTargetFromPlan");

    factory.registerNodeType<SetTask>("SetTask");
    factory.registerNodeType<ClearTask>("ClearTask");
    factory.registerNodeType<TestActivity>("TestActivity");
    factory.registerSimpleAction("Recharge", std::bind(Recharge));
    // REGISTER CONDITIONS
    factory.registerSimpleCondition("TaskAvailable", std::bind(TaskAvailable));
    factory.registerSimpleCondition("LowBattery", std::bind(LowBattery));

    factory.registerSimpleCondition("isFrontierListEmpty", std::bind(isFrontierListEmpty));
    factory.registerSimpleCondition("isPlanEmpty", std::bind(isPlanEmpty));
    factory.registerNodeType<TargetFound>("TargetFound");
    factory.registerNodeType<TargetDiscovered>("TargetDiscovered");
    factory.registerNodeType<GreedyTargetDiscovered>("GreedyTargetDiscovered");
    factory.registerSimpleCondition("NewPlanArrived", std::bind(NewPlanArrived));
    tree_ = factory.createTreeFromFile(path);

    tree_.rootBlackboard()->set<std::string>("TASK", "Idle");
}

/*
    SUBSCRIBER CALLBACKS
 */
void HandleBT::subPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static bool first_msg = true;
    if (first_msg)
    {
        tree_.rootBlackboard()->set<geometry_msgs::Pose>("Home", msg->pose.pose);
        ROS_WARN_STREAM("Home set to: " << msg->pose.pose);
        first_msg = false;
    }
    state.pose = msg->pose.pose;
};

void HandleBT::subFrontierCallback(const mae_utils::PointArray::ConstPtr &msg)
{
    state.frontier_pts = msg->points;
};

void HandleBT::subPlanCallback(const mae_utils::PointArray::ConstPtr &msg)
{
    // get all poitns from msg except from first

    state.plan_pts = std::vector<geometry_msgs::Point>(msg->points.begin() + 1, msg->points.end());
}

void HandleBT::subTaskCallback(const std_msgs::String::ConstPtr &msg)
{
    state.requested_task = msg->data;
    state.task_arrived = true;
}

void HandleBT::pubActiveTaskCallback(const ros::TimerEvent &event)
{
    std_msgs::String msg;
    msg.data = tree_.rootBlackboard()->get<std::string>("TASK");
    active_task_publisher_.publish(msg);
}

void HandleBT::subLidarReadingsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    state.lidar_readings = *msg;
}