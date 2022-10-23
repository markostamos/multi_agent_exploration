#include <mae_btrees/HandleBT.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mae_btrees/conditions.h>
#include <mae_btrees/actions.h>
#include <mae_btrees/ros_comm.h>
extern RosComm state;

HandleBT::HandleBT(ros::NodeHandle &nh) : nh_(nh)
{

    initRosComm(nh_);

    pose_subscriber_ = nh_.subscribe("/odometry", 100, &HandleBT::subPoseCb, this);
    frontier_subscriber_ = nh_.subscribe("/frontiers", 100, &HandleBT::subFrontierCb, this);
    plan_subscriber_ = nh_.subscribe("/plan", 100, &HandleBT::subPlanCb, this);
    task_subscriber_ = nh_.subscribe("/task", 100, &HandleBT::subTaskCb, this);
    active_task_publisher_ = nh_.advertise<std_msgs::String>("/active_task", 1);
    lidar_readings_subscriber_ = nh_.subscribe("/lidar", 100, &HandleBT::subLidarReadingsCb, this);
    timer_ = nh_.createTimer(ros::Duration(0.5), &HandleBT::pubActiveTaskCb, this);
}

void HandleBT::createTree(std::string path)
{
    BT::BehaviorTreeFactory factory;

    // REGISTER ACTIONS
    bool nav_3d = false;
    ros::param::get("~nav_3d", nav_3d);
    if (nav_3d)
        factory.registerNodeType<GoTo3D>("GoTo");
    else
        factory.registerNodeType<GoTo>("GoTo");

    factory.registerNodeType<SetLocation>("SetLocation");
    factory.registerNodeType<TakeOff>("TakeOff");
    factory.registerNodeType<Land>("Land");
    factory.registerNodeType<SetNextTargetGreedy>("SetNextTargetGreedy");
    factory.registerNodeType<SetNextTargetFromPlan>("SetNextTargetFromPlan");
    factory.registerNodeType<SetTask>("SetTask");
    factory.registerNodeType<ClearTask>("ClearTask");
    factory.registerNodeType<TestActivity>("TestActivity");
    factory.registerNodeType<GoToBackupTarget>("GoToBackupTarget");
    factory.registerSimpleAction("Recharge", std::bind(Recharge));

    // REGISTER CONDITIONS
    factory.registerNodeType<TargetDiscovered>("TargetDiscovered");
    factory.registerNodeType<BetterTargetFound>("BetterTargetFound");
    factory.registerNodeType<GreedyTargetDiscovered>("GreedyTargetDiscovered");
    factory.registerSimpleCondition("TaskAvailable", std::bind(TaskAvailable));
    factory.registerSimpleCondition("LowBattery", std::bind(LowBattery));
    factory.registerSimpleCondition("isFrontierListEmpty", std::bind(isFrontierListEmpty));
    factory.registerSimpleCondition("isPlanEmpty", std::bind(isPlanEmpty));
    factory.registerSimpleCondition("NewPlanArrived", std::bind(NewPlanArrived));

    tree_ = factory.createTreeFromFile(path);

    // Set initial task to idle.
    tree_.rootBlackboard()->set<std::string>("TASK", "Idle");
}

void HandleBT::subPoseCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    static bool first_msg = true;

    // Keep first msg to set home location.
    if (first_msg)
    {
        tree_.rootBlackboard()->set<geometry_msgs::Pose>("Home", msg->pose.pose);
        first_msg = false;
    }
    state.pose = msg->pose.pose;
};

void HandleBT::subFrontierCb(const mae_utils::PointArray::ConstPtr &msg)
{
    state.frontier_pts = msg->points;
};

void HandleBT::subPlanCb(const mae_utils::PointArray::ConstPtr &msg)
{
    // first point is assumed to be the current location, so it gets ignored.
    state.plan_pts = std::vector<geometry_msgs::Point>(msg->points.begin() + 1, msg->points.end());
}

void HandleBT::subTaskCb(const std_msgs::String::ConstPtr &msg)
{
    state.requested_task = msg->data;
    state.task_arrived = true;
}

void HandleBT::pubActiveTaskCb(const ros::TimerEvent &event)
{
    std_msgs::String msg;
    msg.data = tree_.rootBlackboard()->get<std::string>("TASK");
    active_task_publisher_.publish(msg);
}

void HandleBT::subLidarReadingsCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    state.lidar_readings = *msg;
}
