#ifndef GOTO_BACKUP_TARGET_H
#define GOTO_BACKUP_TARGET_H

/* Set Locations to blackboard by name */
extern RosComm state;
class GoToBackupTarget : public BT::SyncActionNode
{
public:
    GoToBackupTarget(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        simple_move_base_goal_pub_ = state.nh->advertise<geometry_msgs::PoseStamped>(state.ns + "/move_base_simple/goal", 1);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick()
    {
        geometry_msgs::Pose current_target;

        if (state.checkpoints.empty())
            return BT::NodeStatus::FAILURE;

        // calculate average point
        geometry_msgs::Point avg_point;
        for (auto point : state.frontier_pts)
        {
            avg_point.x += point.x;
            avg_point.y += point.y;
            avg_point.y += point.z;
        }
        avg_point.x /= state.frontier_pts.size();
        avg_point.y /= state.frontier_pts.size();
        avg_point.z /= state.frontier_pts.size();

        geometry_msgs::Point target = *std::min_element(state.frontier_pts.begin(), state.frontier_pts.end(),
                                                        [&](const geometry_msgs::Point &a, const geometry_msgs::Point &b)
                                                        {
                                                            return dist2D(a, avg_point) < dist2D(b, avg_point);
                                                        });
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "world";
        msg.pose = poseFromVec({target.x, target.y, 0});
        simple_move_base_goal_pub_.publish(msg);
        state.checkpoints.clear();
        return BT::NodeStatus::SUCCESS;
    }

    ros::Publisher simple_move_base_goal_pub_;
};

#endif
