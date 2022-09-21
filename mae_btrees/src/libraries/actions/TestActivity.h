#ifndef TEST_ACTIVITY_H
#define TEST_ACTIVITY_H

// Example of Asynchronous node that use StatefulActionNode as base class
class TestActivity : public BT::StatefulActionNode
{
public:
    TestActivity(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return {BT::InputPort<int>("msec")};
    }

    BT::NodeStatus onStart() override
    {
        ROS_INFO_STREAM("STARTED ASYNC ACTIVITY");
        int msec = 0;
        getInput("msec", msec);

        if (msec <= 0)
        {
            // No need to go into the RUNNING state
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            using namespace std::chrono;
            // once the deadline is reached, we will return SUCCESS.
            deadline_ = system_clock::now() + milliseconds(msec);
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus onRunning() override
    {
        if (std::chrono::system_clock::now() >= deadline_)
        {
            ROS_INFO_STREAM("ASYNC ACTIVITY FINISHED");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        // nothing to do here...
        ROS_INFO_STREAM("ASYNC ACTIVITY interrupted!!!!!!");
    }

private:
    std::chrono::system_clock::time_point deadline_;
};
#endif // TEST_ACTIVITY_H
