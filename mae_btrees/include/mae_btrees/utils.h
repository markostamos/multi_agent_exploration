#ifndef UTILS_H
#define UTILS_H


BT::NodeStatus print_blackboard_value(BT::TreeNode &self);

class update_blackboard : public BT::SyncActionNode
{
public:
    update_blackboard(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("text")};
    }

    // This Action writes a value into the port "text"
    BT::NodeStatus tick() override;
};


struct Position2D
{
    double x;
    double y;
};



// function that gets a string and outputs position2D
Position2D pos2D_from_string(std::string str){
    auto tokens = BT::splitString(str,',');
    Position2D output;
    output.x = BT::convertFromString<double>(tokens[0]);
    output.y = BT::convertFromString<double>(tokens[1]);
    return output;
};



#endif // UTILS_H