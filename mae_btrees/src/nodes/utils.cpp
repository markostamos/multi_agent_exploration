#include <mae_btrees/utils.h>

BT::NodeStatus print_blackboard_value(BT::TreeNode &self)
{
    BT::Optional<std::string> msg = self.getInput<std::string>("input");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [input]: ", msg.error());
    }

    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}



BT::NodeStatus update_blackboard::tick()
{
    // the output may change at each tick(). Here we keep it simple.
    setOutput("text", "The answer is 42");
    return BT::NodeStatus::SUCCESS;
}
