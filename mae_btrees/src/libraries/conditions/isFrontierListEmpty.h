#ifndef IS_FRONTIER_LIST_EMPTY_H
#define IS_FRONTIER_LIST_EMPTY_H

extern RosComm state;
BT::NodeStatus isFrontierListEmpty()
{

    return state.frontier_pts.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

#endif