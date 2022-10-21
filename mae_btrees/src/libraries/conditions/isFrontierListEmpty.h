#ifndef IS_FRONTIER_LIST_EMPTY_H
#define IS_FRONTIER_LIST_EMPTY_H

extern RosComm state;
/**
 * @brief Checks whether the list of frontier points is empty.
 *
 * @returns Success if the list is empty, failure otherwise.
 */
BT::NodeStatus isFrontierListEmpty()
{

    return state.frontier_pts.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

#endif