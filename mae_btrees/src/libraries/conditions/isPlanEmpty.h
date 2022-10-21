#ifndef PLAN_EMPTY_H
#define PLAN_EMPTY_H

extern RosComm state;
/**
 * @brief Checks whether the list of plan points is empty.
 *
 * @return Success if the list is empty, failure otherwise.
 */
BT::NodeStatus isPlanEmpty()
{

    return state.plan_pts.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

#endif // PLAN_EMPTY_H
