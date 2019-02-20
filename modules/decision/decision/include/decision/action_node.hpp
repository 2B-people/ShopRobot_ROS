#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>



namespace shop{
namespace decision{

class MoveAction :public ActionNode
{
public:
    MoveAction();
    ~MoveAction();
private:
    
};






} // namespace decision
} // namespace shop

#endif