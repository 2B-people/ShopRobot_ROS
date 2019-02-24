#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include <ros/ros.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>

//TODO
namespace shop
{
namespace decision
{

class MoveAction : public ActionNode
{
  public:
    MoveAction();
    ~MoveAction();

  private:
};

} // namespace decision
} // namespace shop

#endif