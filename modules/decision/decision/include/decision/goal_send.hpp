#ifndef GOAL_SEND_H
#define GOAL_SEND_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <data/MoveAction.h>
#include <data/Opening.h>
#include <data/ShopActionAction.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>

namespace shop
{
namespace decision
{
typedef actionlib::SimpleActionClient<data::MoveAction> MOVEACTIONCLINT;
typedef actionlib::SimpleActionClient<data::OpeningAction> OPENINGCLINT;
typedef actionlib::SimpleActionClient<data::ShopActionAction> SHOPACTION;

class GoalSend
{
  public:
    GoalSend(const Blackboard::Ptr &blackboard_ptr) : blackboard_ptr_(blackboard_ptr),
    move_action_clint_(name_+ "", true),
    open_action_clint_(),
    shop_action_clint_()
    {
    }
    ~GoalSend() = default;

  private:
    Blackboard::Ptr blackboard_ptr_;

    ros::NodeHandle nh_;
    MOVEACTIONCLINT move_action_clint_;
    OPENINGCLINT open_action_clint_;
    SHOPACTION shop_action_clint_;
};

} // namespace decision
} // namespace shop

#endif
