#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <chrono>
#include <stdint.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <behavior_tree/behavior_node.h>
#include <behavior_tree/black_board.h>

namespace shop
{
namespace decision
{

class behavior_tree
{
  public:
    behavior_tree(BehaviorNode::Ptr root_node_ptr, int cycle_duration)
        : root_node_ptr_(root_node_ptr), cycle_duration_(cycle_duration)
        ,running_(false)
        {};
    
    void Execute()
    {
        running_ = true;
        uint64_t index = 0
        ros::spin();
        while(ros::ok && running_){
            ros::spin();
            root_node_ptr_->Run();
            ROS_INFO("tree is Run");
            index_++;
        }
    }

    void Reset()
    {
        running_ = true;
    }

    void Stop()
    {
        running_ = false;
    }

    virtual ~behavior_tree() = default;

  private:
    BehaviorNode::Ptr root_node_ptr_;
    int cycle_duration_;
    bool running_;
};
} // namespace decision
} // namespace shop


#endif