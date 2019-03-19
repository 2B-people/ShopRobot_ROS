#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <chrono>
#include <stdint.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>


namespace shop{
namespace decision{

class BehaviorTree
{
  public:
    BehaviorTree(BehaviorNode::Ptr root_node_ptr, int cycle_duration)
        : root_node_ptr_(root_node_ptr), cycle_duration_(cycle_duration), running_(false){};
    virtual ~BehaviorTree() = default;
    // @ breif run方法
    void Execute()
    {
        running_ = true;
        uint64_t index = 0;
        while (ros::ok)
        {
            if (running_)
            {
                ros::spinOnce();
                root_node_ptr_->Run();
                // ROS_INFO("tree is Run num:%d", (int)index);
                index++;
                ROS_INFO("-------------------------------");
            }
            else
            {
                ROS_INFO("tree is shop ,in num%d",(int)index);
                ROS_INFO("-------------------------------");
            }   
        }
        
    }
    // @breif 重启行为树
    void Reset()
    {
        running_ = true;
    }
    // @breif 关闭行为树
    void Stop()
    {
        running_ = false;
    }

  private:
    BehaviorNode::Ptr root_node_ptr_;
    int cycle_duration_;
    bool running_;
};
} // namespace decision
} // namespace shop

#endif