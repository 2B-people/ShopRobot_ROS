#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

using namespace shop::decision;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision");
    auto blackboard_ptr = std::make_shared<Blackboard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr);

    //decision
    auto robot_ptr = std::make_shared<shop::decision::SelectorNode>("robot_tests", blackboard_ptr);
    shop::decision::BehaviorTree se(robot_ptr, 100);
    se.Execute();
}