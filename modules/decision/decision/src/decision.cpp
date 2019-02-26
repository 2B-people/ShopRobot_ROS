#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

using namespace shop::decision;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision");
    auto blackboard_ptr = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr);

    //decision

    auto carry_ptr = std::make_shared<shop::decision::ParallelNode>("carrying", blackboard_ptr,4);
    auto opening_ptr = std::make_shared<shop::decision::ParallelNode>("opening", blackboard_ptr,5);
    auto robot_ptr = std::make_shared<shop::decision::SequenceNode>("game", blackboard_ptr);
    robot_ptr->AddChildren(opening_ptr);
    robot_ptr->AddChildren(carry_ptr);
    auto end_judge = std::make_shared<shop::decision::PreconditionNode>("end judge",blackboard_ptr,
                                                                        robot_ptr,
                                                                        [&](){
                                                                            return blackboard_ptr_->GetBoolValue("end_flag");
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    shop::decision::BehaviorTree se(end_judge, 100);
    se.Execute();
}