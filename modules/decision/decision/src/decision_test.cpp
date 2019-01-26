#include <decision/behavior_tree.h>
#include <decision/black_board.h>

using namespace shop::decision;

class ActionTest : public shop::decision::ActionNode
{

  public:
    ActionTest(std::string name, const Blackboard::Ptr &blackboard_ptr)
        : ActionNode::ActionNode(name, blackboard_ptr)
        {}
    ~ActionTest() = default;

  private:
    virtual void OnInitialize()
    {
        ROS_INFO("------------");
    };

    virtual BehaviorState Update()
    {
        ROS_INFO("%s is running!", name_.c_str());
        sleep(5);
        return BehaviorState::FAILURE;
    }

    virtual void OnTerminate(BehaviorState state)
    {
        switch (state)
        {
        case BehaviorState::IDLE:
            break;
        case BehaviorState::SUCCESS:
            break;
        case BehaviorState::FAILURE:
            break;
        default:
            return;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv ,"decision_test");
    auto blackboard_ptr = std::make_shared<shop::decision::Blackboard>();
    auto action1_ptr = std::make_shared<ActionTest>("actiontest1", blackboard_ptr);
    auto action2_ptr = std::make_shared<ActionTest>("actiontest2", blackboard_ptr);
    auto action3_ptr = std::make_shared<ActionTest>("actiontest3", blackboard_ptr);
    auto action4_ptr = std::make_shared<ActionTest>("actiontest4", blackboard_ptr);
    auto action5_ptr = std::make_shared<ActionTest>("actiontest5", blackboard_ptr);

    auto jud1_ptr = std::make_shared<shop::decision::PreconditionNode>("jud1_tests", blackboard_ptr,
                                                                       action1_ptr,
                                                                       [&]() {
                                                                           if (blackboard_ptr->test1 ())
                                                                           {
                                                                               return true;
                                                                           }
                                                                           else
                                                                           {
                                                                               return false;
                                                                           }
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto jud2_ptr = std::make_shared<shop::decision::PreconditionNode>("jud2_tests", blackboard_ptr,
                                                                       action2_ptr,
                                                                       [&]() {
                                                                           if (blackboard_ptr->test2 ())
                                                                           {
                                                                               return true;
                                                                           }
                                                                           else
                                                                           {
                                                                               return false;
                                                                           }
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto sw1_ptr = std::make_shared<shop::decision::SelectorNode>("sw1_tests", blackboard_ptr);
    sw1_ptr->AddChildren(jud2_ptr);
    sw1_ptr->AddChildren(jud1_ptr);

    auto sw1_ptr_node = std::make_shared<shop::decision::PreconditionNode>("sw1_tests_node", blackboard_ptr,
                                                                           sw1_ptr,
                                                                           [&]() {
                                                                               return true;
                                                                           },
                                                                           shop::decision::AbortType::LOW_PRIORITY);

    auto jud3_ptr = std::make_shared<shop::decision::PreconditionNode>("jud3_tests", blackboard_ptr,
                                                                       action3_ptr,
                                                                       [&]() {
                                                                           if (blackboard_ptr->test4 ())
                                                                           {
                                                                               return true;
                                                                           }
                                                                           else
                                                                           {
                                                                               return false;
                                                                           }
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto jud4_ptr = std::make_shared<shop::decision::PreconditionNode>("jud4_tests", blackboard_ptr,
                                                                       action4_ptr,
                                                                       [&]() {
                                                                           if (blackboard_ptr->test5 ())
                                                                           {
                                                                               return true;
                                                                           }
                                                                           else
                                                                           {
                                                                               return false;
                                                                           }
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto sw2_ptr = std::make_shared<shop::decision::SelectorNode>("sw1_tests", blackboard_ptr);
    sw2_ptr->AddChildren(jud3_ptr);
    sw2_ptr->AddChildren(jud4_ptr);
    sw2_ptr->AddChildren(action5_ptr);

    auto sw2_ptr_node = std::make_shared<shop::decision::PreconditionNode>("sw2_tests_node", blackboard_ptr,
                                                                           sw2_ptr,
                                                                           [&]() {
                                                                               if (blackboard_ptr->test6 ())
                                                                               {
                                                                                   return true;
                                                                               }
                                                                               else
                                                                               {
                                                                                   return false;
                                                                               }
                                                                           },
                                                                           shop::decision::AbortType::LOW_PRIORITY);

    auto robot_ptr = std::make_shared<shop::decision::SelectorNode>("robot_tests", blackboard_ptr);
    robot_ptr->AddChildren(sw1_ptr_node);
    robot_ptr->AddChildren(sw2_ptr_node);

    shop::decision::BehaviorTree se(robot_ptr,100);
    se.Execute();

}

