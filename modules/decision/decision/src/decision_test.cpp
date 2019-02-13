#include <ros/time.h>
#include <ros/duration.h>

#include<string>

#include <data/SerialTest.h>

#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

using namespace shop::decision;

class TestDir:public BoolDir
{

public:
    TestDir(bool key):BoolDir(key){
    };
    virtual ~TestDir() = default;
};



class BlackTest : public Blackboard
{
    public:
        BlackTest():Blackboard::Blackboard(){
            auto test_bool = std::make_shared<TestDir>(true);
            AddDataIntoWorld("test1",test_bool);
        }
    private:
        
};

class ActionTest : public shop::decision::ActionNode
{

  public:
    ActionTest(std::string name, const Blackboard::Ptr &blackboard_ptr)
        : ActionNode::ActionNode(name, blackboard_ptr),index_(0)
    {
        ros::Time::init();
        test_pub_ = nh_.advertise<data::SerialTest>("actiontest",1);
    }
    ~ActionTest() = default;

  private:
    ros::Publisher test_pub_;
    data::SerialTest test_msg_;
    ros::NodeHandle nh_;
    ros::Time begin_;
    uint64_t index_;
    virtual void OnInitialize()
    {
        begin_ = ros::Time::now();
    };

    virtual BehaviorState Update()
    {
        ros::Time at_now = ros::Time::now();
        double begin = begin_.toSec();
        double at_now_double =at_now.toSec(); 
        if(at_now - begin_ >= ros::Duration(3))
        {
            ROS_INFO("%s is success", name_.c_str());
            return BehaviorState::SUCCESS;
        }
        else
        {
            ROS_INFO("%s is running!", name_.c_str());
            for(size_t i = 0; i < 30; i++)
            {
                test_msg_.data[i] = index_;
            }
            test_pub_.publish(test_msg_);
            return BehaviorState::RUNNING;  
        }
        
    }

    virtual void OnTerminate(BehaviorState state)
    {
        ROS_INFO("------------");
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
    ros::init(argc, argv, "decision_test");
    auto blackboard_ptr = std::make_shared<BlackTest>();
    auto action1_ptr = std::make_shared<ActionTest>("actiontest1", blackboard_ptr);
    auto action2_ptr = std::make_shared<ActionTest>("actiontest2", blackboard_ptr);
    auto action3_ptr = std::make_shared<ActionTest>("actiontest3", blackboard_ptr);
    auto action4_ptr = std::make_shared<ActionTest>("actiontest4", blackboard_ptr);
    auto action5_ptr = std::make_shared<ActionTest>("actiontest5", blackboard_ptr);

    auto jud1_ptr = std::make_shared<shop::decision::PreconditionNode>("jud1_tests", blackboard_ptr,
                                                                       action1_ptr,
                                                                       [&]() {
                                                                               return false;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto jud2_ptr = std::make_shared<shop::decision::PreconditionNode>("jud2_tests", blackboard_ptr,
                                                                       action2_ptr,
                                                                       [&]() {
                                                                                return false;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto sw1_ptr = std::make_shared<shop::decision::SelectorNode>("sw1_tests", blackboard_ptr);
    sw1_ptr->AddChildren(jud2_ptr);
    sw1_ptr->AddChildren(jud1_ptr);

    auto sw1_ptr_node = std::make_shared<shop::decision::PreconditionNode>("sw1_Reevaluarion", blackboard_ptr,
                                                                           sw1_ptr,
                                                                           [&]() {
                                                                               return true;
                                                                           },
                                                                           shop::decision::AbortType::LOW_PRIORITY);

    auto jud3_ptr = std::make_shared<shop::decision::PreconditionNode>("jud3_tests", blackboard_ptr,
                                                                       action3_ptr,
                                                                       [&]() {
                                                                                return false;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto jud4_ptr = std::make_shared<shop::decision::PreconditionNode>("jud4_tests", blackboard_ptr,
                                                                       action4_ptr,
                                                                       [&]() {
                                                                           auto dir_ptr = std::dynamic_pointer_cast<TestDir>(blackboard_ptr->GetDirPtr("test1"));
                                                                           dir_ptr->Set(true);
                                                                           return dir_ptr->GetValue();
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto sw2_ptr = std::make_shared<shop::decision::SelectorNode>("sw2_tests", blackboard_ptr);
    sw2_ptr->AddChildren(jud4_ptr);
    sw2_ptr->AddChildren(jud3_ptr);
    sw2_ptr->AddChildren(action5_ptr);

    auto sw2_ptr_node = std::make_shared<shop::decision::PreconditionNode>("sw2_Reevaluarion", blackboard_ptr,
                                                                           sw2_ptr,
                                                                           [&]() {
                                                                               return true;
                                                                           },
                                                                           shop::decision::AbortType::BOTH);

    auto robot_ptr = std::make_shared<shop::decision::SelectorNode>("robot_tests", blackboard_ptr);
    robot_ptr->AddChildren(sw1_ptr_node);
    robot_ptr->AddChildren(sw2_ptr_node);

    shop::decision::BehaviorTree se(robot_ptr, 100);
    se.Execute();
}
