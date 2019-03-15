#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>
#include <data/Coord.h>

//此文件用于debug,
namespace shop
{
namespace decision
{
class ShopActiontest : public ActionNode
{
  public:
    ShopActiontest(uint8_t robot_num, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                   const GoalAction::Ptr &goalaction_ptr)
        : ActionNode(name, blackboard_ptr),
          robot_num_(robot_num),
          goalaction_ptr_(goalaction_ptr),
          private_blackboard_ptr_(blackboard_ptr),
          goal_flag_(false)
    {
        std::string name_key = "shop/robot" + std::to_string(robot_num_) + "/target_actionname_write";
        client_ = nh_.serviceClient<data::ActionName>(name_key);
    }
    ~ShopActiontest() = default;

  private:
    virtual void OnInitialize()
    {

        std::string goal_name = "T";
        if (goal_name != "NONE")
        {
            goalaction_ptr_->SendShopGoal(robot_num_, goal_name);
            goal_flag_ = true;
        }
        ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    }
    virtual BehaviorState Update()
    {
        return goalaction_ptr_->GetShopBehaviorState(robot_num_);
    }

    virtual void OnTerminate(BehaviorState state)
    {
        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
            if (goal_flag_)
            {
                goalaction_ptr_->CancelShopGoal(robot_num_);
                goal_flag_ = false;
            }
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
            break;
        default:
            ROS_ERROR("%s is err", name_.c_str());
            return;
        }
    }
    uint8_t robot_num_;
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    bool goal_flag_;
    GoalAction::Ptr goalaction_ptr_;
    PrivateBoard::Ptr private_blackboard_ptr_;
};

} // namespace decision

} // namespace shop

using namespace shop::decision;
data::Coord robot1_coord_now_;

void Robo1CoordNowCB(const data::Coord::ConstPtr &msg)
{
    robot1_coord_now_.x = msg->x;
    robot1_coord_now_.y = msg->y;
    robot1_coord_now_.pose = msg->pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision");
    ros::NodeHandle nh;
    bool is_debug_;
    nh.param("debug", is_debug_, false);
    robot1_coord_now_.x = 0;
    robot1_coord_now_.y = 0;
    robot1_coord_now_.pose = 0;
    ros::Subscriber robot1_coordinate_now_ = nh.subscribe("robot1_web/coord_now", 10, Robo1CoordNowCB);

    auto blackboard_ptr = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr);
    int index = 0;

    //decision

    //执行action节点
    auto robot1_opening_ptr = std::make_shared<shop::decision::OpenAction>(1, "robot1 opening", blackboard_ptr, goal_action_ptr);
    auto robot2_opening_ptr = std::make_shared<shop::decision::OpenAction>(2, "robot2 opening", blackboard_ptr, goal_action_ptr);
    auto robot3_opening_ptr = std::make_shared<shop::decision::OpenAction>(3, "robot3 opening", blackboard_ptr, goal_action_ptr);
    auto robot4_opening_ptr = std::make_shared<shop::decision::OpenAction>(4, "robot4 opening", blackboard_ptr, goal_action_ptr);

    auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, "robot1 move", blackboard_ptr, goal_action_ptr);
    auto robot2_move_ptr = std::make_shared<shop::decision::MoveAction>(2, "robot2 move", blackboard_ptr, goal_action_ptr);
    auto robot3_move_ptr = std::make_shared<shop::decision::MoveAction>(3, "robot3 move", blackboard_ptr, goal_action_ptr);
    auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, "robot4 move", blackboard_ptr, goal_action_ptr);

    auto robot1_action_ptr = std::make_shared<shop::decision::ShopActiontest>(1, "robot1 shop", blackboard_ptr, goal_action_ptr);
    auto robot2_action_ptr = std::make_shared<shop::decision::ShopActiontest>(2, "robot2 shop", blackboard_ptr, goal_action_ptr);
    auto robot3_action_ptr = std::make_shared<shop::decision::ShopActiontest>(3, "robot3 shop", blackboard_ptr, goal_action_ptr);
    auto robot4_action_ptr = std::make_shared<shop::decision::ShopActiontest>(4, "robot4 shop", blackboard_ptr, goal_action_ptr);

    auto photo_ptr = std::make_shared<shop::decision::CameraAction>("photo ", blackboard_ptr, goal_action_ptr);
    auto distinguish_ptr = std::make_shared<shop::decision::DetectionAction>("distinguish", blackboard_ptr, goal_action_ptr);

    auto behavior_ptr = std::make_shared<shop::decision::SequenceNode>("test1", blackboard_ptr);
    behavior_ptr->AddChildren(robot4_move_ptr);
    behavior_ptr->AddChildren(robot4_action_ptr);
    behavior_ptr->AddChildren(photo_ptr);
    auto robot4_cycle_ptr = std::make_shared<shop::decision::CycleNode>((4), "robot cycle",
                                                                        blackboard_ptr, behavior_ptr);

    auto robot4_opening_behavior_ptr = std::make_shared<shop::decision::SequenceNode>("test", blackboard_ptr);
    robot4_opening_behavior_ptr->AddChildren(robot4_opening_ptr);
    robot4_opening_behavior_ptr->AddChildren(robot4_cycle_ptr);
    // robot4_opening_behavior_ptr->AddChildren(robot3_move_ptr);
    // robot4_opening_behavior_ptr->AddChildren(robot1_action_ptr);
    // robot4_opening_behavior_ptr->AddChildren(robot3_action_ptr);
    blackboard_ptr->SetCoordValue(3, 4, 2, 0);
    while (ros::ok)
    {
        ros::spinOnce();
        robot4_opening_behavior_ptr->Run();
        // if (robot1_coord_now_.x == 4 && robot1_coord_now_.y == 2)
        // {
        //     goal_action_ptr->CancelMoveGoal(1);
        //     ROS_INFO("in this");
        //     while (1)
        //     {
        //     }
        // }
        auto state = robot4_opening_behavior_ptr->GetBehaviorState();
        if (state == BehaviorState::SUCCESS)
        {
            // auto temp_dir_ptr = blackboard_ptr->GetDirPtr("robot1/run_coordinate");
            // auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);

            // dir_ptr->OpenLock();
            // dir_ptr->Set(7, 4, 4);
            // robot4_opening_behavior_ptr->Reset();
            ROS_INFO("success");
            while (1)
            {
            }
        }

        index++;
        // ROS_INFO("tree is run %d",index);
    }
}