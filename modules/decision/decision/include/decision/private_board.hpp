#ifndef PRIVATE_BOARD_H
#define PRIVATE_BOARD_H

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

namespace shop
{
namespace decision
{
//行为树专用黑板,
class PrivateBoard : public Blackboard
{
  public:
    typedef std::shared_ptr<PrivateBoard> Ptr;
    PrivateBoard()
        : Blackboard()
    {
        //game结束的flag
        auto end_game = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("end_flag", end_game);

        auto local_plan_run = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("local_plan_run", local_plan_run);
        //各类flag
        auto robot1_opeing_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot1_opeing_flag", robot1_opeing_flag_ptr);

        auto robot2_opeing_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot2_opeing_flag", robot2_opeing_flag_ptr);

        auto robot3_opeing_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot3_opeing_flag", robot3_opeing_flag_ptr);

        auto robot4_opeing_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot4_opeing_flag", robot4_opeing_flag_ptr);

        //拍照成功flag
        auto photo_done_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("photo_done_flag", photo_done_flag_ptr);

        //局部规划的flag
        auto robot1_local_plan_flag = std::make_shared<BoolDir>(false);
        auto robot2_local_plan_flag = std::make_shared<BoolDir>(false);
        auto robot3_local_plan_flag = std::make_shared<BoolDir>(false);
        auto robot4_local_plan_flag = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot1/local_plan/flag", robot1_local_plan_flag);
        AddDataIntoWorld("robot2/local_plan/flag", robot2_local_plan_flag);
        AddDataIntoWorld("robot3/local_plan/flag", robot3_local_plan_flag);
        AddDataIntoWorld("robot4/local_plan/flag", robot4_local_plan_flag);

        auto robot1_local_plan_fuc = std::make_shared<BoolDir>(false);
        auto robot2_local_plan_fuc = std::make_shared<BoolDir>(false);
        auto robot3_local_plan_fuc = std::make_shared<BoolDir>(false);
        auto robot4_local_plan_fuc = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot1/local_plan/fuc", robot1_local_plan_fuc);
        AddDataIntoWorld("robot2/local_plan/fuc", robot2_local_plan_fuc);
        AddDataIntoWorld("robot3/local_plan/fuc", robot3_local_plan_fuc);
        AddDataIntoWorld("robot4/local_plan/fuc", robot4_local_plan_fuc);

        //可以在任何地方写的目标坐标
        auto robot1_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
        auto robot2_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
        auto robot3_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
        auto robot4_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
        AddDataIntoWorld("robot1/run_coordinate", robot1_run_coordinate);
        AddDataIntoWorld("robot2/run_coordinate", robot2_run_coordinate);
        AddDataIntoWorld("robot3/run_coordinate", robot3_run_coordinate);
        AddDataIntoWorld("robot4/run_coordinate", robot4_run_coordinate);

        //@breif
        //目标动作
        auto robot1_action_name = std::make_shared<ActionNameDir>("NONE");
        auto robot2_action_name = std::make_shared<ActionNameDir>("NONE");
        auto robot3_action_name = std::make_shared<ActionNameDir>("NONE");
        auto robot4_action_name = std::make_shared<ActionNameDir>("NONE");

        AddDataIntoWorld("robot1/action_name", robot1_action_name);
        AddDataIntoWorld("robot2/action_name", robot2_action_name);
        AddDataIntoWorld("robot3/action_name", robot3_action_name);
        AddDataIntoWorld("robot4/action_name", robot4_action_name);

        //初始值为number1
        auto photo_number = std::make_shared<PhotoNemberDir>(0);
        AddDataIntoWorld("photo_number", photo_number);
    }
    ~PrivateBoard() = default;

    bool GetBoolValue(std::string key)
    {
        auto dir_ptr = GetDirPtr(key);
        auto bool_dir_ptr = std::dynamic_pointer_cast<BoolDir>(dir_ptr);
        return bool_dir_ptr->GetValue();
    }

    void SetBoolValue(bool set_bool, std::string key)
    {
        auto dir_ptr = GetDirPtr(key);
        auto bool_dir_ptr = std::dynamic_pointer_cast<BoolDir>(dir_ptr);
        bool_dir_ptr->OpenLock();
        bool_dir_ptr->Set(set_bool);
    }

    void SetCoordValue(uint8_t robot_num, uint16_t set_x, uint16_t set_y, uint8_t set_pose)
    {
        std::string coordinate_key = "robot" + std::to_string(robot_num) + "/run_coordinate";
        auto temp_dir_ptr = GetDirPtr(coordinate_key);
        auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
        dir_ptr->OpenLock();
        dir_ptr->Set(set_x, set_y, set_pose);
    }

    void SetActionName(uint8_t robot_num, std::string name)
    {
        std::string action_key = "robot" + std::to_string(robot_num) + "/action_name";
        ROS_INFO("set %s", action_key.c_str());
        auto temp_dir_ptr = GetDirPtr(action_key);
        auto dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(temp_dir_ptr);
        dir_ptr->OpenLock();
        dir_ptr->Set(name);
    }

  private:
};
} // namespace decision

} // namespace shop

#endif