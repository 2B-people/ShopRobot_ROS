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
        auto opeing_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("opening_flag", opeing_flag_ptr);
        
        //game结束的flag
        auto end_game = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("end_flag", end_game);

        auto local_plan_run = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("local_plan_run", local_plan_run);
        //各类flag

        auto robot1_opening_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot1_opening_flag", robot1_opening_flag_ptr);

        auto robot2_opening_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot2_opening_flag", robot2_opening_flag_ptr);

        auto robot3_opening_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot3_opening_flag", robot3_opening_flag_ptr);

        auto robot4_opening_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot4_opening_flag", robot4_opening_flag_ptr);

        auto robot4_photo_flag_ptr = std::make_shared<BoolDir>(false);
        AddDataIntoWorld("robot4_photo_flag", robot4_photo_flag_ptr);

        // //拍照成功flag
        // auto open_done_flag_ptr = std::make_shared<BoolDir>(false);
        // AddDataIntoWorld("open_done_flag", photo_done_flag_ptr);

        // //局部规划的flag
        // auto robot1_local_plan_flag = std::make_shared<BoolDir>(false);
        // auto robot2_local_plan_flag = std::make_shared<BoolDir>(false);
        // auto robot3_local_plan_flag = std::make_shared<BoolDir>(false);
        // auto robot4_local_plan_flag = std::make_shared<BoolDir>(false);
        // AddDataIntoWorld("robot1/local_plan/flag", robot1_local_plan_flag);
        // AddDataIntoWorld("robot2/local_plan/flag", robot2_local_plan_flag);
        // AddDataIntoWorld("robot3/local_plan/flag", robot3_local_plan_flag);
        // AddDataIntoWorld("robot4/local_plan/flag", robot4_local_plan_flag);

        // auto robot1_local_plan_fuc = std::make_shared<BoolDir>(false);
        // auto robot2_local_plan_fuc = std::make_shared<BoolDir>(false);
        // auto robot3_local_plan_fuc = std::make_shared<BoolDir>(false);
        // auto robot4_local_plan_fuc = std::make_shared<BoolDir>(false);
        // AddDataIntoWorld("robot1/local_plan/fuc", robot1_local_plan_fuc);
        // AddDataIntoWorld("robot2/local_plan/fuc", robot2_local_plan_fuc);
        // AddDataIntoWorld("robot3/local_plan/fuc", robot3_local_plan_fuc);
        // AddDataIntoWorld("robot4/local_plan/fuc", robot4_local_plan_fuc);

        // //初始值为number1
        // auto photo_number = std::make_shared<PhotoNemberDir>(0);
        // AddDataIntoWorld("photo_number", photo_number);
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

  private:
};
} // namespace decision

} // namespace shop

#endif