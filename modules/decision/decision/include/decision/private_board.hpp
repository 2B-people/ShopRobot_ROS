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