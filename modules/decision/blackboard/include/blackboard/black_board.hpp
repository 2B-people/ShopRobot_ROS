#ifndef BLACK_BOARD_H
#define BLACK_BOARD_H

#include <map>
#include <string>
#include <chrono>
#include <stdint.h>

#include <ros/ros.h>
#include <ros/time.h>

// bug
// #include <blackboard/data_structure.hpp>

namespace shop{
namespace decision{

enum class DictionaryType
{
    NONE,
    BOOL,       //布尔
    COORDINATE, //坐标
    ACTIONNAME, //动作名字
    GOODS,      //货物
    ROADBLOCK,   //路障
    GOODSHELF,   //货架
    PHOTONEMBER  //照片次数
};

//字典基类,提供接口
class DirBase : public std::enable_shared_from_this<DirBase>
{
  public:
    typedef std::shared_ptr<DirBase> Ptr;
    DirBase(DictionaryType dictionary_type)
        : dictionary_type_(dictionary_type) , flag_(true){};
    virtual ~DirBase() = default;
    //得到字典类型
    DictionaryType GetDictionaryType()
    {
        return dictionary_type_;
    }

    bool GetLock()
    {
        return flag_;
    }
    virtual void RepeatInit() = 0;
    virtual void Lock()
    {
        flag_ = false;
    }
    virtual void OpenLock()
    {
        flag_ = true;
    }

  protected:
    bool flag_;


  private:
    DictionaryType dictionary_type_;
};

//黑板基类
class Blackboard : public std::enable_shared_from_this<Blackboard>
{

  public:
    typedef std::shared_ptr<Blackboard> Ptr;
    Blackboard(){};
    virtual ~Blackboard() = default;
    unsigned int GetWorldDataSize()
    {
        return black_map_.size();
    }
    // @breif 添加字典到world_data
    void AddDataIntoWorld(std::string key, DirBase::Ptr dir_ptr)
    {
        if (black_map_.size())
        {
            black_map_.insert(std::pair<std::string, DirBase::Ptr>(key, dir_ptr));
        }
        else
        {
            auto search = black_map_.find(key);
            if (search != black_map_.end())
            {
                ROS_INFO("AddData is WARN:key: %s is in dirctionaries!!", key.c_str());
            }
            else
            {
                black_map_.insert(std::pair<std::string, DirBase::Ptr>(key, dir_ptr));
            }
        }
    }
    // @breif 清除world_data的所有数据,慎用
    void ClearWorldDataALL()
    {
        ROS_INFO("blackboard is cleared!");
        black_map_.clear();
    }
    // @breif 取得目标数据字典类的指针
    //       -注意:此方法传回来只能是基类指针,必须使用std::dynamic_pointer_cast<Temp>来转化类型
    // @param key:目标数据的key
    // @return 字典基类的指针
    //example:
    // bool GetBoolValue(std::string key)
    // {
  ros::NodeHandle nh;
    //     auto dir_ptr = GetDirPtr(key);
    //     auto bool_dir_ptr = std::dynamic_pointer_cast<BoolDir>(dir_ptr);
    //     return bool_dir_ptr->GetValue();
    // }
    DirBase::Ptr GetDirPtr(std::string key)
    {
        if (black_map_.size() == 0)
        {
            ROS_WARN("dirctionary is nothings!");
            return nullptr;
        }
        auto search = black_map_.find(key);
        if (search != black_map_.end())
        {
            return black_map_[key];
        }
        else
        {
            ROS_WARN("Can't find key:%s in dirctionary", key.c_str());
            return nullptr;
        }
    }


  protected:
    //数据结构:map容器,储存字典指针
    std::map<std::string, DirBase::Ptr> black_map_;
};

} // namespace decision
} // namespace shop

#endif
