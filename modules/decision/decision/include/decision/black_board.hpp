#ifndef BLACK_BOARD_H
#define BLACK_BOARD_H

#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <stdint.h>

#include <ros/ros.h>
#include <ros/time.h>


namespace shop
{
namespace decision
{

enum class DictionaryType
{
    NONE,
    BOOL,
    COORDINATE,
};

class DirBase : public std::enable_shared_from_this<DirBase>
{
  public:
    typedef std::shared_ptr<DirBase> Ptr;
    DirBase(DictionaryType dictionary_type) : dictionary_type_(dictionary_type){};
    virtual ~DirBase() = default;
    //得到字典值
    DictionaryType GetDictionaryType()
    {
        return dictionary_type_;
    }

  protected:
    virtual void OnInit() = 0;

  private:
    DictionaryType dictionary_type_;
    //data
};

class BoolDir : public DirBase
{
  public:
    BoolDir(bool initial_data) : DirBase::DirBase(DictionaryType::BOOL), initial_data_(initial_data)
    {
        bool_data_ = initial_data_;
    };
    virtual ~BoolDir() = default;
    bool GetVelar()
    {
        return bool_data_;
    }

  protected:
    void Set(bool set_data)
    {
        bool_data_ = set_data;
    }
    void OnInit()
    {
        bool_data_ = initial_data_;
    }

  private:
    bool bool_data_;
    bool initial_data_;
};

class CoordinateDir : public DirBase
{
  public:
    CoordinateDir(uint16_t initial_x, uint16_t initial_y) : DirBase::DirBase(DictionaryType::COORDINATE),
                initial_x_(initial_x), initial_y_(initial_y)
    {
        x_data_ = initial_x_;
        y_data_ = initial_y_;
    };
    virtual ~CoordinateDir() = default;
    uint16_t GetCoordinateX()
    {
        return x_data_;
    }
    uint16_t GetCoordinateY()
    {
        return y_data_;
    }    

  protected:
    void Set(uint16_t set_x, uint16_t set_y)
    {
        x_data_ = set_x;
        y_data_ = set_y;
    }
    void OnInit()
    {
        x_data_ = initial_x_;
        y_data_ = initial_y_;
    }

  private:
    uint16_t initial_x_;
    uint16_t initial_y_;
    uint16_t x_data_;
    uint16_t y_data_;
};


class Blackboard : public std::enable_shared_from_this<Blackboard>
{

  public:
    typedef std::shared_ptr<Blackboard> Ptr;
    Blackboard(){};
    virtual ~Blackboard() = default;
    unsigned int GetWorldDataSize()
    {
        return world_data_map_.size();
    }
    void AddDataIntoWorld(std::string key, DirBase::Ptr dir_ptr)
    {
        auto search = world_data_map_.find(key);
        if (search != world_data_map_.end())
        {
            ROS_INFO("AddData is WARN:key: %s is in dirctionaries!!", key.c_str());
        }
        else
        {
            world_data_map_.insert(std::pair<std::string, DirBase::Ptr>(key, dir_ptr));
        }
    }
    void ClearWorldDataALL()
    {
        ROS_INFO("blackboard is cleared!");
        world_data_map_.clear();
    }
    DirBase::Ptr GetDirPtr(std::string key)
    {
        if(world_data_map_.size() == 0){
            ROS_WARN("dirctionary is nothings!");
            return nullptr;
        }
        auto search = world_data_map_.find(key);
        if (search != world_data_map_.end())
        {
            return world_data_map_[key];
        }
        else
        {
            ROS_WARN("Can't find key:%s in dirctionary",key);
            return nullptr;
        }
    }
  private:
    std::map<std::string, DirBase::Ptr> world_data_map_;
};

} // namespace decision
} // namespace shop
#endif