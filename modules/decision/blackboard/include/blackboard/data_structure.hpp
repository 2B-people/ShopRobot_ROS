#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H

#include <ros/ros.h>

#include <string.h>
#include <stdint.h>
#include <vector>
#include <array>

#include <blackboard/black_board.hpp>

namespace shop
{
namespace decision
{

//布尔类型
class BoolDir : public DirBase
{
  public:
    BoolDir(bool initial_data)
        : DirBase(DictionaryType::BOOL), initial_bool_(initial_data)
    {
        middle_bool_ = initial_bool_;
        real_bool_ = middle_bool_;
    };
    virtual ~BoolDir() = default;
    // @breif bool的字典只有一个值
    bool GetValue()
    {
        return real_bool_;
    }

    void Set(bool set_data)
    {
        middle_bool_ = set_data;
        if (flag_)
        {
            real_bool_ = middle_bool_;
        }
        else
        {
            ROS_WARN("booldir is lock");
        }
    }

    void RepeatInit()
    {
        middle_bool_ = initial_bool_;
        real_bool_ = initial_bool_;
        OpenLock();
    }

  protected:
  private:
    bool middle_bool_;
    bool initial_bool_;
    bool real_bool_;
};

class ActionNameDir : public DirBase
{
  public:
    ActionNameDir(std::string initial_name)
        : DirBase(DictionaryType::ACTIONNAME),
          initial_name_(initial_name), name_(initial_name)
    {
    }
    virtual ~ActionNameDir() = default;

    std::string GetActionName()
    {
        return name_;
    }

    void Set(std::string set_name)
    {
        if (flag_)
        {
            name_ = set_name;
        }
        else
        {
            ROS_WARN("%s is lock in %s", set_name.c_str(), __FUNCTION__);
        }
    }

    void RepeatInit()
    {
        name_ = initial_name_;
        OpenLock();
    }

  private:
    std::string name_;
    std::string initial_name_;
};

class PhotoNemberDir : public DirBase
{

  public:
    PhotoNemberDir(uint8_t number)
        : DirBase(DictionaryType::PHOTONEMBER), initial_number_(number), number_(number)
    {
    }
    ~PhotoNemberDir() = default;

    uint8_t GetPhotoNumber()
    {
        return number_;
    }

    void Set(uint8_t number)
    {
        if (flag_)
        {
            number_ = number;
        }
        else
        {
            ROS_WARN("%d is lock in %s", number, __FUNCTION__);
        }
    }

    void RepeatInit()
    {
        number_ = initial_number_;
        OpenLock();
    }

  private:
    uint8_t number_;
    uint8_t initial_number_;
};

//坐标类型
class CoordinateDir : public DirBase
{
  public:
    CoordinateDir(uint16_t initial_x, uint16_t initial_y, uint8_t initial_pose)
        : DirBase(DictionaryType::COORDINATE), initial_x_(initial_x), initial_y_(initial_y), initial_pose_(initial_pose)
    {
        middle_x_ = initial_x_;
        middle_y_ = initial_y_;
        middle_pose_ = initial_pose_;
        real_x_ = middle_x_;
        real_y_ = middle_y_;
        real_pose_ = middle_pose_;
    };
    virtual ~CoordinateDir() = default;
    uint16_t GetCoordinateX()
    {
        return real_x_;
    }
    uint16_t GetCoordinateY()
    {
        return real_y_;
    }

    uint8_t GetCoordinatePOSE()
    {
        return real_pose_;
    }
    void Set(uint16_t set_x, uint16_t set_y, uint8_t set_pose)
    {
        middle_x_ = set_x;
        middle_y_ = set_y;
        middle_pose_ = set_pose;
        if (flag_)
        {
            real_x_ = middle_x_;
            real_y_ = middle_y_;
            real_pose_ = middle_pose_;
        }
        else
        {
            ROS_WARN("coordinate is lock");
        }
    }
    void RepeatInit()
    {
        OpenLock();
        middle_x_ = initial_x_;
        middle_y_ = initial_y_;
        real_x_ = initial_x_;
        real_y_ = initial_y_;
        real_pose_ = initial_pose_;
    }

  protected:
  private:
    uint16_t initial_x_;
    uint16_t initial_y_;
    uint16_t initial_pose_;
    uint16_t middle_x_;
    uint16_t middle_y_;
    uint16_t middle_pose_;
    uint16_t real_x_;
    uint16_t real_y_;
    uint16_t real_pose_;
};

//货物类别
enum class GoodsName : int8_t
{
    NONE = 0, //空,放好或者未识别
    RED,
    BLUE,
    GREEN,
    SYY,
    YLD,
    ADG,
    XH,
    HN,
    LH,
    WQ,
    MF,
    TLS,
};

// 用一个基类来包装goods类型,可以储存数据分数,可以容错
// class GoodsKey
// {
//   public:
//     GoodsKey(/* args */);
//     ~GoodsKey();

//   private:
// };

class GoodsDir : public DirBase
{
  public:
    GoodsDir() : DirBase(DictionaryType::GOODS)
    {
        location_goods_.fill(GoodsName::NONE);
        location_goods_lock_flag_.fill(true);
    };
    virtual ~GoodsDir() = default;

    GoodsName GetLocationGoods(int location)
    {
        return location_goods_[location];
    }

    //goods数据为一个数组,所以重写了几个函数,只是改变了参数
    void Lock(int location)
    {
        location_goods_lock_flag_[location] = false;
    }
    void OpenLock(int location)
    {
        location_goods_lock_flag_[location] = true;
    }
    void Set(int location, GoodsName name)
    {
        if (location_goods_lock_flag_.at(location))
        {
            location_goods_[location] = name;
            // if (location_goods_.at(location) == GoodsName::NONE)
            // {
            // }
            // else
            // {
            //     ROS_WARN("%d location have %d", location, (int)location_goods_[location]);
            // }
        }
        else
        {
            ROS_WARN("%d location is lock", location);
        }
    }
    void RepeatInit()
    {
        location_goods_.fill(GoodsName::NONE);
        location_goods_lock_flag_.fill(true);
    }

  protected:
  private:
    std::array<GoodsName, 12> location_goods_;
    std::array<bool, 12> location_goods_lock_flag_;
};

//路障类型
class RoadblockDir : public DirBase
{
  public:
    RoadblockDir(uint8_t number) : DirBase(DictionaryType::ROADBLOCK)
    {
        roadblock_middle_number_ = number;
        roadblock_initial_number_ = number;
        roadblock_true_number_ = roadblock_middle_number_;
    }
    virtual ~RoadblockDir() = default;

    uint8_t GetRoadbockNumber()
    {
        return roadblock_true_number_;
    }
    void Set(uint8_t number_set)
    {
        roadblock_middle_number_ = number_set;
        if (flag_)
        {
            roadblock_true_number_ = roadblock_middle_number_;
        }
        else
        {
            ROS_WARN("roadbock number is lock");
        }
    }
    void RepeatInit()
    {
        roadblock_middle_number_ = roadblock_initial_number_;
        roadblock_true_number_ = roadblock_initial_number_;
        OpenLock();
    }

  protected:
  private:
    uint8_t roadblock_middle_number_;
    uint8_t roadblock_true_number_;
    uint8_t roadblock_initial_number_;
};

//bug!!
// //货架障碍
// class GoodShelfDir : public DirBase
// {
//   public:
//     GoodShelfDir() : DirBase(DictionaryType::GOODSHELF)
//     {
//         goods_shelf_barrier_.resize(6);
//         for (int i = 0; i < goods_shelf_barrier_.size(); i++)
//         {
//             goods_shelf_barrier_[i].resize(2);
//         }
//         for (int i = 0; i < goods_shelf_barrier_.size(); i++)
//         {
//             for (int j = 0; j < goods_shelf_barrier_[0].size(); j++)
//             {
//                 goods_shelf_barrier_[i][j] = false;
//             }
//         }
//     }
//     virtual ~GoodShelfDir() = default;
//     bool GetGoodShelfBarrier(uint8_t x, uint8_t y)
//     {
//         return goods_shelf_barrier_[x][y];
//     }
//     void Set(uint8_t x, uint8_t y, bool set_barrier)
//     {
//         if (flag_)
//         {
//             goods_shelf_barrier_[x][y] = set_barrier;
//         }
//         else
//         {
//             ROS_WARN("goods shelf [%d,%d] is lock", x, y);
//         }
//     }
//     void RepeatInit()
//     {
//         for (int i = 0; i < goods_shelf_barrier_.size(); i++)
//         {
//             for (int j = 0; j < goods_shelf_barrier_[0].size(); j++)
//             {
//                 goods_shelf_barrier_[i][j] = false;
//             }
//         }
//         OpenLock();
//     }

//   private:
//     std::vector<std::vector<bool>> goods_shelf_barrier_;
// };

//初始化的时候是无,
//设定为true为有货物
class GoodShelfDir : public DirBase
{
  public:
    GoodShelfDir() : DirBase(DictionaryType::GOODSHELF)
    {
        for (size_t i = 0; i < 12; i++)
        {
            goods_shelf_barrier_[i] = false;
        }
    }
    virtual ~GoodShelfDir() = default;

    bool GetGoodShelfBarrier(uint8_t loction)
    {
        return goods_shelf_barrier_[loction];
    }
    void Set(uint8_t loction, bool set_barrier)
    {
        if (flag_)
        {
            goods_shelf_barrier_[loction] = set_barrier;
        }
        else
        {
            ROS_WARN("goods shelf [%d] is lock", loction);
        }
    }
    void RepeatInit()
    {
        for (size_t i = 0; i < 12; i++)
        {
            goods_shelf_barrier_[i] = false;
        }

        OpenLock();
    }

  private:
    bool goods_shelf_barrier_[12];
};

} // namespace decision
} // namespace shop

#endif