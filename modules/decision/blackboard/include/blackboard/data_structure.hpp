#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H

#include <ros/ros.h>

#include <string.h>
#include <vector>
#include <array>

#include <blackboard/black_board.hpp>

namespace shop{
namespace decision{

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

  protected:
    void Set(bool set_data)
    {
        middle_bool_ = set_data;
        if (flag_)
        {
            real_bool_ = middle_bool_;
        }
        else
        {
            ROS_WARN(" is lock");
        }
    }

    void RepeatInit()
    {
        middle_bool_ = initial_bool_;
        real_bool_ = initial_bool_;
        OpenLock();
    }

  private:
    bool middle_bool_;
    bool initial_bool_;
    bool real_bool_;
};

//坐标类型
class CoordinateDir : public DirBase
{
  public:
    CoordinateDir(uint16_t initial_x, uint16_t initial_y)
        : DirBase(DictionaryType::COORDINATE), initial_x_(initial_x), initial_y_(initial_y)
    {
        middle_x_ = initial_x_;
        middle_y_ = initial_y_;
        real_x_ = middle_x_;
        real_y_ = middle_y_;
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

  protected:
    void Set(uint16_t set_x, uint16_t set_y)
    {
        middle_x_ = set_x;
        middle_y_ = set_y;
        if (flag_)
        {
            real_x_ = middle_x_;
            real_y_ = middle_y_;
        }
        else
        {
            ROS_WARN(" is lock");
        }
    }
    void RepeatInit()
    {
        OpenLock();
        middle_x_ = initial_x_;
        middle_y_ = initial_y_;
        real_x_ = initial_x_;
        real_y_ = initial_y_;
    }

  private:
    uint16_t initial_x_;
    uint16_t initial_y_;
    uint16_t middle_x_;
    uint16_t middle_y_;
    uint16_t real_x_;
    uint16_t real_y_;
};

//货物类别
enum class GoodsName : int
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

//TODO 用一个基类来包装goods类型,可以储存数据分数,可以容错
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

    GoodsName GetLocationGoods(const int location)
    {
        return location_goods_[location];
    }

  protected:
    //goods数据为一个数组,所以重写了几个函数,只是改变了参数
    void Lock(const int location)
    {
        location_goods_lock_flag_[location] = false;
    }
    void OpenLock(const int location)
    {
        location_goods_lock_flag_[location] = true;
    }
    void Set(const int location, GoodsName name)
    {
        if (location_goods_lock_flag_.at(location))
        {
            if (location_goods_.at(location) == GoodsName::NONE)
            {
                location_goods_[location] = name;
            }
            else
            {
                ROS_WARN("%d location have %d", location, (int)location_goods_[location]);
            }
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

  private:
    std::array<GoodsName, 12> location_goods_;
    std::array<bool, 12> location_goods_lock_flag_;
};

} // namespace decision
} // namespace shop

#endif