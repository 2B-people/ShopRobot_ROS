#include <blackboard/world_board.h>

namespace shop
{
namespace decision
{
WorldBoard::WorldBoard(std::string name)
    : shop::common::RRTS(name), Blackboard()
{
    auto goods_dir_ptr = std::make_shared<GoodsDir>();
    auto roadblock_dir_ptr = std::make_shared<RoadblockDir>(0);
    auto robot1_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0);
    auto robot2_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0);
    auto robot3_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0);
    auto robot4_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0);

    //goods
    AddDataIntoWorld("shop_all_goods", goods_dir_ptr);
    goods_write_srv_ = nh_.advertiseService("shop/goods_write_srv", &WorldBoard::GoodsWirteCB, this);
    goods_read_srv_ = nh_.advertiseService("shop/goods_read_srv", &WorldBoard::GoodsReadCB, this);

    //roadblock
    AddDataIntoWorld("shop_roadblock", roadblock_dir_ptr);
    roadblock_write_srv_ = nh_.advertiseService("shop/roadblock_write_srv", &WorldBoard::RoadblockWirteCB, this);
    roadblock_read_srv_ = nh_.advertiseService("shop/roadblock_read_srv", &WorldBoard::RoadblockReadCB, this);

    //coordinate
    AddDataIntoWorld("robot1_target_coordinate", robot1_target_coordinate_dir_ptr);
    AddDataIntoWorld("robot2_target_coordinate", robot2_target_coordinate_dir_ptr);
    AddDataIntoWorld("robot3_target_coordinate", robot3_target_coordinate_dir_ptr);
    AddDataIntoWorld("robot4_target_coordinate", robot4_target_coordinate_dir_ptr);
    target_coordinate_lock_srv_ = nh_.advertiseService("shop/robot/coordinate_srv", &WorldBoard::TargetCoordinateLockCB, this);
    robot1_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot1/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB1, this);
    robot2_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot2/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB2, this);
    robot3_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot3/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB3, this);
    robot4_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot4/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB4, this);
    robot1_target_coordinate_read_srv_ = nh_.advertiseService("shop/robot1/target_coordinate_read", &WorldBoard::TargetCoordinateReadCB1, this);
    robot2_target_coordinate_read_srv_ = nh_.advertiseService("shop/robot2/target_coordinate_read", &WorldBoard::TargetCoordinateReadCB2, this);
    robot3_target_coordinate_read_srv_ = nh_.advertiseService("shop/robot3/target_coordinate_read", &WorldBoard::TargetCoordinateReadCB3, this);
    robot4_target_coordinate_read_srv_ = nh_.advertiseService("shop/robot4/target_coordinate_read", &WorldBoard::TargetCoordinateReadCB4, this);
}

WorldBoard::~WorldBoard()
{
    black_map_.clear();
}

bool WorldBoard::GoodsWirteCB(data::Goods::Request &req, data::Goods::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_all_goods");
    auto goods_dir_ptr = std::dynamic_pointer_cast<GoodsDir>(middle_dirbase_ptr);
    goods_dir_ptr->OpenLock(req.location);
    goods_dir_ptr->Set(req.location, (GoodsName)req.name);
    goods_dir_ptr->Lock(req.location);
    if (goods_dir_ptr->GetLocationGoods(req.location) == (GoodsName)req.name)
    {
        res.success_flag = true;
        return true;
    }
    else
    {
        res.success_flag = false;
        return true;
    }
}
bool WorldBoard::GoodsReadCB(data::Goods::Request &req, data::Goods::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_all_goods");
    auto goods_dir_ptr = std::dynamic_pointer_cast<GoodsDir>(middle_dirbase_ptr);
    GoodsName goods_name = goods_dir_ptr->GetLocationGoods(req.location);
    res.success_flag = true;
    res.name = (uint8_t)goods_name;
    return true;
}

bool WorldBoard::RoadblockWirteCB(data::Roadblock::Request &req, data::Roadblock::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_roadblock");
    auto roadblock_dir_ptr = std::dynamic_pointer_cast<RoadblockDir>(middle_dirbase_ptr);
    roadblock_dir_ptr->Set(req.number);
    roadblock_dir_ptr->Lock();

    return true;
}
bool WorldBoard::RoadblockReadCB(data::Roadblock::Request &req, data::Roadblock::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_roadblock");
    auto roadblock_dir_ptr = std::dynamic_pointer_cast<RoadblockDir>(middle_dirbase_ptr);
    uint8_t number = roadblock_dir_ptr->GetRoadbockNumber();
    if (number != 0)
    {
        res.number = number;
        res.success_flag = true;
        return true;
    }
    else
    {
        ROS_WARN("we don't know roadblack");
        res.success_flag = false;
        return false;
    }
}

bool WorldBoard::TargetCoordinateWriteCB1(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    coordinate_dir_ptr->Set(req.x, req.y);
    if (coordinate_dir_ptr->GetLock())
    {
        res.success_flag = true;
        coordinate_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::TargetCoordinateWriteCB2(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot2_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    coordinate_dir_ptr->Set(req.x, req.y);
    if (coordinate_dir_ptr->GetLock())
    {
        res.success_flag = true;
        coordinate_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::TargetCoordinateWriteCB3(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot3_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    coordinate_dir_ptr->Set(req.x, req.y);
    if (coordinate_dir_ptr->GetLock())
    {
        res.success_flag = true;
        coordinate_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::TargetCoordinateWriteCB4(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot4_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    coordinate_dir_ptr->Set(req.x, req.y);
    if (coordinate_dir_ptr->GetLock())
    {
        res.success_flag = true;
        coordinate_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::TargetCoordinateReadCB1(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    res.x = coordinate_dir_ptr->GetCoordinateX();
    res.y = coordinate_dir_ptr->GetCoordinateY();
    return true;
}
bool WorldBoard::TargetCoordinateReadCB2(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot2_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    res.x = coordinate_dir_ptr->GetCoordinateX();
    res.y = coordinate_dir_ptr->GetCoordinateY();
    return true;
}
bool WorldBoard::TargetCoordinateReadCB3(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot3_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    res.x = coordinate_dir_ptr->GetCoordinateX();
    res.y = coordinate_dir_ptr->GetCoordinateY();
    return true;
}
bool WorldBoard::TargetCoordinateReadCB4(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot4_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    res.x = coordinate_dir_ptr->GetCoordinateX();
    res.y = coordinate_dir_ptr->GetCoordinateY();
    return true;
}

bool WorldBoard::TargetCoordinateLockCB(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    switch (req.number)
    {
    case 1:
        middle_dirbase_ptr = GetDirPtr("robot1_target_coordinate");
        coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
        coordinate_dir_ptr->OpenLock();
        break;
    case 2:
        middle_dirbase_ptr = GetDirPtr("robot2_target_coordinate");
        coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
        coordinate_dir_ptr->OpenLock();
        break;
    case 3:
        middle_dirbase_ptr = GetDirPtr("robot3_target_coordinate");
        coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
        coordinate_dir_ptr->OpenLock();
        break;
    case 4:
        middle_dirbase_ptr = GetDirPtr("robot4_target_coordinate");
        coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
        coordinate_dir_ptr->OpenLock();
        break;
    default:
        ROS_ERROR("NO ROBOT%d in game", req.number);
        return false;
        break;
    }
    return true;
}

} // namespace decision
} // namespace shop

MAIN(shop::decision::WorldBoard, "world_board")
