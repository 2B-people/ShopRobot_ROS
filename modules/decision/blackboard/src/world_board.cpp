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
    //目标坐标
    auto robot1_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0, 0);
    auto robot2_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0, 0);
    auto robot3_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0, 0);
    auto robot4_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(0, 0, 0);

    //目标动作
    auto robot1_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE");
    auto robot2_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE");
    auto robot3_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE");
    auto robot4_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE");

    //货架障碍物
    auto a_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();
    auto b_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();
    auto c_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();
    auto d_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();

    nh_.param("debug", debug_, false);
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

    // actionname
    AddDataIntoWorld("robot1_target_actionname", robot1_target_actionname_dir_ptr);
    AddDataIntoWorld("robot2_target_actionname", robot1_target_actionname_dir_ptr);
    AddDataIntoWorld("robot3_target_actionname", robot1_target_actionname_dir_ptr);
    AddDataIntoWorld("robot4_target_actionname", robot1_target_actionname_dir_ptr);
    robot1_target_actionname_write_srv_ = nh_.advertiseService("shop/robot1/target_actionname_write", &WorldBoard::TargetActionNameWriteCB1, this);
    robot2_target_actionname_write_srv_ = nh_.advertiseService("shop/robot2/target_actionname_write", &WorldBoard::TargetActionNameWriteCB2, this);
    robot3_target_actionname_write_srv_ = nh_.advertiseService("shop/robot3/target_actionname_write", &WorldBoard::TargetActionNameWriteCB3, this);
    robot4_target_actionname_write_srv_ = nh_.advertiseService("shop/robot4/target_actionname_write", &WorldBoard::TargetActionNameWriteCB4, this);
    robot1_target_actionname_read_srv_ = nh_.advertiseService("shop/robot1/target_actionname_read", &WorldBoard::TargetActioinNameReadCB1, this);
    robot2_target_actionname_read_srv_ = nh_.advertiseService("shop/robot2/target_actionname_read", &WorldBoard::TargetActioinNameReadCB2, this);
    robot3_target_actionname_read_srv_ = nh_.advertiseService("shop/robot3/target_actionname_read", &WorldBoard::TargetActioinNameReadCB3, this);
    robot4_target_actionname_read_srv_ = nh_.advertiseService("shop/robot4/target_actionname_read", &WorldBoard::TargetActioinNameReadCB4, this);

    // good shelf barrier
    AddDataIntoWorld("A_shelf_barrier", a_shelf_barrier_dir_ptr);
    AddDataIntoWorld("B_shelf_barrier", b_shelf_barrier_dir_ptr);
    AddDataIntoWorld("C_shelf_barrier", c_shelf_barrier_dir_ptr);
    AddDataIntoWorld("D_shelf_barrier", d_shelf_barrier_dir_ptr);
    a_shelf_barrier_write_srv_ = nh_.advertiseService("shop/A_shelf_barrier_wirte", &WorldBoard::AshelfWirteCB, this);
    b_shelf_barrier_write_srv_ = nh_.advertiseService("shop/B_shelf_barrier_wirte", &WorldBoard::BshelfWirteCB, this);
    c_shelf_barrier_write_srv_ = nh_.advertiseService("shop/C_shelf_barrier_wirte", &WorldBoard::CshelfWirteCB, this);
    d_shelf_barrier_write_srv_ = nh_.advertiseService("shop/D_shelf_barrier_wirte", &WorldBoard::DshelfWirteCB, this);
    a_shelf_barrier_read_srv_ = nh_.advertiseService("shop/A_shelf_barrier_read", &WorldBoard::AshelfReadCB, this);
    b_shelf_barrier_read_srv_ = nh_.advertiseService("shop/B_shelf_barrier_read", &WorldBoard::BshelfReadCB, this);
    c_shelf_barrier_read_srv_ = nh_.advertiseService("shop/C_shelf_barrier_read", &WorldBoard::CshelfReadCB, this);
    d_shelf_barrier_read_srv_ = nh_.advertiseService("shop/D_shelf_barrier_read", &WorldBoard::DshelfReadCB, this);
}

WorldBoard::~WorldBoard()
{
    black_map_.clear();
}

bool WorldBoard::GoodsWirteCB(data::Goods::Request &req, data::Goods::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_all_goods");
    auto goods_dir_ptr = std::dynamic_pointer_cast<GoodsDir>(middle_dirbase_ptr);
    if (debug_)
    {
        /* code */
    }

    ROS_INFO("goods location:%d, is %d", req.location, req.name);
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
    coordinate_dir_ptr->Set(req.x, req.y, req.pose);
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
    coordinate_dir_ptr->Set(req.x, req.y, req.pose);
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
    coordinate_dir_ptr->Set(req.x, req.y, req.pose);
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
    coordinate_dir_ptr->Set(req.x, req.y, res.pose);
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

bool WorldBoard::AshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("A_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    good_shelf_dir_ptr->OpenLock();
    good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    if (good_shelf_dir_ptr->GetLock())
    {
        res.success_flag = true;
        good_shelf_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::BshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("B_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    good_shelf_dir_ptr->OpenLock();
    good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    if (good_shelf_dir_ptr->GetLock())
    {
        res.success_flag = true;
        good_shelf_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::CshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("C_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    good_shelf_dir_ptr->OpenLock();
    good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    if (good_shelf_dir_ptr->GetLock())
    {
        res.success_flag = true;
        good_shelf_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}
bool WorldBoard::DshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("D_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    good_shelf_dir_ptr->OpenLock();
    good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    if (good_shelf_dir_ptr->GetLock())
    {
        res.success_flag = true;
        good_shelf_dir_ptr->Lock();
        return true;
    }
    else
    {
        res.success_flag = false;
        return false;
    }
}

bool WorldBoard::AshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("A_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    for (size_t x = 0; x < 6; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 0);
    }
    for (size_t x = 6; x < 12; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 1);
    }
    return true;
}

bool WorldBoard::BshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("B_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    for (size_t x = 0; x < 6; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 0);
    }
    for (size_t x = 6; x < 12; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 1);
    }
    return true;
}
bool WorldBoard::CshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("C_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    for (size_t x = 0; x < 6; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 0);
    }
    for (size_t x = 6; x < 12; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 1);
    }
    return true;
}
bool WorldBoard::DshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("D_shelf_barrier");
    auto good_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    for (size_t x = 0; x < 6; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 0);
    }
    for (size_t x = 6; x < 12; x++)
    {
        res.shelf_barrier_all[x] = good_shelf_dir_ptr->GetGoodShelfBarrier(x, 1);
    }
    return true;
}

bool WorldBoard::TargetActionNameWriteCB1(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name);
    res.success_flag;
    return true;
}

bool WorldBoard::TargetActionNameWriteCB2(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name);
    res.success_flag;
    return true;
}
bool WorldBoard::TargetActionNameWriteCB3(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name);
    res.success_flag;
    return true;
}
bool WorldBoard::TargetActionNameWriteCB4(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name);
    res.success_flag;
    return true;
}



} // namespace decision
} // namespace shop

MAIN(shop::decision::WorldBoard, "world_board")
