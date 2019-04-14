#include <blackboard/world_board.h>

namespace shop
{
namespace decision
{
WorldBoard::WorldBoard(std::string name)
    : shop::common::RRTS(name), Blackboard()
{
    auto goods_dir_ptr = std::make_shared<GoodsDir>();
    auto roadblock_dir_ptr = std::make_shared<RoadblockDir>(1);
    //目标坐标
    auto robot1_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(10, 10, 10);
    auto robot2_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(10, 10, 10);
    auto robot3_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(10, 10, 10);
    auto robot4_target_coordinate_dir_ptr = std::make_shared<CoordinateDir>(10, 10, 10);

    //目标动作
    auto robot1_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE", 0);
    auto robot2_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE", 0);
    auto robot3_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE", 0);
    auto robot4_target_actionname_dir_ptr = std::make_shared<ActionNameDir>("NONE", 0);

    //货架障碍物
    auto a_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();
    auto b_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();
    auto c_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();
    auto d_shelf_barrier_dir_ptr = std::make_shared<GoodShelfDir>();

    auto photo_nub_ptr = std::make_shared<PhotoNemberDir>(0);

    nh_.param("debug", is_debug_, false);

    AddDataIntoWorld("photo_number", photo_nub_ptr);
    photo_number_write_srv_ = nh_.advertiseService("shop/photo_number_srv", &WorldBoard::PhotoNumWirteCB, this);
    photo_number_pub_ = nh_.advertise<data::Photo>("shop/photo_number", 1);

    //goods
    AddDataIntoWorld("shop_all_goods", goods_dir_ptr);
    goods_write_srv_ = nh_.advertiseService("shop/goods_write_srv", &WorldBoard::GoodsWirteCB, this);
    goods_pub_ = nh_.advertise<data::Cargo>("shop/goods", 1);

    //roadblock
    AddDataIntoWorld("shop_roadblock", roadblock_dir_ptr);
    roadblock_write_srv_ = nh_.advertiseService("shop/roadblock_write_srv", &WorldBoard::RoadblockWirteCB, this);
    roadblock_pub_ = nh_.advertise<data::RoadblockMsg>("shop/roadblock", 1);

    //coordinate
    AddDataIntoWorld("robot1_target_coordinate", robot1_target_coordinate_dir_ptr);
    AddDataIntoWorld("robot2_target_coordinate", robot2_target_coordinate_dir_ptr);
    AddDataIntoWorld("robot3_target_coordinate", robot3_target_coordinate_dir_ptr);
    AddDataIntoWorld("robot4_target_coordinate", robot4_target_coordinate_dir_ptr);
    robot1_target_coord_pub_ = nh_.advertise<data::Coord>("shop/robot1/target_coord", 1);
    robot2_target_coord_pub_ = nh_.advertise<data::Coord>("shop/robot2/target_coord", 1);
    robot3_target_coord_pub_ = nh_.advertise<data::Coord>("shop/robot3/target_coord", 1);
    robot4_target_coord_pub_ = nh_.advertise<data::Coord>("shop/robot4/target_coord", 1);

    target_coordinate_lock_srv_ = nh_.advertiseService("shop/robot/coordinate_srv", &WorldBoard::TargetCoordinateLockCB, this);
    robot1_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot1/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB1, this);
    robot2_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot2/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB2, this);
    robot3_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot3/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB3, this);
    robot4_target_coordinate_write_srv_ = nh_.advertiseService("shop/robot4/target_coordinate_write", &WorldBoard::TargetCoordinateWriteCB4, this);

    // actionname
    AddDataIntoWorld("robot1_target_actionname", robot1_target_actionname_dir_ptr);
    AddDataIntoWorld("robot2_target_actionname", robot2_target_actionname_dir_ptr);
    AddDataIntoWorld("robot3_target_actionname", robot3_target_actionname_dir_ptr);
    AddDataIntoWorld("robot4_target_actionname", robot4_target_actionname_dir_ptr);
    robot1_target_actionname_write_srv_ = nh_.advertiseService("shop/robot1/target_actionname_write", &WorldBoard::TargetActionNameWriteCB1, this);
    robot2_target_actionname_write_srv_ = nh_.advertiseService("shop/robot2/target_actionname_write", &WorldBoard::TargetActionNameWriteCB2, this);
    robot3_target_actionname_write_srv_ = nh_.advertiseService("shop/robot3/target_actionname_write", &WorldBoard::TargetActionNameWriteCB3, this);
    robot4_target_actionname_write_srv_ = nh_.advertiseService("shop/robot4/target_actionname_write", &WorldBoard::TargetActionNameWriteCB4, this);
    robot1_target_action_pub_ = nh_.advertise<data::Action>("shop/robot1/target_action", 1);
    robot2_target_action_pub_ = nh_.advertise<data::Action>("shop/robot2/target_action", 1);
    robot3_target_action_pub_ = nh_.advertise<data::Action>("shop/robot3/target_action", 1);
    robot4_target_action_pub_ = nh_.advertise<data::Action>("shop/robot4/target_action", 1);

    // good shelf barrier
    AddDataIntoWorld("A_shelf_barrier", a_shelf_barrier_dir_ptr);
    AddDataIntoWorld("B_shelf_barrier", b_shelf_barrier_dir_ptr);
    AddDataIntoWorld("C_shelf_barrier", c_shelf_barrier_dir_ptr);
    AddDataIntoWorld("D_shelf_barrier", d_shelf_barrier_dir_ptr);
    a_shelf_barrier_pub_ = nh_.advertise<data::Barrier>("shop/a_barrier", 1);
    b_shelf_barrier_pub_ = nh_.advertise<data::Barrier>("shop/b_barrier", 1);
    c_shelf_barrier_pub_ = nh_.advertise<data::Barrier>("shop/c_barrier", 1);
    d_shelf_barrier_pub_ = nh_.advertise<data::Barrier>("shop/d_barrier", 1);
    a_shelf_barrier_write_srv_ = nh_.advertiseService("shop/A_shelf_barrier_wirte", &WorldBoard::AshelfWirteCB, this);
    b_shelf_barrier_write_srv_ = nh_.advertiseService("shop/B_shelf_barrier_wirte", &WorldBoard::BshelfWirteCB, this);
    c_shelf_barrier_write_srv_ = nh_.advertiseService("shop/C_shelf_barrier_wirte", &WorldBoard::CshelfWirteCB, this);
    d_shelf_barrier_write_srv_ = nh_.advertiseService("shop/D_shelf_barrier_wirte", &WorldBoard::DshelfWirteCB, this);
}

WorldBoard::~WorldBoard()
{
    black_map_.clear();
}

void WorldBoard::Run()
{

    auto middle_dirbase_ptr = GetDirPtr("shop_all_goods");
    auto goods_dir_ptr = std::dynamic_pointer_cast<GoodsDir>(middle_dirbase_ptr);

    middle_dirbase_ptr = GetDirPtr("shop_roadblock");
    auto roadblock_dir_ptr = std::dynamic_pointer_cast<RoadblockDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot1_target_coordinate");
    auto robot1_coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot2_target_coordinate");
    auto robot2_coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot3_target_coordinate");
    auto robot3_coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot4_target_coordinate");
    auto robot4_coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);

    middle_dirbase_ptr = GetDirPtr("robot1_target_actionname");
    auto robot1_action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot2_target_actionname");
    auto robot2_action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot3_target_actionname");
    auto robot3_action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("robot4_target_actionname");
    auto robot4_action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);

    middle_dirbase_ptr = GetDirPtr("A_shelf_barrier");
    auto a_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("B_shelf_barrier");
    auto b_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("C_shelf_barrier");
    auto c_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);
    middle_dirbase_ptr = GetDirPtr("D_shelf_barrier");
    auto d_shelf_dir_ptr = std::dynamic_pointer_cast<GoodShelfDir>(middle_dirbase_ptr);

    middle_dirbase_ptr = GetDirPtr("photo_number");
    auto photo_number_dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(middle_dirbase_ptr);

    data::Cargo goods;
    data::RoadblockMsg roadblock;
    data::Coord coord;
    data::Action action;
    data::Barrier barrier;
    data::Photo photo;

    while (ros::ok)
    {
        ros::spinOnce();
        // 广播goods
        for (int i = 0; i < 12; i++)
        {
            goods.goods_loction[i] = (int8_t)goods_dir_ptr->GetLocationGoods(i);
        }
        goods_pub_.publish(goods);

        // 广播roadblock
        roadblock.location_place = roadblock_dir_ptr->GetRoadbockNumber();
        roadblock_pub_.publish(roadblock);

        coord.pose = 5;
        //广播coord
        coord.x = robot4_coordinate_dir_ptr->GetCoordinateX();
        coord.y = robot4_coordinate_dir_ptr->GetCoordinateY();
        robot4_target_coord_pub_.publish(coord);

        coord.x = robot1_coordinate_dir_ptr->GetCoordinateX();
        coord.y = robot1_coordinate_dir_ptr->GetCoordinateY();
        robot1_target_coord_pub_.publish(coord);

        coord.x = robot2_coordinate_dir_ptr->GetCoordinateX();
        coord.y = robot2_coordinate_dir_ptr->GetCoordinateY();
        robot2_target_coord_pub_.publish(coord);

        coord.x = robot3_coordinate_dir_ptr->GetCoordinateX();
        coord.y = robot3_coordinate_dir_ptr->GetCoordinateY();
        robot3_target_coord_pub_.publish(coord);


        // 广播action
        action.name = robot1_action_dir_ptr->GetActionName();
        action.action_state = robot1_action_dir_ptr->GetActionState();
        action.is_action = robot1_action_dir_ptr->GetIsAction();
        robot1_target_action_pub_.publish(action);

        action.name = robot2_action_dir_ptr->GetActionName();
        action.action_state = robot2_action_dir_ptr->GetActionState();
        action.is_action = robot2_action_dir_ptr->GetIsAction();
        robot2_target_action_pub_.publish(action);

        action.name = robot3_action_dir_ptr->GetActionName();
        action.action_state = robot3_action_dir_ptr->GetActionState();
        action.is_action = robot3_action_dir_ptr->GetIsAction();
        robot3_target_action_pub_.publish(action);

        action.name = robot4_action_dir_ptr->GetActionName();
        action.action_state = robot4_action_dir_ptr->GetActionState();
        action.is_action = robot4_action_dir_ptr->GetIsAction();
        robot4_target_action_pub_.publish(action);

        photo.photo = photo_number_dir_ptr->GetPhotoNumber();
        photo.is_discern = photo_number_dir_ptr->GetDiscern();
        photo_number_pub_.publish(photo);

        //广播
        for (int i = 0; i < 12; i++)
        {
            barrier.shelf_barrier_all[i] = a_shelf_dir_ptr->GetGoodShelfBarrier(i);
        }
        a_shelf_barrier_pub_.publish(barrier);
        for (int i = 0; i < 12; i++)
        {
            barrier.shelf_barrier_all[i] = b_shelf_dir_ptr->GetGoodShelfBarrier(i);
        }
        b_shelf_barrier_pub_.publish(barrier);
        for (int i = 0; i < 12; i++)
        {
            barrier.shelf_barrier_all[i] = c_shelf_dir_ptr->GetGoodShelfBarrier(i);
        }
        c_shelf_barrier_pub_.publish(barrier);
        for (int i = 0; i < 12; i++)
        {
            barrier.shelf_barrier_all[i] = d_shelf_dir_ptr->GetGoodShelfBarrier(i);
        }
        d_shelf_barrier_pub_.publish(barrier);
    }
}

bool WorldBoard::PhotoNumWirteCB(data::PhotoSrv::Request &req, data::PhotoSrv::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("photo_number");
    auto dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(middle_dirbase_ptr);
    dir_ptr->OpenLock();
    dir_ptr->Set(req.photo, req.is_discern);
    return true;
}
bool WorldBoard::GoodsWirteCB(data::Goods::Request &req, data::Goods::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_all_goods");
    auto goods_dir_ptr = std::dynamic_pointer_cast<GoodsDir>(middle_dirbase_ptr);
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

bool WorldBoard::RoadblockWirteCB(data::Roadblock::Request &req, data::Roadblock::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("shop_roadblock");
    auto roadblock_dir_ptr = std::dynamic_pointer_cast<RoadblockDir>(middle_dirbase_ptr);
    roadblock_dir_ptr->Set(req.number);
    roadblock_dir_ptr->Lock();

    return true;
}

bool WorldBoard::TargetCoordinateWriteCB1(data::Coordinate::Request &req, data::Coordinate::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_coordinate");
    auto coordinate_dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(middle_dirbase_ptr);
    coordinate_dir_ptr->OpenLock();
    coordinate_dir_ptr->Set(req.x, req.y, req.pose);
    ROS_INFO("robot1 board set x:%d y:%d pose:%d", req.x, req.y, req.pose);
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
    coordinate_dir_ptr->OpenLock();
    coordinate_dir_ptr->Set(req.x, req.y, req.pose);
    ROS_INFO("robot2 board set x:%d y:%d pose:%d", req.x, req.y, req.pose);
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
    coordinate_dir_ptr->OpenLock();
    coordinate_dir_ptr->Set(req.x, req.y, req.pose);
    ROS_INFO("robot3 board set x:%d y:%d pose:%d", req.x, req.y, req.pose);
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
    coordinate_dir_ptr->OpenLock();
    coordinate_dir_ptr->Set(req.x, req.y, res.pose);
    ROS_INFO("robot4 board set x:%d y:%d pose:%d", req.x, req.y, req.pose);
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
    if (is_debug_)
    {
        ROS_WARN("location %d is set %d", req.location, req.shelf_barrier);
    }
    // good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    good_shelf_dir_ptr->Set(req.location, req.shelf_barrier);
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
    if (is_debug_)
    {
        ROS_WARN("location %d is set %d", req.location, req.shelf_barrier);
    }

    // good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    good_shelf_dir_ptr->Set(req.location, req.shelf_barrier);
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
    if (is_debug_)
    {
        ROS_WARN("location %d is set %d", req.location, req.shelf_barrier);
    }
    good_shelf_dir_ptr->Set(req.location, req.shelf_barrier);
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
    if (is_debug_)
    {
        ROS_WARN("location %d is set %d", req.location, req.shelf_barrier);
    }
    // good_shelf_dir_ptr->Set(req.x, req.y, req.shelf_barrier);
    good_shelf_dir_ptr->Set(req.location, req.shelf_barrier);
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

bool WorldBoard::TargetActionNameWriteCB1(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot1_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name, req.action_state,req.is_action);
    ROS_INFO("robot1 target action is %s", req.action_name.c_str());
    res.success_flag;
    return true;
}

bool WorldBoard::TargetActionNameWriteCB2(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot2_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name, req.action_state,req.is_action);
    ROS_INFO("robot2 target action is %s state is %d", req.action_name.c_str(), req.action_state);
    res.success_flag;
    return true;
}
bool WorldBoard::TargetActionNameWriteCB3(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot3_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name, req.action_state,req.is_action);
    ROS_INFO("robot3 target action is %s", req.action_name.c_str());
    res.success_flag;
    return true;
}
bool WorldBoard::TargetActionNameWriteCB4(data::ActionName::Request &req, data::ActionName::Response &res)
{
    auto middle_dirbase_ptr = GetDirPtr("robot4_target_actionname");
    auto action_dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(middle_dirbase_ptr);
    action_dir_ptr->OpenLock();
    action_dir_ptr->Set(req.action_name, req.action_state,req.is_action);
    ROS_INFO("robot4 target action is %s", req.action_name.c_str());
    res.success_flag;
    return true;
}

} // namespace decision
} // namespace shop

MAIN(shop::decision::WorldBoard, "world_board")
