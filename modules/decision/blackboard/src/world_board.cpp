#include <blackboard/world_board.h>

namespace shop
{
namespace decision
{
WorldBoard::WorldBoard(std::string name)
    : shop::common::RRTS(name), Blackboard()
{
    auto goods_dir_ptr = std::make_shared<GoodsDir>();
    AddDataIntoWorld("shop_all_goods", goods_dir_ptr);
    goods_srv_ = nh_.advertiseService("goods_srv", &WorldBoard::GoodsCB, this);
    coordinate_target_1_pub_ = nh_.advertise<data::Coordinate>("robot1/target_coordinate", 30);
    coordinate_target_2_pub_ = nh_.advertise<data::Coordinate>("robot2/target_coordinate", 30);
    coordinate_target_3_pub_ = nh_.advertise<data::Coordinate>("robot3/target_coordinate", 30);
    coordinate_target_4_pub_ = nh_.advertise<data::Coordinate>("robot4/target_coordinate", 30);


}

WorldBoard::~WorldBoard()
{
    black_map_.clear();
}

WorldBoard::Run()
{
    ros::spin();
}

bool WorldBoard::GoodsCB(data::Goods::Request &req, data::Goods::Response &res)
{
    auto goods_dirbase_ptr = GetDirPtr("shop_all_goods");
    auto goods_dir_ptr = std::dynamic_pointer_cast<GoodsDir>(goods_dirbase_ptr);
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

} // namespace decision
} // namespace shop

MAIN(shop::decision::WorldBoard, "world_board")
