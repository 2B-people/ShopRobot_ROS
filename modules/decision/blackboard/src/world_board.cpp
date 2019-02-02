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
}

WorldBoard::~WorldBoard()
{
    black_map_.clear();
}



bool WorldBoard::GoodsCB(data::Goods::Request &req, data::Goods::Response &res)
{
    
    if (true)
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

bool WorldBoard::

} // namespace decision
} // namespace shop

MAIN(shop::decision::WorldBoard, "world_board")


