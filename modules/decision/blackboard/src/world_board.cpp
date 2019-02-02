#include <blackboard/world_board.h>

namespace shop
{
namespace decision
{
WorldBoard::WorldBoard(std::string name) 
        : shop::common : RRTS(name), Blackboard()
{
    auto goods_dir_ptr = std::make_shared<GoodsDir>();
    AddDataIntoWorld("shop_all_goods", goodds_dir_ptr);   
}

WorldBoard::~WorldBoard()
{
}

} // namespace decision
} // namespace shop
