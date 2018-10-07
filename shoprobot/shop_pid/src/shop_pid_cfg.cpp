#include "shop_pid/shop_pid.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pid_cfg");
    ros::NodeHandle nh;

    ShopPID *shop_pid = new SHOPPID();

    return 0;
}