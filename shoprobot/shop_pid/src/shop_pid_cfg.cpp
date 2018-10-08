#include "shop_pid/shop_pid.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pid_cfg");
    ros::NodeHandle nh;

    ShopPID *shop_pid = new ShopPID();

    dynamic_reconfigure::Server<dynamic_reconfigure::shop_pid_config> server;
    dynamic_reconfigure::Server<dynamic_reconfigure::shop_pid_config>::CallbackType f;    
    cb = boost::bind(&ShopPID::configCallback, shop_pid, _1, _2);
    server.setCallback(f);

    double p;
    double i;
    double d;
    int rate;

    ros::NodeHandle pnh("~");
    pnh.param("p",p,3);
    pnh.param("i",i,0.03);
    pnh.param("d",d,1);
    pnh.param("rate",rate,1);


    ros::Publisher pub_msg = nh.advertise<shop_pid::PID>("pid",10);

    ros::Rate r(rate);

    while (nh.ok())
    {
        shop_pid->publishMessage(&pub_message);
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}