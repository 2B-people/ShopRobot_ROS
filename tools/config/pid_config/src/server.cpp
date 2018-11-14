#include <ros/ros.h>

#include <dynameic_reconfigure/server.h>
#include <shop_pid/shopPIDdConfig.h>

double p;
double d;
double i;
bool test;
int equipment;

void callback(shop_pid::shopPIDdConfig &config, double level)
{
    p = config.p;
    i = config.i;
    d = config.d;
    test = config.test;
    equipment = config.equipment;
}

main(int argc, char const *argv[])
{
    ros::init(argc, agrv, "shop_pid");
    ros::NodeHandle nh;

    ros::Publisher pub_message = nh.advertise<msgs::PID>("pid", 10);
    

    dynamic_reconfigure::Server<> server;
    dynamic_reconfigure::Server<>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate r(rate);

    while (nh.ok())
    {

        switch (equipment)
        {
        case 0:

            break;
        case 1:

            break;
        case 2:

            break;
        default:
            break;
        }

        if (test)
        {
            if (equipment == 0)
            {
            }
            if (equipment == 0)
            {
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
