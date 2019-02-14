#include <web_serial/web_server_class.h>

namespace shop{
namespace webserver{

WebServer::WebServer(std::string name,std::string addr,uint16_t bind_port) 
    : common::RRTS(name), is_open_(false),server_addr_(addr),bind_port_(bind_port);
{
    name_ = name;
    ros::NodeHandle nh_private_("~");

    is_open_ = true;

    //param init
    nh_private_.getParam("",);
    //publish init
}

WebServer::~WebServer()
{

}

} // namespace webserver
} // namespace shop
