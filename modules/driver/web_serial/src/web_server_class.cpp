#include <web_serial/web_server_class.h>

namespace shop
{
namespace webserver
{

WebServer::WebServer(std::string name)
    : common::RRTS(name), is_open_(false), move_stop_(true), shop_stop_(true),
      client_sockfd_(0), server_sockfd_(0), server_addr_("127.0.0.1"), bind_port_(80),
      move_as_(nh_, name + "/move_action", boost::bind(&WebServer::MoveExecuteCB, this, _1), false),
      opening_as_(nh_, name + "opening_action", boost::bind(&WebServer::OpeningExecuteCB, this, _1), false),
      action_as_(nh_, name + "/shop_action", boost::bind(&WebServer::ShopExecuteCB, this, _1), false)
{
    ros::NodeHandle nh_private_("~");

    is_open_ = true;
    move_stop_ = false;
    shop_stop_ = false;

    now_coord_.x = 0;
    now_coord_.y = 0;
    now_coord_.pose = 0;

    memset(&my_addr_, 0, sizeof(my_addr_)); //数据初始化--清零

    //param init
    nh_private_.getParam(name_ + "/addr", server_addr_);
    nh_private_.getParam(name_ + "/port", bind_port_);
    //publish init
    move_pub_ = nh_.advertise<data::Coord>(name_ + "/coord_now", 1);

    //action 绑定抢断函数
    move_as_.registerPreemptCallback(boost::bind(&WebServer::MovePreemptCB, this));
    action_as_.registerPreemptCallback(boost::bind(&WebServer::ShopPreemptCB, this));
    //action open
    move_as_.start();
    action_as_.start();
    if (name_ != "robot4_web")
    {
        opening_as_.registerPreemptCallback(boost::bind(&WebServer::OpenPreemptCB, this));
        opening_as_.start();
    }
}

WebServer::~WebServer()
{
    is_open_ = false;
    move_stop_ = true;
    shop_stop_ = true;
    ROS_INFO("%s is destrust", name_.c_str());

    close(client_sockfd_); /*关闭套接字*/
    close(server_sockfd_);
}

void WebServer::Stop()
{
    is_open_ = false;
    move_stop_ = true;
    shop_stop_ = true;
}
void WebServer::Resume()
{
    is_open_ = true;
    move_stop_ = false;
    shop_stop_ = false;
}

bool WebServer::InitWeb()
{
    socklen_t sin_size;

    my_addr_.sin_family = AF_INET;                              //设置为IP通信
    my_addr_.sin_addr.s_addr = inet_addr(server_addr_.c_str()); //服务器IP地址--允许连接到所有本地地址上
    my_addr_.sin_port = htons(bind_port_);                      //服务器端口号

    /*创建服务器端套接字--IPv4协议，面向连接通信，TCP协议*/
    if ((server_sockfd_ = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR("%s socket error", name_.c_str());
        return false;
    }

    /*将套接字绑定到服务器的网络地址上*/
    if (bind(server_sockfd_, (struct sockaddr *)&my_addr_, sizeof(struct sockaddr)) < 0)
    {
        ROS_ERROR("%s bind error", name_.c_str());
        return false;
    }

    /*监听连接请求--监听队列长度为5*/
    if (listen(server_sockfd_, 5) < 0)
    {
        ROS_ERROR("%s listen error", name_.c_str());
        return false;
    };

    sin_size = sizeof(struct sockaddr_in);

    /*等待客户端连接请求到达*/
    if ((client_sockfd_ = accept(server_sockfd_, (struct sockaddr *)&remote_addr_, &sin_size)) < 0)
    {
        ROS_ERROR("%s accept error", name_.c_str());
        return false;
    }
    ROS_INFO("accept client %s/n", inet_ntoa(remote_addr_.sin_addr));
    send(client_sockfd_, "Welcome to shop ros server!!", 21, 0); //发送欢迎信息
    return true;
}

void WebServer::MoveExecuteCB(const data::MoveGoal::ConstrPtr &goal, MOVEACTIONSERVER *as)
{
    data::MoveFeedback feedback;
    data::MoveResult result;
    ROS_INFO("%s is write move", name_.c_str());

    int16_t wirte_x = goal->x;
    int16_t write_y = goal->y;
    int8_t write_pose = goal->pose;
    //TODO:转string

    send(client_sockfd_, &goal_buf, BUFF_MAX, 0);

    char re_frist_buf[BUFF_MAX];
    recv(client_sockfd_, &re_frist_buf, BUFF_MAX, 0);
    //得到一次现在的坐标
    data::Coord begin_coord = DataToCoord(re_frist_buf);
    float progress_overall = float((goal->x + goal->y) - (begin_coord.x + begin_coord.y));
    if (progress_overall < 0)
    {
        progress_overall = -progress_overall;
    }
    while (ros::ok)
    {
        if (is_open_)
        {
            if (move_stop_ == false)
            {
                char re_buf[BUFF_MAX];
                recv(client_sockfd_, &re_buf, BUFF_MAX, 0);
                data::Coord now_coord = DataToCoord(re_buf);
                now_coord_ = DataToCoord(re_buf);

                feedback->progress = ((now_coord.x + now_coord.y) - (begin_coord.x + begin_coord.y)) / progress_overall;
                move_as_.publishFeedback(feedback);
                move_pub_.publish(now_coord);
                if (now_coord.x == goal->x && now_coord.y == goal->y && now_coord.pose == goal->pose)
                {
                    ROS_INFO("move to target,x:%d,y:%d" goal->x, goal->y);
                    break;
                }
            }
            else
            {
                ROS_WARN("move_stop is true!!!");
            }
        }
    }
    result.success_flag = true;
    move_as_.setSucceeded(result);
}

//TODO
void WebServer::ShopExecuteCB(const data::ShopActionGoal::ConstrPtr &goal, SHOPACTIONSERVER *as)
{
    data::ShopActionFeedback feedback;
    data::ShopActionResult result;
    ROS_INFO("%s is write action", name_.c_str());

    //TODO to string

    send(client_sockfd_, &goal_buf, BUFF_MAX, 0);
    while (ros::ok())
    {
        if (is_open_)
        {
            if (move_stop_ == false)
            {
            }
            else
            {
                ROS_WARN("shop_stop is true!!!");
            }
        }
    }
    result.success_flag = true;
    action_as_.setSucceeded(result);
}

void WebServer::OpeningExecuteCB(const data::)
{

    result.success_flag = true;
    opening_as_.setSucceeded(result);
}

void WebServer::MovePreemptCB()
{

    if (move_as_.isActive())
    {
        send(client_sockfd_, "stop", BUFF_MAX, 0);
        while (ros::ok)
        {
            char re_buf[BUFF_MAX];
            recv(client_sockfd_, &re_buf, BUFF_MAX, 0);
            data::Coord stop_now_coord = DataToCoord(re_buf);
            if (now_coord_.pose == 1 && stop_now_coord.x == now_coord_.x + 1 && stop_now_coord.y == now_coord_.y)
            {
                break;
            }
            else if (now_coord_.pose == 2 && stop_now_coord.x == now_coord_.x && stop_now_coord.y == now_coord_.y + 1)
            {
                break;
            }
            else if (now_coord_.pose == 3 && stop_now_coord.x == now_coord_.x - 1 && stop_now_coord.y == now_coord_.y)
            {
                break;
            }
            else if (now_coord_.pose == 4 && stop_now_coord.x == now_coord_.x && stop_now_coord.y == now_coord_.y - 1)
            {
                break;
            }
        }
        ROS_INFO("%s is stop move!", name_.c_str());
        move_as_.setPreempted();
    }
}

void WebServer::ShopPreemptCB()
{
    if (action_as_.isActive())
    {
        //TODO
        action_as_.setPreempted();
    }
}

void WebServer::OpenPreemptCB()
{
    if (opening_as_.isActive())
    {
        //TODO
        opening_as_.setPreempted();
    }
}

data::Coord WebServer::DataToCoord(const char *buf)
{
    std::string string_buf(buf);
    data::Coord rul_coord;
    rul_coord.x = string_buf[0];
    rul_coord.y = string_buf[2];
    rul_coord.pose = string_bug[4];
    return rul_coord;
}

} // namespace webserver
} // namespace shop
