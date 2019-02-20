#include <web_serial/web_server_class.h>

namespace shop
{
namespace webserver
{

WebServer::WebServer(std::string name)
    : common::RRTS(name, 3), is_open_(false), move_stop_(true), shop_stop_(true),is_debug_(false),
      client_sockfd_(0), server_sockfd_(0), server_addr_("127.0.0.1"), bind_port_(1234),
      //atcion初始化
      move_as_(nh_, (name + "/move_action"), boost::bind(&WebServer::MoveExecuteCB, this, _1), false),
      opening_as_(nh_, (name + "/opening_action"), boost::bind(&WebServer::OpeningExecuteCB, this, _1), false),
      action_as_(nh_, (name + "/shop_action"), boost::bind(&WebServer::ShopExecuteCB, this, _1), false)
{
    ros::NodeHandle nh_private_("~");

    is_open_ = true;
    move_stop_ = false;
    shop_stop_ = false;

    now_coord_.x = 0;
    now_coord_.y = 0;
    now_coord_.pose = 0;

    memset(&my_addr_, 0, sizeof(my_addr_)); //数据初始化--清零

    ROS_INFO("%s is begin", name_.c_str());

    //param init
    nh_.getParam("debug",is_debug_);
    bool is_get_addr = nh_private_.getParam("addr", server_addr_);
    bool is_get_port = nh_private_.getParam("port", bind_port_);
    if (is_debug_)
    {
        if (is_get_addr)
        {
            ROS_ERROR("%s addr cant get param", name_.c_str());
        }
        else
        {
            ROS_INFO("%s is %d", name.c_str(), bind_port_);
        }

        if (is_get_port)
        {
            ROS_ERROR("%s port cant get param", name_.c_str());
        }
        else
        {
            ROS_INFO("%s is %d", name.c_str(), bind_port_);
        }

        ROS_WARN("%s %s %d", name_.c_str(), server_addr_.c_str(), bind_port_);
    }

    //publish init
    move_pub_ = nh_.advertise<data::Coord>(name_ + "/coord_now", 1);

    roadblock_client_ = nh_.serviceClient<data::Roadblock>("shop/roadblock_write_srv");

    if (name_ == "robot1_web")
    {
        shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/C_shelf_barrier_wirte");
    }
    else if (name_ == "robot2_web")
    {
        shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/B_shelf_barrier_wirte");
    }
    else if (name_ == "robot3_web")
    {
        shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/D_shelf_barrier_wirte");
    }

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
    /*关闭套接字*/
    close(client_sockfd_);
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

//socket 初始化
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

    /*监听连接请求--监听队列长度为10*/
    if (listen(server_sockfd_, 10) < 0)
    {
        ROS_ERROR("%s listen error", name_.c_str());
        return false;
    };

    sin_size = sizeof(struct sockaddr_in);

    /*等待客户端连接请求到达*/
    //@breif accept为,要一直等待阻塞型
    if ((client_sockfd_ = accept(server_sockfd_, (struct sockaddr *)&remote_addr_, &sin_size)) < 0)
    {
        ROS_ERROR("%s accept error", name_.c_str());
        return false;
    }
    ROS_INFO("accept client %s/n", inet_ntoa(remote_addr_.sin_addr));
    send(client_sockfd_, "Welcome to shop ros server!!", 21, 0); //发送欢迎信息
    return true;
}

//RUN MAIN里调用实际运行
void WebServer::Run()
{
    ros::AsyncSpinner async_spinner(thread_num_);
    InitWeb();
    async_spinner.start();
    ros::waitForShutdown();
}

//移动的回调函数
void WebServer::MoveExecuteCB(const data::MoveGoal::ConstPtr &goal)
{
    //反馈
    data::MoveFeedback feedback;
    //结果
    data::MoveResult result;

    ROS_INFO("%s is write move", name_.c_str());

    //目标转化为string在发下去
    data::Coord coord_goal;
    coord_goal.x = goal->x;
    coord_goal.y = goal->y;
    coord_goal.pose = goal->pose;
    std::string coord_goal_str = CoordToData(coord_goal);

    send(client_sockfd_, (char *)coord_goal_str.c_str(), BUFF_MAX, 0);

    //得到一次现在的坐标,得到进度计算的分母
    char re_frist_buf[BUFF_MAX];
    recv(client_sockfd_, &re_frist_buf, BUFF_MAX, 0);
    data::Coord begin_coord = DataToCoord(re_frist_buf);
    float progress_overall = float((goal->x + goal->y) - (begin_coord.x + begin_coord.y));
    //等待结束,移动到目标坐标
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
                //判断目标坐标
                if (now_coord.x == goal->x && now_coord.y == goal->y && now_coord.pose == goal->pose)
                {
                    ROS_INFO("move to target,x:%d,y:%d", goal->x, goal->y);
                    break;
                }
                else
                {
                    ROS_INFO("read x:%d,y:%d,pose:%d", now_coord.x, now_coord.y, now_coord.pose);
                    feedback.progress = (float)((now_coord.x + now_coord.y) - (begin_coord.x + begin_coord.y)) / progress_overall;
                    move_as_.publishFeedback(feedback);
                    move_pub_.publish(now_coord);
                }
            }
            else
            {
                ROS_WARN("shop flag is stop!!!");
            }
        }
    }
    result.success_flag = true;
    move_as_.setSucceeded(result);
}

//shopatcion的回调函数
void WebServer::ShopExecuteCB(const data::ShopActionGoal::ConstPtr &goal)
{
    data::ShopActionFeedback feedback;
    data::ShopActionResult result;
    ROS_INFO("%s is write action", name_.c_str());
    //发送指令
    send(client_sockfd_, goal->action_name.c_str(), BUFF_MAX, 0);
    //等待结束
    while (ros::ok())
    {
        if (is_open_)
        {
            if (move_stop_ == false)
            {
                char re_buf[BUFF_MAX];
                recv(client_sockfd_, &re_buf, BUFF_MAX, 0);
                std::string re_buf_string = re_buf;
                if (re_buf_string == "finish")
                {
                    ROS_INFO("%s is finish", goal->action_name.c_str());
                    break;
                }
                else
                {
                    feedback.progress = re_buf_string;
                    action_as_.publishFeedback(feedback);
                }
            }
            else
            {
                ROS_WARN("shop flag is stop!!!");
            }
        }
    }
    result.success_flag = true;
    action_as_.setSucceeded(result);
}

//开局初始化函数
void WebServer::OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal)
{
    data::OpeningFeedback feedback;
    data::OpeningResult result;
    ROS_INFO("%s is write Open", name_.c_str());

    send(client_sockfd_, goal->car_begin.c_str(), BUFF_MAX, 0);

    while (ros::ok())
    {
        if (is_open_)
        {
            if (open_stop_)
            {
                char re_buf[BUFF_MAX];
                recv(client_sockfd_, &re_buf, BUFF_MAX, 0);
                std::string re_buf_str = re_buf;
                if (re_buf_str == "finish")
                {
                    break;
                }
                else if (re_buf_str[0] == 'S')
                {
                    data::ShelfBarrier srv = DataToBarrier(re_buf_str);
                    if (shelf_barrier_client_.call(srv))
                    {
                        ROS_INFO("shop barrier is wirte!");
                    }
                    else
                    {
                        ROS_ERROR("failed to call shop barrier board!");
                    }
                    feedback.progress = "Is set barrier";
                    opening_as_.publishFeedback(feedback);
                }
                else
                {
                    feedback.progress = re_buf_str;
                    opening_as_.publishFeedback(feedback);
                }
            }
            else
            {
                ROS_WARN("shop flag is stop!!!");
            }
        }
    }

    result.success_flag = true;
    opening_as_.setSucceeded(result);
}

//移动抢占中断回调函数
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

//动作抢占中断回调函数
void WebServer::ShopPreemptCB()
{
    std::string stop_buf("shop now");
    if (action_as_.isActive())
    {
        send(client_sockfd_, stop_buf.c_str(), BUFF_MAX, 0);
        action_as_.setPreempted();
    }
}

//开局抢占中断回调函数
void WebServer::OpenPreemptCB()
{
    std::string stop_buf("shop now");
    if (opening_as_.isActive())
    {
        send(client_sockfd_, stop_buf.c_str(), BUFF_MAX, 0);
        opening_as_.setPreempted();
    }
}

//---------------------------------------------//

// @breif 数据到坐标
// @pargm buf,recv读到的数据
// @return 坐标类型
data::Coord WebServer::DataToCoord(const char *buf)
{
    std::string string_buf(buf);
    data::Coord rul_coord;
    rul_coord.x = string_buf[0];
    rul_coord.y = string_buf[2];
    rul_coord.pose = string_buf[4];
    return rul_coord;
}

// @breif 坐标到数据
// @pargm 坐标类型
// @return string要用send写入的
// @breif 格式:"1 2 1",
// - "x y pose"
std::string WebServer::CoordToData(data::Coord temp)
{
    std::string temp_str;
    temp_str = std::to_string(temp.x) + " " +
               std::to_string(temp.y) + " " +
               std::to_string(temp.pose);
    return temp_str;
}

data::ShelfBarrier WebServer::DataToBarrier(std::string temp)
{
}

} // namespace webserver
} // namespace shop
