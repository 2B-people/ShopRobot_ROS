/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-11 21:48:43
 * @LastEditTime: 2019-03-28 21:05:44
 */
#include <web_serial/web_server_class.h>

namespace shop
{
namespace webserver
{

WebServer::WebServer(std::string name)
    : common::RRTS(name, 3), is_open_(false), move_stop_(true), shop_stop_(true), is_debug_(false),
      client_sockfd_(0), server_sockfd_(0), server_addr_("192.168.31.100"), bind_port_(1111),
      //atcion初始化
      move_as_(nh_, (name + "/move_action"), boost::bind(&WebServer::MoveExecuteCB, this, _1), false),
      opening_as_(nh_, (name + "/opening_action"), boost::bind(&WebServer::OpeningExecuteCB, this, _1), false),
      action_as_(nh_, (name + "/shop_action"), boost::bind(&WebServer::ShopExecuteCB, this, _1), false)
{
    ros::NodeHandle nh_private_("~");

    is_open_ = true;
    move_stop_ = false;
    shop_stop_ = false;

    now_coord_.x = 10;
    now_coord_.y = 10;
    now_coord_.pose = 10;

    memset(&my_addr_, 0, sizeof(my_addr_)); //数据初始化--清零

    ROS_INFO("%s is begin", name_.c_str());

    //param init
    nh_.param("debug", is_debug_, false);
    bool is_get_addr = nh_private_.getParam("addr", server_addr_);
    bool is_get_port = nh_private_.getParam("port", bind_port_);
    if (is_debug_)
    {
        // if (is_get_addr)
        // {
        //     ROS_INFO("%s addr cant get param", name_.c_str());
        // }
        // else
        // {
        //     ROS_INFO("%s is %d", name.c_str(), bind_port_);
        // }

        // if (is_get_port)
        // {
        //     ROS_INFO("%s port cant get param", name_.c_str());
        // }
        // else
        // {
        //     ROS_INFO("%s is %d", name.c_str(), bind_port_);
        // }
        ROS_INFO("%s %s %d", name_.c_str(), server_addr_.c_str(), bind_port_);
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

    //web init
    if (InitWeb() == false)
    {
        exit(-1);
    }

    //action 绑定抢断函数
    move_as_.registerPreemptCallback(boost::bind(&WebServer::MovePreemptCB, this));
    action_as_.registerPreemptCallback(boost::bind(&WebServer::ShopPreemptCB, this));
    opening_as_.registerPreemptCallback(boost::bind(&WebServer::OpenPreemptCB, this));
    //action open
    move_as_.start();
    action_as_.start();
    opening_as_.start();

    ROS_INFO("%s is run", name.c_str());
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

    /*创建客户端套接字--IPv4协议，面向连接通信，TCP协议*/
    if ((client_sockfd_ = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {

        ROS_ERROR("%s socket error", name_.c_str());
        return false;
    }

    /*将套接字绑定到服务器的网络地址上*/
    if (connect(client_sockfd_, (struct sockaddr *)&my_addr_, sizeof(struct sockaddr)) < 0)
    {
        ROS_ERROR("%s connect error", name_.c_str());
        return false;
    }
    ROS_WARN("is to server/n");
    Send("I");
    return true;

    //使用服务器的sockot存在问题
    // /*创建服务器端套接字--IPv4协议，面向连接通信，TCP协议*/
    // if ((server_sockfd_ = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    // {
    //     ROS_ERROR("%s socket error", name_.c_str());
    //     return false;
    // }

    // /*将套接字绑定到服务器的网络地址上*/
    // if (bind(server_sockfd_, (struct sockaddr *)&my_addr_, sizeof(struct sockaddr)) < 0)
    // {
    //     ROS_ERROR("%s bind error", name_.c_str());
    //     return false;
    // }

    // /*监听连接请求--监听队列长度为10*/
    // if (listen(server_sockfd_, 10) < 0)
    // {
    //     ROS_ERROR("%s listen error", name_.c_str());
    //     return false;
    // };

    // sin_size = sizeof(struct sockaddr_in);

    // /*等待客户端连接请求到达*/
    // //@breif accept为,要一直等待阻塞型
    // if ((client_sockfd_ = accept(server_sockfd_, (struct sockaddr *)&remote_addr_, &sin_size)) < 0)
    // {
    //     ROS_ERROR("%s accept error", name_.c_str());
    //     return false;
    // }
    // ROS_INFO("accept client %s/n", inet_ntoa(remote_addr_.sin_addr));
    // send(client_sockfd_, "server", 21, 0); //发送欢迎信息
}

//RUN MAIN里调用实际运行
void WebServer::Run()
{
    ros::AsyncSpinner async_spinner(thread_num_);
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
    coord_goal.pose = 5;
    std::string coord_goal_str = CoordToData(coord_goal);
    //bug send函数
    Send(coord_goal_str);

    // ROS_INFO("move is write %s", coord_goal_str.c_str());

    //得到一次现在的坐标,得到进度计算的分母
    std::string re_frist_buf = Recv();
    data::Coord begin_coord = DataToCoord(re_frist_buf);
    float progress_overall = float((goal->x + goal->y) - (begin_coord.x + begin_coord.y));
    //等待结束,移动到目标坐标
    while (ros::ok)
    {
        if (is_open_)
        {
            if (move_stop_ == false)
            {
                std::string re_buf = Recv();

                if (is_debug_)
                {
                    ROS_INFO("RE_BUf is %s", re_buf.c_str());
                }

                data::Coord now_coord = DataToCoord(re_buf);
                move_pub_.publish(now_coord);
                if (now_coord.x == 10 && now_coord.y == 10 && now_coord.pose == 10)
                {
                    ROS_WARN("move err");
                }

                //判断目标坐标
                //@note 下位机可以不用频道方向
                if (now_coord.x == goal->x && now_coord.y == goal->y /*&& now_coord.pose == goal->pose*/)
                {
                    ROS_INFO("move to target,x:%d,y:%d", goal->x, goal->y);
                    break;
                }
                else
                {
                    ROS_INFO("read x:%d,y:%d,pose:%d", now_coord.x, now_coord.y, now_coord.pose);
                    feedback.progress = (float)((now_coord.x + now_coord.y) - (begin_coord.x + begin_coord.y)) / progress_overall;
                    move_as_.publishFeedback(feedback);
                }
            }
            else
            {
                ROS_WARN("shop flag is stop!!!");
            }
        }
    }
    ROS_INFO("%s FININSH", __FUNCTION__);
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
    Send(goal->action_name);
    //等待结束
    while (ros::ok())
    {
        if (is_open_)
        {
            if (move_stop_ == false)
            {
                std::string re_buf_string = Recv();
                if (re_buf_string.substr(0, 6) == "finish")
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

//开局函数
void WebServer::OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal)
{
    data::OpeningFeedback feedback;
    data::OpeningResult result;
    ROS_INFO("%s is write Open", name_.c_str());

    Send(goal->car_begin);

    while (ros::ok())
    {
        if (is_open_)
        {

            std::string re_buf = Recv();
            // ROS_INFO("%s", re_buf.c_str());
            if (re_buf.substr(0, 6) == "finish")
            {
                break;
            }
            else if (re_buf[0] == 'B')
            {
                // ROS_INFO("IN HERE");
                feedback.begin_flag = true;
                opening_as_.publishFeedback(feedback);
            }
            else if (re_buf[0] == 'S')
            {
                data::ShelfBarrier srv = DataToBarrier(re_buf);
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
            else if (re_buf[0] == 'R')
            {
                data::Coord now_coord = DataToCoord(re_buf);
                move_pub_.publish(now_coord);
            }
            else
            {
                feedback.progress = re_buf;
                opening_as_.publishFeedback(feedback);
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
        Send("S");
        while (ros::ok)
        {
            std::string re_buf = Recv();
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
    std::string stop_buf("open shop now");
    if (action_as_.isActive())
    {
        Send(stop_buf);
        action_as_.setPreempted();
    }
}

//开局抢占中断回调函数
void WebServer::OpenPreemptCB()
{
    std::string stop_buf("open shop now");
    if (opening_as_.isActive())
    {
        Send(stop_buf);
        opening_as_.setPreempted();
    }
}

//------------------------------------------------------------------------//

// @breif 数据到坐标
// @pargm buf,recv读到的数据
// @return 坐标类型
data::Coord WebServer::DataToCoord(std::string buf)
{
    data::Coord rul_coord;
    if (buf[0] == 'R')
    {
        rul_coord.x = buf[1] - '0';
        rul_coord.y = buf[3] - '0';
        rul_coord.pose = buf[5] - '0';
        return rul_coord;
    }
    else
    {
        ROS_WARN("data err!is %s", buf.c_str());
        rul_coord.x = 10;
        rul_coord.y = 10;
        rul_coord.pose = 10;
        return rul_coord;
    }
}

// @breif 坐标到数据
// @pargm 坐标类型
// @return string要用send写入的
// @breif 格式:"1 2 1",
// - "x y pose"
std::string WebServer::CoordToData(data::Coord temp)
{
    std::string temp_str;
    temp_str = "R" + std::to_string(temp.x) + " " +
               std::to_string(temp.y) + " " +
               std::to_string(temp.pose);
    return temp_str;
}

// TODO
// @breif data到货框障碍物
// @pargm 坐标类型
// @return 货框

data::ShelfBarrier WebServer::DataToBarrier(std::string temp)
{
    data::ShelfBarrier ruselt;
    // ruselt.location = temp[]
}

bool WebServer::Send(std::string temp)
{
    std::string str_ = "HDU" + temp;
    ROS_INFO("%s", str_.c_str());
    send(client_sockfd_, (char *)str_.c_str(), BUFF_MAX, 0);
}

std::string WebServer::Recv(void)
{
    std::string temp;
    char re_frist_buf[BUFF_MAX];
    while (1)
    {
        memset(re_frist_buf, '\0', BUFF_MAX);
        recv(client_sockfd_, &re_frist_buf, BUFF_MAX, 0);

        // ROS_WARN("%s is recv %s", name_.c_str(), re_frist_buf);

        temp = re_frist_buf;

        if (temp.size() >= 3)
        {
            std::string jud = temp.substr(0, 3);
            temp = temp.substr(3, temp.size() - 2);
            if (jud == "HDU")
            {
                break;
            }
        }
    }
    return temp;
}

} // namespace webserver
} // namespace shop
