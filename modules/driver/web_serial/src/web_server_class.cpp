/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: your name
 * @Date: 2019-04-17 16:22:43
 * @LastEditTime: 2019-04-17 16:22:51
 */
/*
 * @Description: In User Settings Edit
 * @Author: 2b-people
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-11 21:48:43
 * @LastEditTime: 2019-05-23 11:40:08
 */
#include <web_serial/web_server_class.h>

namespace shop
{
namespace webserver
{

WebServer::WebServer(std::string name)
    : common::RRTS(name, 3), is_open_(false), move_stop_(true), shop_stop_(true), is_debug_(false),
      client_sockfd_(0), server_sockfd_(0), server_addr_("192.168.31.100"), bind_port_(1111), go_flag_(false),
      move_flag_(false), is_recv_(0), last_recv_(0), wifi_err_(false), is_move_(false),
      //atcion初始化
      move_as_(nh_, ("shop/" + name + "/move_action"), boost::bind(&WebServer::MoveExecuteCB, this, _1), false),
      opening_as_(nh_, ("shop/" + name + "/opening_action"), boost::bind(&WebServer::OpeningExecuteCB, this, _1), false),
      action_as_(nh_, ("shop/" + name + "/shop_action"), boost::bind(&WebServer::ShopExecuteCB, this, _1), false)
{
    ros::NodeHandle nh_private_("~");

    is_open_ = true;
    move_stop_ = false;
    shop_stop_ = false;

    is_run_action_ = false;
    is_finish_ = false;
    is_begin_ = false;

    now_coord_.x = 10;
    now_coord_.y = 10;
    now_coord_.pose = 10;

    target_coord_.x = 10;
    target_coord_.y = 10;
    target_coord_.pose = 10;

    cmd_coord_.x = 10;
    cmd_coord_.y = 10;
    cmd_coord_.pose = 10;

    target_action_.name = "NONE";
    target_action_.is_action = false;
    target_action_.action_state = 0;

    failure_index_ = 0;

    memset(&my_addr_, 0, sizeof(my_addr_)); //数据初始化--清零

    ROS_INFO("%s is begin", name_.c_str());

    //param init
    nh_.param("debug", is_debug_, false);
    bool is_get_addr = nh_private_.getParam("addr", server_addr_);
    bool is_get_port = nh_private_.getParam("port", bind_port_);

    if (is_debug_)
    {
        ROS_INFO("%s %s %d", name_.c_str(), server_addr_.c_str(), bind_port_);
    }

    //publish init
    move_pub_ = nh_.advertise<data::Coord>("shop/" + name_ + "/now_coord", 1);
    move_pub_.publish(now_coord_);

    //subscribe init
    move_target_sub_ = nh_.subscribe<data::Coord>("shop/" + name_ + "/target_coord", 10, boost::bind(&WebServer::TargetCoordCB, this, _1));
    move_cmd_sub_ = nh_.subscribe<data::Coord>("shop/" + name_ + "/cmd_coord", 10, boost::bind(&WebServer::CmdCoordCB, this, _1));
    action_sub_ = nh_.subscribe<data::Action>("shop/" + name_ + "/target_action", 10, boost::bind(&WebServer::TargetActionCB, this, _1));

    //clint init
    roadblock_client_ = nh_.serviceClient<data::Roadblock>("shop/roadblock_write_srv");
    action_client_ = nh_.serviceClient<data::ActionName>("shop/" + name_ + "/target_actionname_write");

    if (name_ == "robot1")
    {
        c_shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/C_shelf_barrier_wirte");
        b_shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/B_shelf_barrier_wirte");
        d_shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/D_shelf_barrier_wirte");
    }
    else if (name_ == "robot4")
    {
    }
    //action 绑定抢断函数
    move_as_.registerPreemptCallback(boost::bind(&WebServer::MovePreemptCB, this));
    action_as_.registerPreemptCallback(boost::bind(&WebServer::ShopPreemptCB, this));
    opening_as_.registerPreemptCallback(boost::bind(&WebServer::OpenPreemptCB, this));

    //action open
    move_as_.start();
    action_as_.start();
    opening_as_.start();

    //web init
    if (InitWeb() == false)
    {
        exit(-1);
    }
    time_cb_ = nh_.createTimer(ros::Duration(1), boost::bind(&WebServer::ReInitWeb, this, _1));
    Send("I");
}

WebServer::~WebServer()
{
    is_open_ = false;
    move_stop_ = true;
    shop_stop_ = true;
    // ROS_INFO("%s is destrust", name_.c_str());
    /*关闭套接字*/
    if (read_thread_ != nullptr)
    {
        read_thread_->join();
        delete read_thread_;
    }
    close(client_sockfd_);
    close(server_sockfd_);
}

//RUN MAIN里调用实际运行
void WebServer::Run(void)
{
    ros::AsyncSpinner async_spinner(thread_num_);
    read_thread_ = new std::thread(boost::bind(&WebServer::ReceiveLoop, this));
    // async_spinner.start();
    // ros::waitForShutdown();

    while (ros::ok)
    {
        ros::spinOnce();
        if (failure_index_ == 8)
        {
            wifi_err_ = true;
            ROS_ERROR("wifi is done!");
            close(client_sockfd_);
            ros::Duration(0.5).sleep();

            if ((client_sockfd_ = accept(server_sockfd_, (struct sockaddr *)&remote_addr_, &sin_size_)) < 0)
            {
                ROS_ERROR("%s accept error", name_.c_str());
            }
            ROS_WARN("recv accept client %s/n", inet_ntoa(remote_addr_.sin_addr));
            wifi_err_ = false;
        }
    }
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

    sin_size_ = sizeof(struct sockaddr_in);

    /*等待客户端连接请求到达*/
    //@breif accept为,要一直等待阻塞型
    if ((client_sockfd_ = accept(server_sockfd_, (struct sockaddr *)&remote_addr_, &sin_size_)) < 0)
    {
        ROS_ERROR("%s accept error", name_.c_str());
        return false;
    }
    ROS_INFO("accept client %s/n", inet_ntoa(remote_addr_.sin_addr));
    //send(client_sockfd_, "Welcome to shop ros server!!", 21, 0); //发送欢迎信息
    return true;
}

void WebServer::ReInitWeb(const ros::TimerEvent &event)
{
    if (last_recv_ == is_recv_)
    {
        failure_index_++;
        //ROS_ERROR("%s in here", name_.c_str());
    }
    else
    {
        failure_index_ = 0;
        //ROS_ERROR("%s in here2", name_.c_str());
    }

    last_recv_ = is_recv_;
}

void WebServer::ReceiveLoop(void)
{
    ROS_INFO("ReceiveLoop is ready!");

    std::string re_buf_string;
    int status = 0;

    while (is_open_ && ros::ok())
    {
        if (wifi_err_ == false)
        {
            re_buf_string = Recv(&status);

            if (re_buf_string.substr(0, 6) == "finish")
            {
                ROS_INFO("is finish");
                if (is_run_action_)
                {
                    ROS_WARN("in here");
                    is_finish_ = true;
                }
            }
            else if (re_buf_string[0] == 'R')
            {
                now_coord_ = DataToCoord(re_buf_string);
                move_pub_.publish(now_coord_);
            }
            else if (re_buf_string[0] == 'B')
            {
                ROS_INFO("IN HERE B");
                is_begin_ = true;
            }
            else if (re_buf_string[0] == 'O')
            {
                ROS_WARN("IN HERE O");
                ROS_WARN("is read %s", re_buf_string.c_str());
                data::Roadblock result;
                result.request.number = re_buf_string[1] - '0';
                roadblock_client_.call(result);
            }
            else if (re_buf_string[0] == 'S')
            {
                ROS_WARN("IN HERE S");
                DataToBarrier(re_buf_string);
            }
        }
        else
        {
            while (1)
            {
                if (wifi_err_ == false)
                    break;
            }
        }
    }
}

//移动的回调函数
void WebServer::MoveExecuteCB(const data::MoveGoal::ConstPtr &goal)
{
    //反馈
    data::MoveFeedback feedback;
    //结果
    data::MoveResult result;

    is_move_ = true;
    if (goal->pose == 1)
    {

        if (cmd_coord_.x == 10 || cmd_coord_.y == 10)
        {
            ROS_INFO("%s move is err,cmd is 10", name_.c_str());
            result.success_flag = false;
            move_as_.setPreempted(result);
            is_move_ = false;
            return;
        }
        if (move_flag_ == false)
        {
            move_flag_ = true;
            if (name_ == "robot4")
            {
                ROS_INFO("robot4 first in");
                result.success_flag = true;
                move_as_.setSucceeded(result);
                is_move_ = false;
                return;
            }
        }
        if (cmd_coord_.x == now_coord_.x && cmd_coord_.y == now_coord_.y)
        {
            ROS_INFO("%s move is wart plan", name_.c_str());
            result.success_flag = false;
            move_as_.setPreempted(result);
            is_move_ = false;
            return;
        }

        if (wifi_err_ == true)
        {
            ROS_INFO("%s wifi is err", name_.c_str());
            result.success_flag = false;
            move_as_.setPreempted(result);
            is_move_ = false;
            return;
        }

        //发送目标
        ROS_INFO("%s is write move x:%d ,y:%d", name_.c_str(), cmd_coord_.x, cmd_coord_.y);
        data::Coord in_coord;
        in_coord.x = cmd_coord_.x;
        in_coord.y = cmd_coord_.y;
        in_coord.pose = 5;
        std::string coord_goal_str = CoordToData(in_coord);
        Send(coord_goal_str);

        // ROS_INFO("move is write %s", coord_goal_str.c_str());
        while (ros::ok && is_open_ && move_stop_ == false)
        {
            if (wifi_err_ == true)
            {
                ROS_INFO("%s wifi is err", name_.c_str());
                result.success_flag = false;
                move_as_.setPreempted(result);
                is_move_ = false;
                return;
            }

            if (now_coord_.x >= 10 || now_coord_.y >= 10)
            {
                ROS_WARN("coord data is err");
                result.success_flag = false;
                move_as_.setPreempted(result);
                is_move_ = false;
                return;
            }
            if (in_coord.x != cmd_coord_.x || in_coord.y != cmd_coord_.y)
            {
                in_coord.x = cmd_coord_.x;
                in_coord.y = cmd_coord_.y;
                in_coord.pose = 5;
                std::string coord_goal_str = CoordToData(in_coord);
                Send(coord_goal_str);
            }

            //判断目标坐标
            //@note 下位机可以不用频道方向
            if (now_coord_.x == in_coord.x && now_coord_.y == in_coord.y)
            {
                if (target_coord_.x == in_coord.x && target_coord_.y == in_coord.y)
                {
                    ROS_INFO("move to target,x:%d,y:%d", target_coord_.x, target_coord_.y);
                    ROS_INFO("%s FININSH", __FUNCTION__);
                    result.success_flag = true;
                    move_as_.setSucceeded(result);
                    is_move_ = false;
                    return;
                }
                else
                {
                    ROS_INFO("%s is waitting other robot", name_.c_str());
                    result.success_flag = false;
                    move_as_.setPreempted(result);
                    is_move_ = false;
                    return;
                }
            }
            else
            {
                // ROS_INFO("read x:%d,y:%d,pose:%d", now_coord_.x, now_coord_.y, now_coord_.pose);
                feedback.progress = 0.0;
                move_as_.publishFeedback(feedback);
            }
        }
    }
    else if (goal->pose == 2)
    {

        if (wifi_err_ == true)
        {
            ROS_INFO("%s wifi is err", name_.c_str());
            result.success_flag = false;
            move_as_.setPreempted(result);
            is_move_ = false;
            return;
        }

        //发送目标
        std::string coord_goal_str = CoordToData(target_coord_);
        Send(coord_goal_str);

        ROS_INFO("move is write %s", coord_goal_str.c_str());
        data::Coord last_coord;
        int index = 0;
        last_coord.x = now_coord_.x;
        last_coord.y = now_coord_.y;
        float progress_overall = 0.0;
        //等待结束,移动到目标坐标
        while (ros::ok && is_open_ && move_stop_ == false)
        {
            if (wifi_err_ == true)
            {
                ROS_INFO("%s wifi is err", name_.c_str());
                result.success_flag = false;
                move_as_.setPreempted(result);
                is_move_ = false;
                return;
            }

            // ROS_INFO("RE_BUf is %s", re_buf.c_str());
            if (now_coord_.x == 10 && now_coord_.y == 10)
            {
                ROS_WARN("coord data is err");
            }

            //判断目标坐标
            //@note 下位机可以不用频道方向
            if (target_coord_.x == now_coord_.x && target_coord_.y == now_coord_.y)
            {
                ROS_INFO("move to target,x:%d,y:%d", target_coord_.x, target_coord_.y);
                ROS_INFO("%s FININSH", __FUNCTION__);
                result.success_flag = true;
                move_as_.setSucceeded(result);
                is_move_ = false;
                return;
            }
            else
            {
                // ROS_INFO("read x:%d,y:%d,pose:%d", now_coord_.x, now_coord_.y, now_coord_.pose);
                feedback.progress = 0.0;
                move_as_.publishFeedback(feedback);
            }

            if (last_coord.x != now_coord_.x || last_coord.y != now_coord_.y)
            {
                index = 0;
                last_coord.x = now_coord_.x;
                last_coord.y = now_coord_.y;
            }
            else
            {
                index++;
            }

            if (index == 20)
            {
                Send(coord_goal_str);
                index = 0;
            }
            ros::Duration(0.1).sleep();
        }
    }
}
//shopatcion的回调函数
void WebServer::ShopExecuteCB(const data::ShopActionGoal::ConstPtr &goal)
{
    //feedback
    data::ShopActionFeedback feedback;
    //result
    data::ShopActionResult result;

    //name is none
    if (target_action_.name == "NONE")
    {
        ROS_WARN("action name is NONE");
        result.success_flag = false;
        action_as_.setPreempted(result);
        return;
    }

    //判断目标的是否能用
    if (target_action_.action_state == 0)
    {
        ROS_WARN("action state is 0");
        result.success_flag = false;
        action_as_.setPreempted(result);
        return;
    }

    if (wifi_err_ == true)
    {
        ROS_WARN("wifi is err");
        result.success_flag = false;
        action_as_.setPreempted(result);
        return;
    }

    ROS_WARN("%s is write action", name_.c_str());
    std::string temp = target_action_.name;
    ROS_WARN("target action is %s", temp.c_str());

    //动作结束可以规划
    data::ActionName action_srv;
    action_srv.request.is_action = true;
    action_srv.request.action_name = target_action_.name;
    action_srv.request.action_state = target_action_.action_state;
    action_client_.call(action_srv);

    //发送此次的目标
    Send(temp);
    is_run_action_ = true;
    uint32_t index = 0;

    //等待结束
    while (ros::ok() && is_open_ && shop_stop_ == false)
    {
        if (wifi_err_ == true)
        {
            ROS_WARN("wifi is err");
            result.success_flag = false;
            action_as_.setPreempted(result);
            return;
        }

        if (is_finish_ == true)
        {
            is_finish_ = false;
            break;
        }
        else
        {
            index++;
        }
        if (index == 150)
        {
            Send(temp);
            index = 0;
        }

        ros::Duration(0.1).sleep();
    }

    //动作结束可以规划
    //data::ActionName action_srv;
    action_srv.request.is_action = false;
    action_srv.request.action_name = target_action_.name;
    // ROS_ERROR("XXX%d", target_action_.action_state);
    switch (target_action_.action_state)
    {
    case 1:
        action_srv.request.action_state = 2;
        break;
    case 2:
        action_srv.request.action_state = 1;
        break;
    case 3:
        action_srv.request.action_state = 1;
        break;
    default:
        ROS_ERROR("error is %s", __FUNCTION__);
        break;
    }

    is_run_action_ = false;
    action_client_.call(action_srv);

    ROS_INFO("%s FININSH", __FUNCTION__);

    result.success_flag = true;
    action_as_.setSucceeded(result);
    return;
}

//开局函数
void WebServer::OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal)
{
    data::OpeningFeedback feedback;
    data::OpeningResult result;

    if (go_flag_)
    {
        // ROS_ERROR("this is bug!");
        result.success_flag = false;
        opening_as_.setPreempted(result);
        return;
    }

    ROS_INFO("%s is write Open", name_.c_str());

    Send("go");

    is_run_action_ = true;

    while (ros::ok() && is_open_)
    {
        if (is_finish_)
        {
            is_finish_ = false;
            break;
        }
        if (is_begin_)
        {
            is_begin_ = false;
            feedback.begin_flag = true;
            opening_as_.publishFeedback(feedback);
        }
    }

    is_run_action_ = false;
    result.success_flag = true;
    go_flag_ = true;
    opening_as_.setSucceeded(result);
    return;
}

//移动抢占中断回调函数
void WebServer::MovePreemptCB()
{
    if (move_as_.isActive())
    {
        Send("S");
        while (ros::ok)
        {
            if (now_coord_.pose == 1 && now_coord_.x == now_coord_.x + 1 && now_coord_.y == now_coord_.y)
            {
                break;
            }
            else if (now_coord_.pose == 2 && now_coord_.x == now_coord_.x && now_coord_.y == now_coord_.y + 1)
            {
                break;
            }
            else if (now_coord_.pose == 3 && now_coord_.x == now_coord_.x - 1 && now_coord_.y == now_coord_.y)
            {
                break;
            }
            else if (now_coord_.pose == 4 && now_coord_.x == now_coord_.x && now_coord_.y == now_coord_.y - 1)
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
    std::string stop_buf("stop");
    if (action_as_.isActive())
    {
        Send(stop_buf);
        action_as_.setPreempted();
    }
}

//开局抢占中断回调函数
void WebServer::OpenPreemptCB()
{
    std::string stop_buf("stop");
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
        rul_coord.x = now_coord_.x;
        rul_coord.y = now_coord_.x;
        rul_coord.pose = now_coord_.pose;
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

// @breif data到货框障碍物
// @pargm data
void WebServer::DataToBarrier(std::string temp)
{
    ROS_WARN("barrier is %s", temp.c_str());
    int index = temp[1] - '0';
    data::ShelfBarrier shelf;
    switch (index)
    {
    case 0:
        shelf.request.location = 1;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 0;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 3;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 2;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        d_shelf_barrier_client_.call(shelf);
        break;
    case 1:
        shelf.request.location = 5;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 4;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 7;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 6;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        d_shelf_barrier_client_.call(shelf);
        break;
    case 2:
        shelf.request.location = 9;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 8;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 11;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        d_shelf_barrier_client_.call(shelf);
        shelf.request.location = 10;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        d_shelf_barrier_client_.call(shelf);
        break;
    case 3:
        shelf.request.location = 1;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 0;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 3;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 2;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        c_shelf_barrier_client_.call(shelf);
        break;
    case 4:
        shelf.request.location = 5;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 4;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 7;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 6;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        c_shelf_barrier_client_.call(shelf);
        break;
    case 5:
        shelf.request.location = 9;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 8;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 11;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        c_shelf_barrier_client_.call(shelf);
        shelf.request.location = 10;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        c_shelf_barrier_client_.call(shelf);
        break;
    case 6:
        shelf.request.location = 1;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 0;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 3;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 2;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        b_shelf_barrier_client_.call(shelf);
        break;
    case 7:
        shelf.request.location = 5;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 4;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 7;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 6;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        b_shelf_barrier_client_.call(shelf);
        break;
    case 8:
        shelf.request.location = 9;
        shelf.request.shelf_barrier = (bool)(temp[2] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 8;
        shelf.request.shelf_barrier = (bool)(temp[3] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 11;
        shelf.request.shelf_barrier = (bool)(temp[4] - '0');
        b_shelf_barrier_client_.call(shelf);
        shelf.request.location = 10;
        shelf.request.shelf_barrier = (bool)(temp[5] - '0');
        b_shelf_barrier_client_.call(shelf);
        break;
    default:
        break;
    }
}
bool WebServer::Send(std::string temp)
{
    std::string str_ = "HDU" + temp;
    if (is_debug_)
    {
        ROS_INFO("%s", str_.c_str());
    }

    send(client_sockfd_, (char *)str_.c_str(), BUFF_MAX, 0);
}
std::string WebServer::Recv(int *status)
{
    std::string temp;
    char re_frist_buf[BUFF_MAX];
    while (1)
    {
        memset(re_frist_buf, '\0', BUFF_MAX);
        *status = recv(client_sockfd_, &re_frist_buf, BUFF_MAX, 0);
        is_recv_++;
        if (status < 0)
        {
            return "ERR";
        }
        if (is_debug_)
        {
            ROS_WARN("%s is recv %s", name_.c_str(), re_frist_buf);
        }

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

void WebServer::TargetCoordCB(const data::Coord::ConstPtr &msg)
{
    target_coord_.x = msg->x;
    target_coord_.y = msg->y;
    target_coord_.pose = 5;
}
void WebServer::CmdCoordCB(const data::Coord::ConstPtr &msg)
{
    cmd_coord_.x = msg->x;
    cmd_coord_.y = msg->y;
    cmd_coord_.pose = msg->pose;
}
void WebServer::TargetActionCB(const data::Action::ConstPtr &msg)
{
    target_action_.name = msg->name;
    target_action_.is_action = msg->is_action;
    target_action_.action_state = msg->action_state;
}

} // namespace webserver
} // namespace shop
