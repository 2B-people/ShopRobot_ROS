/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: your name
 * @Date: 2019-04-17 16:22:43
 * @LastEditTime: 2019-04-17 16:22:51
 */
/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-11 21:48:43
 * @LastEditTime: 2019-04-17 16:22:52
 */
#include <web_serial/web_server_class.h>

namespace shop
{
namespace webserver
{

WebServer::WebServer(std::string name)
    : common::RRTS(name, 3), is_open_(false), move_stop_(true), shop_stop_(true), is_debug_(false),
      client_sockfd_(0), server_sockfd_(0), server_addr_("192.168.31.100"), bind_port_(1111), go_flag_(false),
      //atcion初始化
      move_as_(nh_, ("shop/" + name + "/move_action"), boost::bind(&WebServer::MoveExecuteCB, this, _1), false),
      opening_as_(nh_, ("shop/" + name + "/opening_action"), boost::bind(&WebServer::OpeningExecuteCB, this, _1), false),
      action_as_(nh_, ("shop/" + name + "/shop_action"), boost::bind(&WebServer::ShopExecuteCB, this, _1), false)
{
    ros::NodeHandle nh_private_("~");

    is_open_ = true;
    move_stop_ = false;
    shop_stop_ = false;

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
        shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/C_shelf_barrier_wirte");
    }
    else if (name_ == "robot2")
    {
        shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/B_shelf_barrier_wirte");
    }
    else if (name_ == "robot3")
    {
        shelf_barrier_client_ = nh_.serviceClient<data::ShelfBarrier>("shop/D_shelf_barrier_wirte");
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
    Send("I");
}

WebServer::~WebServer()
{
    is_open_ = false;
    move_stop_ = true;
    shop_stop_ = true;
    // ROS_INFO("%s is destrust", name_.c_str());
    /*关闭套接字*/
    close(client_sockfd_);
    close(server_sockfd_);
}

//RUN MAIN里调用实际运行
void WebServer::Run(void)
{
    ros::AsyncSpinner async_spinner(thread_num_);
    async_spinner.start();
    ros::waitForShutdown();
    // while (ros::ok)
    // {
    //     ros::spinOnce();
    //     move_pub_.publish(now_coord_);
    // }
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

    return true;
}

//移动的回调函数
void WebServer::MoveExecuteCB(const data::MoveGoal::ConstPtr &goal)
{
    //反馈
    data::MoveFeedback feedback;
    //结果
    data::MoveResult result;

    //目标转化为string在发下去
    // coord_goal.x = goal->x;
    // coord_goal.y = goal->y;
    // coord_goal.pose = 5;

    if (goal->pose == 1)
    {

        if (cmd_coord_.x == 10 || cmd_coord_.y == 10)
        {
            ROS_INFO("%s move is err,cmd is 10",name_.c_str());
            result.success_flag = false;
            move_as_.setPreempted(result);
            return;
        }

        if (now_coord_.x == target_coord_.x && now_coord_.y == target_coord_.y)
        {
            ROS_INFO("%s move is wart for plan",name_.c_str());
            result.success_flag = false;
            move_as_.setPreempted(result);
            return;
        }

        // data::ActionName action_srv;
        // action_srv.request.action_name = target_action_.name;
        // action_srv.request.action_state = target_action_.action_state;
        // action_srv.request.is_action = true;
        // action_client_.call(action_srv);
        //发送目标
        ROS_INFO("%s is write move", name_.c_str());
        std::string coord_goal_str = CoordToData(cmd_coord_);
        Send(coord_goal_str);

        // ROS_INFO("move is write %s", coord_goal_str.c_str());

        //得到一次现在的坐标,得到进度计算的分母
        std::string re_frist_buf = Recv();
        now_coord_ = DataToCoord(re_frist_buf);
        move_pub_.publish(now_coord_);
        float progress_overall = float((target_coord_.x + target_coord_.y) - (now_coord_.x + now_coord_.y));
        while (ros::ok && is_open_ && move_stop_ == false)
        {
            std::string re_buf = Recv();

            if (is_debug_)
            {
                // ROS_INFO("RE_BUf is %s", re_buf.c_str());
            }

            now_coord_ = DataToCoord(re_buf);
            if (now_coord_.x >= 10 || now_coord_.y >= 10)
            {
                ROS_WARN("coord data is err");
                result.success_flag = false;
                move_as_.setPreempted(result);
                return;
            }
            move_pub_.publish(now_coord_);

            //判断目标坐标
            //@note 下位机可以不用频道方向
            if (now_coord_.x == cmd_coord_.x && now_coord_.y == cmd_coord_.y)
            {
                if (target_coord_.x == cmd_coord_.x && target_coord_.y == cmd_coord_.y)
                {
                    ROS_INFO("move to target,x:%d,y:%d", target_coord_.x, target_coord_.y);
                    ROS_INFO("%s FININSH", __FUNCTION__);
                    result.success_flag = true;
                    move_as_.setSucceeded(result);
                    return;
                }
                else
                {
                    ROS_INFO("%s is waitting other robot", name_.c_str());
                    std::string coord_goal_str = CoordToData(cmd_coord_);
                    Send(coord_goal_str);
                }
            }
            else
            {
                // ROS_INFO("read x:%d,y:%d,pose:%d", now_coord_.x, now_coord_.y, now_coord_.pose);
                feedback.progress = (float)((now_coord_.x + now_coord_.y) - (now_coord_.x + now_coord_.y)) / progress_overall;
                move_as_.publishFeedback(feedback);
            }
        }
    }
    else if (goal->pose == 2)
    {
        //发送目标

        //存在bug
        // data::ActionName action_srv;
        // action_srv.request.action_name = target_action_.name;
        // action_srv.request.action_state = target_action_.action_state;
        // action_srv.request.is_action = true;
        // action_client_.call(action_srv);

        std::string coord_goal_str = CoordToData(target_coord_);
        Send(coord_goal_str);

        ROS_INFO("move is write %s", coord_goal_str.c_str());

        //得到一次现在的坐标,得到进度计算的分母
        std::string re_frist_buf = Recv();
        data::Coord begin_coord = DataToCoord(re_frist_buf);
        move_pub_.publish(begin_coord);
        float progress_overall = float((target_coord_.x + target_coord_.y) - (begin_coord.x + begin_coord.y));
        //等待结束,移动到目标坐标
        while (ros::ok && is_open_ && move_stop_ == false)
        {

            std::string re_buf = Recv();

            if (is_debug_)
            {
                // ROS_INFO("RE_BUf is %s", re_buf.c_str());
            }

            // ROS_INFO("RE_BUf is %s", re_buf.c_str());

            now_coord_ = DataToCoord(re_buf);
            if (now_coord_.x == 10 && now_coord_.y == 10)
            {
                ROS_WARN("coord data is err");
            }
            move_pub_.publish(now_coord_);

            //判断目标坐标
            //@note 下位机可以不用频道方向
            if (target_coord_.x == now_coord_.x && target_coord_.y == now_coord_.y)
            {
                ROS_INFO("move to target,x:%d,y:%d", target_coord_.x, target_coord_.y);
                ROS_INFO("%s FININSH", __FUNCTION__);
                result.success_flag = true;
                move_as_.setSucceeded(result);
                return;
            }
            else
            {
                // ROS_INFO("read x:%d,y:%d,pose:%d", now_coord_.x, now_coord_.y, now_coord_.pose);
                feedback.progress = (float)((now_coord_.x + now_coord_.y) - (begin_coord.x + begin_coord.y)) / progress_overall;
                move_as_.publishFeedback(feedback);
            }
        }
    }
}

//shopatcion的回调函数
void WebServer::ShopExecuteCB(const data::ShopActionGoal::ConstPtr &goal)
{
    data::ShopActionFeedback feedback;
    data::ShopActionResult result;

    //判断目标的是否能用
    if (target_action_.action_state == 0)
    {
        ROS_WARN("action state is 0");
        result.success_flag = false;
        action_as_.setPreempted(result);
        return;
    }

    ROS_WARN("%s is write action", name_.c_str());

    // action_srv.request.action_name = target_action_.name;
    // action_srv.request.action_state = target_action_.action_state;
    // action_srv.request.is_action = true;
    // action_client_.call(action_srv);

    //发送此次的目标
    Send(target_action_.name);

    //等待结束
    while (ros::ok() && is_open_ && shop_stop_ == false)
    {

        std::string re_buf_string = Recv();

        if (re_buf_string.substr(0, 6) == "finish")
        {
            ROS_INFO("%s is finish", target_action_.name.c_str());
            break;
        }
        else if (re_buf_string[0] == 'R')
        {
            ROS_WARN("is read R");
            now_coord_ = DataToCoord(re_buf_string);
            move_pub_.publish(now_coord_);
        }
        else
        {
            feedback.progress = re_buf_string;
            action_as_.publishFeedback(feedback);
        }
    }

    //动作结束可以规划

    data::ActionName action_srv;
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
        ROS_ERROR("this is bug!");
        result.success_flag = false;
        opening_as_.setPreempted(result);
        return;
    }

    ROS_INFO("%s is write Open", name_.c_str());

    Send("go");

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
            else if (re_buf[0] == 'R')
            {
                ROS_WARN("is read R");
                now_coord_ = DataToCoord(re_buf);
                move_pub_.publish(now_coord_);
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
            else
            {
                feedback.progress = re_buf;
                opening_as_.publishFeedback(feedback);
            }
        }
    }

    go_flag_ = true;
    result.success_flag = true;
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
            std::string re_buf = Recv();
            data::Coord stop_now_coord_ = DataToCoord(re_buf);
            if (now_coord_.pose == 1 && stop_now_coord_.x == now_coord_.x + 1 && stop_now_coord_.y == now_coord_.y)
            {
                break;
            }
            else if (now_coord_.pose == 2 && stop_now_coord_.x == now_coord_.x && stop_now_coord_.y == now_coord_.y + 1)
            {
                break;
            }
            else if (now_coord_.pose == 3 && stop_now_coord_.x == now_coord_.x - 1 && stop_now_coord_.y == now_coord_.y)
            {
                break;
            }
            else if (now_coord_.pose == 4 && stop_now_coord_.x == now_coord_.x && stop_now_coord_.y == now_coord_.y - 1)
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

// TODO !!!
// @breif data到货框障碍物
// @pargm 坐标类型
// @return 货框S
data::ShelfBarrier WebServer::DataToBarrier(std::string temp)
{
    data::ShelfBarrier ruselt;
    // ruselt.location = temp[]
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
std::string WebServer::Recv(void)
{
    std::string temp;
    char re_frist_buf[BUFF_MAX];
    while (1)
    {
        memset(re_frist_buf, '\0', BUFF_MAX);
        recv(client_sockfd_, &re_frist_buf, BUFF_MAX, 0);
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
    target_coord_.pose = msg->pose;
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
