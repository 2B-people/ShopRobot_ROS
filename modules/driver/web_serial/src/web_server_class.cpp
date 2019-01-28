#include <web_serial/web_server_class.h>

namespace shop
{
namespace webserver
{

WebServer::WebServer(std::string name)
    : common::RRTS(name), is_open_(false), stop_topic_(true), stop_read_(true), stop_send_(true);
{
    name_ = name;
    ros::NodeHandle nh_private_("~");

    is_open_ = true;
    stop_read_ = false;
    stop_send_ = false;
    stop_topic = false;

    send_fifo_p_.read = 0;
    send_fifo_p_.front = 0;
    send_fifo_p_.read = 0;
    send_fifo_p_.front = 0;

    for (size_t i = 0; i < FIFOMAX; i++)
    {
        send_fifo_buff_[i] = "0";
        read_fifo_buff_[i] = "0";
    }

    //param init
    nh_private_.getParam("",);
    //publish init
}

void WebServer::Run()
{
    initweb();

    read_thread_ = new std::thread(boost::bind(&ReadLoop, this));
    topic_thread_ = new std::thread(boost::bind(&SendLoop, this));
    send_thread_ = new std::thread(boost::bind(&, this));

    //TODO

    ros::spin();
}

WebServer::~WebServer()
{
    ROS_WARN_STREAM("%s is die", name_.c_str());
    if (read_thread_ != nullptr)
    {
        stop_read_ = true;
        read_thread_->join();
        delete read_thread_;
    }
    if (send_thread_ != nullptr)
    {
        stop_send_ = true;
        send_thread_->join();
        delete send_thread_;
    }
    if (topic_thread_ != nullptr)
    {
        stop_topic_ = true;
        topic_thread_->join();
        delete topic_thread_;
    }
    is_open_ = false;
}

void WebServer::Stop()
{
    stop_read_ = true;
    stop_send_ = true;
    stop_topic = true;
}

void WebServer::Resume()
{
    stop_read_ = false;
    stop_send_ = false;
    stop_topic = false;
}

void WebServer::initweb()
{
}

void WebServer::ReadLoop()
{
    ROS_INFO("Read loop is ready!");

    while (is_open_ && ros::ok())
    {
        if (!stop_read_)
        {
        }
    }
}

void WebServer::SendLoop()
{
    ROS_INFO("Send Loop is ready!");

    while (is_open_ && ros::ok())
    {
        if (!stop_send_)
        {
        }
    }
}

void WebServer::TopicLoop()
{
    ROS_INFO("Topic Loop is ready!");

    while (is_open_ && ros::ok())
    {
        if (!stop_topic_)
        {
        }
    }
}


//将消息加入队列
void WebServer::AddFifo(std::string data, uint8_t fun)
{
    if (fun == SEND)
    {
        if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front)
        {
            ROS_WARN_STREAM("Fifo is full!!");
            ros::Duration(0.05).sleep();
            if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front)
            {
                send_fifo_buff_ = data;
                send_fifo_p_.read = (send_fifo_p_.read + 1) % FIFOMAX;
            }
        }
        else
        {
            send_fifo_buff_ = data;
            send_fifo_p_.read = (send_fifo_p_.read + 1) % FIFOMAX;
        }
    }
    else if (fun == READ)
    {
        if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front)
        {
            ROS_WARN_STREAM("Fifo is full!!");
            ros::Duration(0.05).sleep();
            if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front)
            {
                read_fifo_buff_ = data;
                send_fifo_p_.read = (send_fifo_p_.read + 1) % FIFOMAX;
            }
        }
        else
        {
            read_fifo_buff_ = data;
            send_fifo_p_.read = (send_fifo_p_.read + 1) % FIFOMAX;
        }
    }
}

//取出队列的消息及删除队列内内容
void WebServer::GetReadFifo(std::string *data, uint8_t fun)
{
    if (fun == SEND)
    {
        *data = send_fifo_buff_[send_fifo_p_.front];
        send_fifo_buff_[send_fifo_p_.front] = "0";
        send_fifo_p_.front = (send_fifo_p_.front + 1) % FIFOMAX;
    }
    else if (fun == READ)
    {
        *data = read_fifo_buff_[send_fifo_p_.front];
        read_fifo_buff_[send_fifo_p_.front] = "0";
        send_fifo_p_.front = (send_fifo_p_.front + 1) % FIFOMAX;
    }
}

} // namespace webserver
} // namespace shop
