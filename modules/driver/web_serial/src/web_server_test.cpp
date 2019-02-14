#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <string>
#include <stdio.h>

#include <ros/ros.h>
#include <data/SerialTest.h>

#define MAXLINE 4

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "web_serial_test");
    ros::NodeHandle nh;
    std::string server_addr_;
    server_addr_ = "127.0.0.1";
    ros::Publisher pub = nh.advertise<data::SerialTest>("test_vel", 10);

    int listenfd, connfd;
    struct sockaddr_in servaddr;
    char buff[MAXLINE];
    int n;
    uint16_t abc = 1111;

    if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(server_addr_.c_str());
    servaddr.sin_port = htons(abc);

    if (bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
    {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }

    if (listen(listenfd, 10) == -1)
    {
        printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }

    printf("======waiting for client's request======\n");
    while (1)
    {
        if ((connfd = accept(listenfd, (struct sockaddr *)NULL, NULL)) == -1)
        {
            printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
            continue;
        }
        n = recv(connfd, buff, MAXLINE, 0);
        buff[n] = '\0';
        data::SerialTest msg;
        
        for(size_t i = 0; i < 8; i++)
        {
            msg.data[i] = buff[i]; 
        }
        pub.publish(msg);
        printf("recv msg from client: %s\n", buff);
        close(connfd);
    }
    close(listenfd);
    return 0;
}
