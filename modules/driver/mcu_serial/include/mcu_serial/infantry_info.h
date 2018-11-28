#ifndef INFANTRY_INFO_H_
#define INFANTRY_INFO_H_

#include <stdint.h>

typedef struct
{
	uint8_t Type_L;
	uint8_t Type_H;
	uint8_t Date[4];
} Connect_Typedef;

#define MSG_vel_x 0X0001
#define MSG_vel_y 0X0002
#define MSG_vel_z 0X0003
#define MSG_distance_f 0X0004 //前距离
#define MSG_distance_b 0X0005 //后距离
#define MSG_distance_l 0X0006 //左距离
#define MSG_distance_r 0X0007 //右距离
#define MSG_remote_ch0 0x0008 //遥控器ch0
#define MSG_remote_ch1 0x0008 //遥控器ch1
#define MSG_remote_ch2 0x0009 //遥控器ch2
#define MSG_remote_ch3 0x000A //遥控器ch3
#define MSG_remote_s1 0x000B  //遥控器s1
#define MSG_remote_s2 0x000C  //遥控器s2


#endif