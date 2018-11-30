#ifndef INFANTRY_INFO_H_
#define INFANTRY_INFO_H_

#include <stdint.h>

typedef struct
{
	uint16_t type;
	uint32_t data;
} Connect_Typedef;

enum
{
	SVEL=1,
	SREMOTE,
	SLAS
};	

#define MSGHANDL 0x49
#define MSGHANDW 0x39

#define MSGVELX 0x01
#define MSGVELY 0X02
#define MSGVELZ 0X03

#define MSGDISTANCEF 0X04 //前距离
#define MSGDISTANCEB 0X05 //后距离
#define MSGDISTANCEL 0X06 //左距离
#define MSGDISTANCER 0X07 //右距离
#define MSGREMOTECH0 0x08 //遥控器ch0
#define MSGREMOTECH1 0x08 //遥控器ch1
#define MSGREMOTECH2 0x09 //遥控器ch2
#define MSGREMOTECH4 0x0A //遥控器ch3
#define MSGREMOTES1 0x0B  //遥控器s1
#define MSGREMOTES2 0x0C  //遥控器s2G

#define MSGRUNSTATE 0x0d  //运行状态

#define MSGCOORDINATEX 0x0e
#define MSGCOORDINATEX 0x0f
#define MSGCOORDINATEATTITUDE 0x10

#endif