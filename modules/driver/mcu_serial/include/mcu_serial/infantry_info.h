#ifndef INFANTRY_INFO_H_
#define INFANTRY_INFO_H_

#include <stdint.h>

typedef struct {
  uint8_t type;
  int16_t data_16;
  int32_t data_32;
  uint16_t data_16_u;
  uint32_t data_32_u;
} Connect_Typedef;

enum State{ SVEL = 1, SREMOTE, SLAS };
enum NumType { INT16_T_TYPE, INT32_T_TYPE, UINT16_T_TYPE, UINT32_T_TYPE };

#define MSG_HAND_L 0x49
#define MSG_HAND_W 0x39

#define MSG_VEL_X 0x01
#define MSG_VEL_Y 0X02
#define MSG_VEL_Z 0X03

#define MSG_DISTANCE_F 0X04 //前距离
#define MSG_DISTANCE_B 0X05 //后距离
#define MSG_DISTANCE_L 0X06 //左距离
#define MSG_DISTANCE_R 0X07 //右距离
#define MSG_REMOTE_CH0 0x08 //遥控器ch0
#define MSG_REMOTE_CH1 0x08 //遥控器ch1
#define MSG_REMOTE_CH2 0x09 //遥控器ch2
#define MSG_REMOTE_CH3 0x0A //遥控器ch3
#define MSG_REMOTE_S1 0x0B  //遥控器s1
#define MSG_REMOTE_S2 0x0C  //遥控器s2G

#define MSG_RUN_STATE 0x0d //运行状态

#define MSG_COORDINATE_X 0x0e
#define MSG_COORDINATE_Y 0x0f
#define MSG_COORDINATE_ATTITUDE 0x10

#endif