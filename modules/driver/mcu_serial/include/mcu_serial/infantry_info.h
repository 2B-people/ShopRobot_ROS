typedef struct
{
	uint8_t Type_L;
	uint8_t Type_H;
	uint8_t Date_1;
	uint8_t Date_2;
	uint8_t Date_3;
	uint8_t Date_4;
}Connect_Typedef;

#define MSG_vel_x         0X0001
#define MSG_vel_y         0X0002
#define MSG_vel_z         0X0003
#define MSG_distance_f    0X0004		//前距离
#define MSG_distance_b    0X0005		//后距离
#define MSG_distance_l    0X0006		//左距离
#define MSG_distance_r    0X0007		//右距离
#define MSG_remote_ch0    0x0008   //遥控器ch0
#define MSG_remote_ch1    0x0008   //遥控器ch1
#define MSG_remote_ch2    0x0009   //遥控器ch2
#define MSG_remote_ch3    0x000A   //遥控器ch3
#define MSG_remote_s1     0x000B   //遥控器s1
#define MSG_remote_s2     0x000C   //遥控器s2


void Set_Type(Connect_Typedef co,uint16_t type)
{
	uint8_t low =(uint8_t)type&0x00FF;
	uint8_t high=(uint8_t)((type>>8)&0x00FF);
	co.Type_L=low;
	co.Type_H=high;
}

void Set_Data(Connect_Typedef co,uint32_t data)
{
	uint8_t data1=(uint8_t)data&0x000000FF;
	uint8_t data2=(uint8_t)(data>>8)&0x000000FF;
	uint8_t data3=(uint8_t)(data>>16)&0x000000FF;
	uint8_t data4=(uint8_t)(data>>24)&0x000000FF;
	
	co.Date_1=data1;
	co.Date_2=data2;
	co.Date_3=data3;
	co.Date_4=data4;
}


