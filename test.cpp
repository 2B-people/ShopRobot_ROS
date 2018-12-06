// C++Test.cpp : 定义控制台应用程序的入口点。
//

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>

using namespace std;
uint16_t DataToType(uint8_t *buff)
{
	uint16_t type = buff[0];
	type = type << 8;
	type |= buff[1];
	return type;
}

float DatatoInt(uint8_t *buff)
{
	int32_t data;
	data = buff[0];
	for (int i = 1; i < 4; i++)
	{
		data = data << 8;
		data |= buff[i];
	}
	return (float)data / 100;
}


void TypetoData(uint8_t len,uint8_t type,uint8_t *data_type){
  uint8_t data;
  data =type;
  data = data<<4;
  data |= len;  
  *data_type = data;
}
//main
int main(int argc, char *argv[])
{
	uint8_t hand = 0x15;
	 uint8_t i = 10;
	 uint8_t d ;

	 printf("%d\n",++i);
	 printf("%d\n",i);
	 TypetoData(5,1,&d);
	//cout << i << endl;
	printf("and: %x",d);

	return 0;
}
