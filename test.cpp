// C++Test.cpp : 定义控制台应用程序的入口点。
//

#include <iostream>
#include <string>
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

//main
int main(int argc, char *argv[])
{
	uint32_t bug = -3;
	cout << bug << endl;
	return 0;
}
