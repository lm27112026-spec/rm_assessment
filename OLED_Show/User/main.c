#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"

uint8_t KeyNum;


int main(void)
{
	OLED_Init();	
	OLED_Clear();
	Key_Init();
	Serial_Init();		
	
	while (1)
	{
		if (Serial_GetRxFlag() == 1)
		{
			OLED_ShowString(1, 1, "                ");
			const uint16_t x = (uint16_t)(Serial_RxPacket[0] | (Serial_RxPacket[1] << 8));
			const uint16_t y = (uint16_t)(Serial_RxPacket[2] | (Serial_RxPacket[3] << 8));
			OLED_ShowString(1, 1, "X:");
			OLED_ShowNum(1, 3, x, 4);
			OLED_ShowString(1, 7, "   ");
			OLED_ShowString(1, 10, "Y:");
			OLED_ShowNum(1, 12, y, 4);
		}
		
	}
}
