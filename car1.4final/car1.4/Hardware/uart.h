#ifndef __UART_H
#define __UART_H

#include "stm32f1xx_hal.h"
#include "stdlib.h"

#define S_BLACK		1
#define S_BLUE		2
#define S_BROWN		3
#define S_ORANGE	4

extern uint8_t aRxBuffer;			//??????
extern uint8_t Uart1_RxBuff[256];	//????
extern uint8_t Uart1_Rx_Cnt;		//??????
extern uint8_t Uart1_RxFlag;
extern uint8_t	cAlmStr[];
extern int32_t color;

enum flag
{
	stop=1,
	pause,
	start,
	turn_left,
	turn_Left,
	turn_right,
	turn_Right
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
