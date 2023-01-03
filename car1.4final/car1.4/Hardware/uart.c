#include "uart.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "motion.h"
#include "math.h"

uint8_t aRxBuffer;			//??????
uint8_t Uart1_RxBuff[256] = {0};		//????
uint8_t Uart1_Rx_Cnt = 0;		//??????
uint8_t Uart1_RxFlag = 0;
uint8_t	cAlmStr[] = "error(>256)\r\n";
uint8_t flag_uart;
int32_t da,dx,color;
uint8_t *stalow,*stahigh;

extern int flag;
extern int flag_admin;
extern double v_left;
extern double v_right;
int flag_a=0,flag_x=0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	//printf("------------------");
	if(Uart1_Rx_Cnt >= 255)  //????
	{
		Uart1_Rx_Cnt = 0;
		memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));
		HAL_UART_Transmit(&huart1, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
	}
	else
	{
		Uart1_RxBuff[Uart1_Rx_Cnt++] = aRxBuffer;   //??????
		
		//start1
//		if(Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x40)//@
//		{
//			//reset
//			Uart1_Rx_Cnt = 0;
//			memset(Uart1_RxBuff,0x00,256);
//			Uart1_RxBuff[Uart1_Rx_Cnt++] = 0x40;
//			stalow=Uart1_RxBuff+Uart1_Rx_Cnt-1;
//			
//			flag_uart=2;
//		}
//		//end1
//		else if(Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x21&&flag_uart==2&&(!flag_admin))//!
//		{
//			//reset
//			flag_uart=0;
//			Uart1_Rx_Cnt = 0;
//			
//			Uart1_RxFlag = 1;//show
//			flag_admin=1;
//			
//			stahigh=Uart1_RxBuff+Uart1_Rx_Cnt-1;
//			
//			*stalow=0;
//			*stahigh=0;
//			//admin
//			if			(strcmp("stop",				(char*)stalow+1)==0)	flag=stop;
//			else if	(strcmp("start",			(char*)stalow+1)==0)	flag=start;
//			else if	(strcmp("pause",			(char*)(stalow+1))==0)	flag=3;//pause;
//			else if	(strcmp("turn left",	(char*)stalow+1)==0)	flag=turn_left;
//			else if	(strcmp("turn Left",	(char*)stalow+1)==0)	flag=turn_Left;
//			else if	(strcmp("turn right",	(char*)stalow+1)==0)	flag=turn_right;
//			else if	(strcmp("turn Right",	(char*)stalow+1)==0)	flag=turn_Right;
//			else		flag_admin=0;

//			printf("\r\n               %d      %d\r\n",flag,flag_admin);
//		}
		//start2
		if(Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x23) //#
		{	
			//reset
			Uart1_Rx_Cnt = 0;
			memset(Uart1_RxBuff,0x00,256);
			Uart1_RxBuff[Uart1_Rx_Cnt++] = 0x23;
			flag_a=0;
			flag_x=0;
			
			flag_uart=1;
		}
		//end2
		else if(Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x24&&flag_uart!=0) //$
		{
			//reset
			flag_uart=0;
			Uart1_Rx_Cnt = 0;
			
			Uart1_RxFlag = 1;//show
			
			if(flag!=0)
			{
				flag_admin=1;
			}
			
			if(da>180||dx>640)
			{
				printf("data error   ");
				flag_a=0;da=0;
				flag_x=0;dx=0;
				//HAL_Delay(900);
			}
			else
			{
				if(flag_a && flag_x && (!flag_admin))
				{
					da=da-90;
					dx=dx-320;
					flag_a=0;
					flag_x=0;
					
					move_adjust(da,dx);
					
					//angle_adjust(da);
					//displacement_adjust(dx);
					//printf("da:%d   ",da);
					//printf("dx:%d\r\n",dx);
				}
				
//				if(flag_x)
//				{
//					dx=dx-320;
//					flag_x=0;
//					//displacement_adjust(dx);
//					//printf("dx:%d\r\n",dx);
//				}
			}
		}
		else if(Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x24) //?????
		{
			//reset
			flag_uart=0;
			dx=0;
			Uart1_Rx_Cnt = 0;
			memset(Uart1_RxBuff,0x00,256);
		}
		else if(flag_uart==0)
		{
			Uart1_Rx_Cnt=0;
		}
		
		//da dx color
		if((aRxBuffer<48||aRxBuffer>57)&&(flag_uart==4||flag_uart==5||flag_uart==6||flag_uart==7)) flag_uart=1;
		if(flag_uart==4) da=10*da+aRxBuffer-48;
		else if(flag_uart==5) dx=10*dx+aRxBuffer-48;
		else if(flag_uart==6) color=aRxBuffer-48;
		else if(flag_uart==7 && (!flag_admin)) 
		{
			flag=aRxBuffer-48;
		}
		//flag_uart: a x c
		if(flag_uart!=0)
		{
			switch(Uart1_RxBuff[Uart1_Rx_Cnt-1])
			{
				case 'a':
					flag_uart=4;
					flag_a=1;
					da=0;
					break;
				case 'x':
					flag_uart=5;
					flag_x=1;
					dx=0;
					break;
				case 'c':
					flag_uart=6;
					color=0;
					break;
				case 'f':
					flag_uart=7;
					flag=0;
					break;
				default:break;
			}
		}
	}
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);   //???????s
    
}
