/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "motor_control.h"
#include "uart.h"
#include "string.h"
#include "motion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印
  return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double TIM1_OVER=0;
double TIM1_CH3_VAL=0,Last_TIM1_CH3_VAL=0,TIM1_CH4_VAL=0,Last_TIM1_CH4_VAL=0;
double pulse_left=0,pulse_right=0;
double speed_left=0,speed_right=0;

int count_left=0,count_right=0;

const double ENCODER_TOTAL_RESOLUTION=13;
const double REDUCTION_RATIO=30;
const double FREQUENCE=1000000;


double v_left=0.3;
double v_right=0.3;
int roll_left=13*30*2;
int roll_right=13*30*2;
int state=0;
int flag=0;
int flag_admin=0;
int n=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
			TIM1_OVER++;  //定时器溢出值增加

	}
	else if(htim==&htim2)
	{
		if(state==0)
		{
			set_speed_left(speed_left,v_left);
			set_speed_right(speed_right,v_right);
//			printf("speed_right:");
//			printf("%f\r\n",speed_right/60*3.1415926*0.064);
			speed_left=0;
			speed_right=0;
			count_left=0;
			count_right=0;
		}
		else
		{
			set_location_left(roll_left,count_left);
			set_location_right(roll_right,count_right);	
//		printf("count_left:");
//		printf("%d\r\n",count_left);
			if(roll_left==count_left&&roll_right==count_right)
			{
				state=0;
			}
     }
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{   	
		 
    // printf("%f\r\n",speed_left);
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==0)
			{   
				  //TIM1_CH3_VAL = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3)+TIM1_OVER*65536;
				  TIM1_CH3_VAL = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
		      pulse_left=TIM1_CH3_VAL-Last_TIM1_CH3_VAL;
		      Last_TIM1_CH3_VAL= TIM1_CH3_VAL;
				  if(pulse_left<0) pulse_left+=65536;
					speed_left=60*FREQUENCE/ENCODER_TOTAL_RESOLUTION/REDUCTION_RATIO/pulse_left;
//				  printf("1");
				 if(state) 
				   {
						 count_left++;
//						 printf("1");
			     }
			}
			else
			{   
				  TIM1_CH3_VAL = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
		      pulse_left=TIM1_CH3_VAL-Last_TIM1_CH3_VAL;
		      Last_TIM1_CH3_VAL= TIM1_CH3_VAL;
				  if(pulse_left<0) pulse_left+=65536;
				  speed_left=-60*FREQUENCE/ENCODER_TOTAL_RESOLUTION/REDUCTION_RATIO/pulse_left;//-
//				  printf("2");
					if(state)
					{
						count_left--;
			    }
		  }
	}


	else if(htim == &htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{   

		  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==0)
	  {
			//TIM1_CH4_VAL = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4)+TIM1_OVER*65536;
			TIM1_CH4_VAL = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
		  pulse_right=TIM1_CH4_VAL-Last_TIM1_CH4_VAL;
		  Last_TIM1_CH4_VAL= TIM1_CH4_VAL;
			if(pulse_right<0) pulse_right+=65536;
		  speed_right=-60*FREQUENCE/ENCODER_TOTAL_RESOLUTION/REDUCTION_RATIO/pulse_right;  //-
		  if(state) 
			{
				count_right--;
	    }
		}
	   else
			{
			TIM1_CH4_VAL = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
		  pulse_right=TIM1_CH4_VAL-Last_TIM1_CH4_VAL;
		  Last_TIM1_CH4_VAL= TIM1_CH4_VAL;
			if(pulse_right<0) pulse_right+=65536;
			speed_right=60*FREQUENCE/ENCODER_TOTAL_RESOLUTION/REDUCTION_RATIO/pulse_right;
			 if(state) 
			 {
				 count_right++;
		   }
		  }
		  
	}
}
	
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
	v_left=0;
	v_right=0;
	HAL_Delay(500);
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
	HAL_UART_Receive_IT(&huart1,&aRxBuffer,1);
//	     	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
//		 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,50000);
//HAL_Delay(2000);
	printf("ready");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//printf("test");
		if(Uart1_RxFlag != 0)
    {
        printf("recive buff is %s\r\n",Uart1_RxBuff);
        Uart1_RxFlag = 0;
        Uart1_Rx_Cnt = 0;
        memset(Uart1_RxBuff,0x00,256);
				//HAL_Delay(500);
    }
		//flag
		if(flag==0)
		{
			flag_admin=0;
		}
		else if(flag==1)//stop)
		{
			flag_admin=1;
			car_stop();
			v_left=0;
			v_right=0;
			while(1)
			{
				if(0)break;
			}
			//reset
		}
		else if(flag==2)//pause)
		{
			flag_admin=1;
			car_stop();
			v_left=0;
			v_right=0;
			HAL_Delay(5000);
	    car_start();
			v_left=0.55;
			v_right=0.6;
			HAL_Delay(300);
			
			//reset
			flag=0;
			flag_admin=0;
		}
		else if(flag==start)
		{
			
			//reset
			flag=0;
			flag_admin=0;
		}
		else if(flag==turn_left)
		{
			
			//reset
			flag=0;
			flag_admin=0;
		}
		else if(flag==turn_right)
		{
			
			//reset
			flag=0;
			flag_admin=0;
		}
		else if(flag==turn_Left)
		{
			
			//reset
			flag=0;
			flag_admin=0;
		}
		else if(flag==turn_Right)
		{
			
			//reset
			flag=0;
			flag_admin=0;
		}
		flag_admin=0;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
