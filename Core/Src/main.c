/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32_hal_legacy.h"
#include "XMF_OLED_STM32Cube.h" //OLED头文件
#include "stdio.h"		//使用printf用
#include "inv_mpu.h"
#include "pid_api.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float pitch,roll,yaw; 		//欧拉角:俯仰角，滚转角，偏航角
extern short aacx,aacy,aacz;		//加速度传感器原始数据  加速度
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据  角速度
//short temp;					//MPU温度
extern int Encoder_LEFT;//左编码器读取的数值
extern int Encoder_Right;//右编码器读取的数值
extern int PWM_MAX,PWM_MIN;	// PWM限幅变量
extern int MOTO1;
extern int MOTO2;

extern float Med_Angle;      // 机械中值，能使得小车真正平衡住的角度 
extern float Target_Speed;	  // 期望速度。---二次开发接口，用于控制小车前进后退及其速度。
extern float 
  Vertical_Kp,
  Vertical_Kd;      // 直立环Kp、Kd
//  Vertical_Kp=0,
//  Vertical_Kd=0;      // 直立环Kp、Kd
extern float 
  Velocity_Kp,
  Velocity_Ki;   // 速度环Kp、Ki（正反馈）
extern float 
  Turn_Kp;
extern int Vertical_out,Velocity_out,Turn_out; // 直立环&速度环&转向环的输出变
char flag=0;
uint8_t Target_Speed_flag;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数    
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
uint8_t str_buff[64];
short temp;
int temp1;
int temp2;
int temp3;
uint8_t str_buff1[64];
uint8_t str_buff2[64];
uint8_t str_buff3[64];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim2))
    {
					flag=1;
					__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE ); //清除IT标志位
				
    }
		if(htim == (&htim3))
		{
					__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE ); //清除IT标志位				
		}
		if(htim == (&htim4))
		{
					__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE ); //清除IT标志位				
		}		
	}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == (&huart1))
	{
		if(Target_Speed_flag==0xa0)
			Target_Speed = Target_Speed+10;
		else if(Target_Speed_flag==0xa1)
			Target_Speed = Target_Speed-10;
	}
	HAL_UART_Receive_IT(&huart1,&Target_Speed_flag,sizeof(Target_Speed_flag));//因为接收中断使用了一次即关闭，所以在最后加入这行代码即可实现无限使用
}	
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	
	OLED_Init();
	MPU_Init();
	mpu_dmp_init();                           //初始化mpu_dmp库		
		/*打开编码器捕获*/
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // 开启编码器A
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // 开启编码器B	
		/*打开编码器捕获*/
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // 开启编码器A
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // 开启编码器B	
		/*打开PWM输出*/
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		/*打开PWM输出*/
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim2);
	
	
	HAL_UART_Receive_IT(&huart1,&Target_Speed_flag,sizeof(Target_Speed_flag));
	
//		/*打开定时器中断*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			if(flag==1)
			{
					MPU6050_get();				
					motor_encoder_get();
					Velocity_out=Velocity(Target_Speed,Encoder_LEFT,Encoder_Right); // 速度环			
					Vertical_out=Vertical(Velocity_out+Med_Angle,pitch,gyroy);			  // 直立环
					Turn_out=Turn(gyroz);
				  MOTO2 = Vertical_out+Turn_out; // 右电机
					MOTO1 = Vertical_out-Turn_out; // 左电机			
					Limit(&MOTO1,&MOTO2);     // PWM限幅
					Load(MOTO1,MOTO2);        // 加载到电机上
					flag=0;
			}
    /* USER CODE BEGIN 3 */
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

//void MPU_Read(void)
//{
//	
//	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp处理得到数据，对返回值进行判断
//	{ 
////		temp=MPU_Get_Temperature();	              //得到温度值
//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//		mpu6050.speed++;                          //上报速度自加
//		if(mpu6050.speed == 4)						//上报速度阈值设置
//		{
//			mpu6050.flag = 1;						//采集成功标志位设置为有效
//			mpu6050.speed = 0;						//上报速度归零
//		}	
//	}
//	else 											//采集不成功										
//	{
//		mpu6050.flag = 0;							//采集成功标志位设置为无效
//	}	
//}
/**
  * @brief  MPU6050数据上报
  * @param  无
  * @retval 无
  */
//void DATA_Report(void)
//{
//	if(mpu6050.flag == 1)						//采集成功时
//	{ 
//		if(temp<0)								//对数据正负判断，判断为负时
//		{
//			temp=-temp;							//对负数据取反
//		}
//		else                                    //判断为正时
//		{
//		}
////		sprintf((char *)str_buff,"%d.%d",temp/100,temp%10);
////		OLED_ShowString(60,0,str_buff);
////		printf("temp:%d.%d --- ",temp/100,temp%10); //通过串口1输出温度
//		
//		temp=pitch*10;							 //赋temp为pitch
//		if(temp<0)								//对数据正负判断，判断为负时
//		{
//			temp=-temp;						    //对负数据取反		
//		}
//		else                                    //判断为正时 
//		{
//		}
//		sprintf((char *)str_buff,"%d.%d",(temp)/10,(temp)%10);
//		OLED_ShowString(60,0,str_buff);
////		OLED_ShowString(65,6,str_buff);
////		printf("pitch:%d.%d --- ",temp/10,temp%10); //通过串口1输出pitch	
//		
//		temp=roll*10;                            //赋temp为roll
//		if(temp<0)								//对数据正负判断，判断为负时
//		{
//			temp=-temp;						    //对负数据取反	
//		}
//		else                                    //判断为正时
//		{
//		}
//		sprintf((char *)str_buff,"%d.%d",temp/10,temp%10);
//		OLED_ShowString(60,4,str_buff);
////		printf("roll:%d.%d --- ",temp/10,temp%10);//通过串口1输出roll
//		
//		temp=yaw*10;                           //赋temp为yaw
//	if(temp<0)								//对数据正负判断，判断为负时
//		{
//			temp=-temp;						    //对负数据取反
////			OLED_ShowString(100,6,"-");
//		}
//		else                                    //判断为正时
//		{	//OLED_ShowString(100,6,"+");
//		}
//		sprintf((char *)str_buff,"%d.%d",temp/10,temp%10);
//		OLED_ShowString(60,2,str_buff);
////		sprintf((char *)str_buff,"%f",yaw);
////		OLED_ShowString(60,0,str_buff);
////		printf("yaw:%d.%d\r\n",temp/10,temp%10);//通过串口1输出yaw	
////		printf("gyrox:%d,gyroy:%d,gyroz:%d,aacx:%d,aacy:%d,aacz:%d\r\n",gyrox,gyroy,gyroz,aacx,aacy,aacz);//上报角速度数据，角加速度数据
//		mpu6050.flag = 0;									//采集成功标志位设置为无效
//	}
//	else ;														//防卡死
//}

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
