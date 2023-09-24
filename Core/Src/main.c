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
#include "XMF_OLED_STM32Cube.h" //OLEDͷ�ļ�
#include "stdio.h"		//ʹ��printf��
#include "inv_mpu.h"
#include "pid_api.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float pitch,roll,yaw; 		//ŷ����:�����ǣ���ת�ǣ�ƫ����
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����  ���ٶ�
extern short gyrox,gyroy,gyroz;	//������ԭʼ����  ���ٶ�
//short temp;					//MPU�¶�
extern int Encoder_LEFT;//���������ȡ����ֵ
extern int Encoder_Right;//�ұ�������ȡ����ֵ
extern int PWM_MAX,PWM_MIN;	// PWM�޷�����
extern int MOTO1;
extern int MOTO2;

extern float Med_Angle;      // ��е��ֵ����ʹ��С������ƽ��ס�ĽǶ� 
extern float Target_Speed;	  // �����ٶȡ�---���ο����ӿڣ����ڿ���С��ǰ�����˼����ٶȡ�
extern float 
  Vertical_Kp,
  Vertical_Kd;      // ֱ����Kp��Kd
//  Vertical_Kp=0,
//  Vertical_Kd=0;      // ֱ����Kp��Kd
extern float 
  Velocity_Kp,
  Velocity_Ki;   // �ٶȻ�Kp��Ki����������
extern float 
  Turn_Kp;
extern int Vertical_out,Velocity_out,Turn_out; // ֱ����&�ٶȻ�&ת�򻷵������
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
//��׼����Ҫ��֧�ֺ���    
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
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
					__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE ); //���IT��־λ
				
    }
		if(htim == (&htim3))
		{
					__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE ); //���IT��־λ				
		}
		if(htim == (&htim4))
		{
					__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE ); //���IT��־λ				
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
	HAL_UART_Receive_IT(&huart1,&Target_Speed_flag,sizeof(Target_Speed_flag));//��Ϊ�����ж�ʹ����һ�μ��رգ����������������д��뼴��ʵ������ʹ��
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
	mpu_dmp_init();                           //��ʼ��mpu_dmp��		
		/*�򿪱���������*/
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // ����������A
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // ����������B	
		/*�򿪱���������*/
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // ����������A
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // ����������B	
		/*��PWM���*/
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		/*��PWM���*/
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim2);
	
	
	HAL_UART_Receive_IT(&huart1,&Target_Speed_flag,sizeof(Target_Speed_flag));
	
//		/*�򿪶�ʱ���ж�*/
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
					Velocity_out=Velocity(Target_Speed,Encoder_LEFT,Encoder_Right); // �ٶȻ�			
					Vertical_out=Vertical(Velocity_out+Med_Angle,pitch,gyroy);			  // ֱ����
					Turn_out=Turn(gyroz);
				  MOTO2 = Vertical_out+Turn_out; // �ҵ��
					MOTO1 = Vertical_out-Turn_out; // ����			
					Limit(&MOTO1,&MOTO2);     // PWM�޷�
					Load(MOTO1,MOTO2);        // ���ص������
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
//	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp����õ����ݣ��Է���ֵ�����ж�
//	{ 
////		temp=MPU_Get_Temperature();	              //�õ��¶�ֵ
//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//		mpu6050.speed++;                          //�ϱ��ٶ��Լ�
//		if(mpu6050.speed == 4)						//�ϱ��ٶ���ֵ����
//		{
//			mpu6050.flag = 1;						//�ɼ��ɹ���־λ����Ϊ��Ч
//			mpu6050.speed = 0;						//�ϱ��ٶȹ���
//		}	
//	}
//	else 											//�ɼ����ɹ�										
//	{
//		mpu6050.flag = 0;							//�ɼ��ɹ���־λ����Ϊ��Ч
//	}	
//}
/**
  * @brief  MPU6050�����ϱ�
  * @param  ��
  * @retval ��
  */
//void DATA_Report(void)
//{
//	if(mpu6050.flag == 1)						//�ɼ��ɹ�ʱ
//	{ 
//		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
//		{
//			temp=-temp;							//�Ը�����ȡ��
//		}
//		else                                    //�ж�Ϊ��ʱ
//		{
//		}
////		sprintf((char *)str_buff,"%d.%d",temp/100,temp%10);
////		OLED_ShowString(60,0,str_buff);
////		printf("temp:%d.%d --- ",temp/100,temp%10); //ͨ������1����¶�
//		
//		temp=pitch*10;							 //��tempΪpitch
//		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
//		{
//			temp=-temp;						    //�Ը�����ȡ��		
//		}
//		else                                    //�ж�Ϊ��ʱ 
//		{
//		}
//		sprintf((char *)str_buff,"%d.%d",(temp)/10,(temp)%10);
//		OLED_ShowString(60,0,str_buff);
////		OLED_ShowString(65,6,str_buff);
////		printf("pitch:%d.%d --- ",temp/10,temp%10); //ͨ������1���pitch	
//		
//		temp=roll*10;                            //��tempΪroll
//		if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
//		{
//			temp=-temp;						    //�Ը�����ȡ��	
//		}
//		else                                    //�ж�Ϊ��ʱ
//		{
//		}
//		sprintf((char *)str_buff,"%d.%d",temp/10,temp%10);
//		OLED_ShowString(60,4,str_buff);
////		printf("roll:%d.%d --- ",temp/10,temp%10);//ͨ������1���roll
//		
//		temp=yaw*10;                           //��tempΪyaw
//	if(temp<0)								//�����������жϣ��ж�Ϊ��ʱ
//		{
//			temp=-temp;						    //�Ը�����ȡ��
////			OLED_ShowString(100,6,"-");
//		}
//		else                                    //�ж�Ϊ��ʱ
//		{	//OLED_ShowString(100,6,"+");
//		}
//		sprintf((char *)str_buff,"%d.%d",temp/10,temp%10);
//		OLED_ShowString(60,2,str_buff);
////		sprintf((char *)str_buff,"%f",yaw);
////		OLED_ShowString(60,0,str_buff);
////		printf("yaw:%d.%d\r\n",temp/10,temp%10);//ͨ������1���yaw	
////		printf("gyrox:%d,gyroy:%d,gyroz:%d,aacx:%d,aacy:%d,aacz:%d\r\n",gyrox,gyroy,gyroz,aacx,aacy,aacz);//�ϱ����ٶ����ݣ��Ǽ��ٶ�����
//		mpu6050.flag = 0;									//�ɼ��ɹ���־λ����Ϊ��Ч
//	}
//	else ;														//������
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
