#include "pid_api.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "inv_mpu.h"//������������
#include "inv_mpu_dmp_motion_driver.h" //DMP��̬��������
#include "mpu6050.h"
#include "stm32_hal_legacy.h"
#include "XMF_OLED_STM32Cube.h" //OLEDͷ�ļ�
#include "stdio.h"		//ʹ��printf��
#include "inv_mpu.h"
float pitch,roll,yaw; 		//ŷ����:�����ǣ���ת�ǣ�ƫ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����  ���ٶ�
short gyrox,gyroy,gyroz;	//������ԭʼ����  ���ٶ�
//short temp;					//MPU�¶�
int Encoder_LEFT;//���������ȡ����ֵ
int Encoder_Right;//�ұ�������ȡ����ֵ
int PWM_MAX=6000,PWM_MIN=-6000;	// PWM�޷�����
int MOTO1;
int MOTO2;

float Med_Angle=0;      // ��е��ֵ����ʹ��С������ƽ��ס�ĽǶ� 
float Target_Speed=0;	  // �����ٶȡ�---���ο����ӿڣ����ڿ���С��ǰ�����˼����ٶȡ�
float 
  Vertical_Kp=300,
  Vertical_Kd=1.8;      // ֱ����Kp��Kd
//  Vertical_Kp=0,
//  Vertical_Kd=0;      // ֱ����Kp��Kd
float 
  Velocity_Kp=-1,
  Velocity_Ki=-0.005;   // �ٶȻ�Kp��Ki����������
float 
  Turn_Kp=0;
int Vertical_out,Velocity_out,Turn_out; // ֱ����&�ٶȻ�&ת�򻷵������
/*****************  
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
���ڣ�ֱ�������
******************/
int Vertical(float Med,float Angle,float gyro_Y) 
{
  int PWM_out;
  
  PWM_out = Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
  
  return PWM_out;
} 

/*****************  
�ٶȻ�PI��������Kp*Ek+Ki*Ek_S(Ek_S��ƫ��Ļ���)
******************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
  // ����ɾ�̬�����������ھ�̬�洢����ʹ�ñ���������
  static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;
  float a=0.7;
  
  // 1.�����ٶ�ƫ��
  // ��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
  Encoder_Err = ((encoder_left+encoder_right)-Target);
  // 2.���ٶ�ƫ����е�ͨ�˲�
  // low_out = (1-a)*Ek+a*low_out_last
  EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last; // ʹ�ò��θ���ƽ�����˳���Ƶ���ţ������ٶ�ͻ��
  EnC_Err_Lowout_last = EnC_Err_Lowout;   // ��ֹ�ٶȹ���Ӱ��ֱ��������������
  // 3.���ٶ�ƫ����ֳ�λ��
  Encoder_S+=EnC_Err_Lowout;
  // 4.�����޷�
  Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
  
  // 5.�ٶȻ��������
  PWM_out = Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
  
  return PWM_out;
}

/*****************  
ת�򻷣�ϵ��*Z����ٶ�
******************/
int Turn(int gyro_Z)
{
  int PWM_out;
  
  PWM_out = Turn_Kp*gyro_Z;
  
  return PWM_out;
}
void MPU6050_get()
{
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp����õ����ݣ��Է���ֵ�����ж�
	{ 
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
	}
}
//��ȡ�������������
void motor_encoder_get()
{
	Encoder_LEFT=(short)__HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3,0);	
	Encoder_Right=-(short)__HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,0);
}

/*�޷�����*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/*����ֵ����*/
int GFP_abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}

//pwm����
void Load(int moto1,int moto2)
{
	//1.�о������ţ���Ӧ����ת
	if(moto1>0)
	{		
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);//��ת
	}
	if(moto1<0)
	{		
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//��ת
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,GFP_abs(moto1));//����		
  //1.�о������ţ���Ӧ����ת
	if(moto2>0)
	{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);//��ת		
	}
	if(moto2<0)			
	{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);//��ת
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,GFP_abs(moto2));//����		
}
