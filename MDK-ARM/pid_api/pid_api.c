#include "pid_api.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "inv_mpu.h"//陀螺仪驱动库
#include "inv_mpu_dmp_motion_driver.h" //DMP姿态解读库读库
#include "mpu6050.h"
#include "stm32_hal_legacy.h"
#include "XMF_OLED_STM32Cube.h" //OLED头文件
#include "stdio.h"		//使用printf用
#include "inv_mpu.h"
float pitch,roll,yaw; 		//欧拉角:俯仰角，滚转角，偏航角
short aacx,aacy,aacz;		//加速度传感器原始数据  加速度
short gyrox,gyroy,gyroz;	//陀螺仪原始数据  角速度
//short temp;					//MPU温度
int Encoder_LEFT;//左编码器读取的数值
int Encoder_Right;//右编码器读取的数值
int PWM_MAX=6000,PWM_MIN=-6000;	// PWM限幅变量
int MOTO1;
int MOTO2;

float Med_Angle=0;      // 机械中值，能使得小车真正平衡住的角度 
float Target_Speed=0;	  // 期望速度。---二次开发接口，用于控制小车前进后退及其速度。
float 
  Vertical_Kp=300,
  Vertical_Kd=1.8;      // 直立环Kp、Kd
//  Vertical_Kp=0,
//  Vertical_Kd=0;      // 直立环Kp、Kd
float 
  Velocity_Kp=-1,
  Velocity_Ki=-0.005;   // 速度环Kp、Ki（正反馈）
float 
  Turn_Kp=0;
int Vertical_out,Velocity_out,Turn_out; // 直立环&速度环&转向环的输出变
/*****************  
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：Med:机械中值(期望角度)，Angle:真实角度，gyro_Y:真实角速度
出口：直立环输出
******************/
int Vertical(float Med,float Angle,float gyro_Y) 
{
  int PWM_out;
  
  PWM_out = Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
  
  return PWM_out;
} 

/*****************  
速度环PI控制器：Kp*Ek+Ki*Ek_S(Ek_S：偏差的积分)
******************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
  // 定义成静态变量，保存在静态存储器，使得变量不丢掉
  static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;
  float a=0.7;
  
  // 1.计算速度偏差
  // 舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
  Encoder_Err = ((encoder_left+encoder_right)-Target);
  // 2.对速度偏差进行低通滤波
  // low_out = (1-a)*Ek+a*low_out_last
  EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last; // 使得波形更加平滑，滤除高频干扰，放置速度突变
  EnC_Err_Lowout_last = EnC_Err_Lowout;   // 防止速度过大影响直立环的正常工作
  // 3.对速度偏差积分出位移
  Encoder_S+=EnC_Err_Lowout;
  // 4.积分限幅
  Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
  
  // 5.速度环控制输出
  PWM_out = Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
  
  return PWM_out;
}

/*****************  
转向环：系数*Z轴角速度
******************/
int Turn(int gyro_Z)
{
  int PWM_out;
  
  PWM_out = Turn_Kp*gyro_Z;
  
  return PWM_out;
}
void MPU6050_get()
{
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp处理得到数据，对返回值进行判断
	{ 
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	}
}
//读取电机编码器数据
void motor_encoder_get()
{
	Encoder_LEFT=(short)__HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3,0);	
	Encoder_Right=-(short)__HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,0);
}

/*限幅函数*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/*绝对值函数*/
int GFP_abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}

//pwm更新
void Load(int moto1,int moto2)
{
	//1.研究正负号，对应正反转
	if(moto1>0)
	{		
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);//正转
	}
	if(moto1<0)
	{		
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//反转
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,GFP_abs(moto1));//右轮		
  //1.研究正负号，对应正反转
	if(moto2>0)
	{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);//正转		
	}
	if(moto2<0)			
	{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);//反转
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,GFP_abs(moto2));//右轮		
}
