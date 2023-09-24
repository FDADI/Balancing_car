#ifndef __PIA_API_H
#define __PID_API_H

/*****************  
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：Med:机械中值(期望角度)，Angle:真实角度，gyro_Y:真实角速度
出口：直立环输出
******************/
int Vertical(float Med,float Angle,float gyro_Y);

/*****************  
速度环PI控制器：Kp*Ek+Ki*Ek_S(Ek_S：偏差的积分)
******************/
int Velocity(int Target,int encoder_left,int encoder_right);
/*****************  
转向环：系数*Z轴角速度
******************/
int Turn(int gyro_Z);
void MPU6050_get(void);
//读取电机编码器数据
void motor_encoder_get(void);

/*限幅函数*/
void Limit(int *motoA,int *motoB);

/*绝对值函数*/
int GFP_abs(int p);

//pwm更新
void Load(int moto1,int moto2);
#endif
