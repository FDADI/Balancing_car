#ifndef __PIA_API_H
#define __PID_API_H

/*****************  
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
���ڣ�ֱ�������
******************/
int Vertical(float Med,float Angle,float gyro_Y);

/*****************  
�ٶȻ�PI��������Kp*Ek+Ki*Ek_S(Ek_S��ƫ��Ļ���)
******************/
int Velocity(int Target,int encoder_left,int encoder_right);
/*****************  
ת�򻷣�ϵ��*Z����ٶ�
******************/
int Turn(int gyro_Z);
void MPU6050_get(void);
//��ȡ�������������
void motor_encoder_get(void);

/*�޷�����*/
void Limit(int *motoA,int *motoB);

/*����ֵ����*/
int GFP_abs(int p);

//pwm����
void Load(int moto1,int moto2);
#endif
