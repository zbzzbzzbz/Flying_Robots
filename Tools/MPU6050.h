#ifndef __MPU6050_H
#define __MPU6050_H

void MPU_Init(void);
int16_t MPU_dataout(uint8_t readaddr1,uint8_t readaddr2);
void MPU_Init2(void);
void MPU_getdata(void);
struct Angle{
	float roll;
	float pitch;
	float yaw;
};
void attitude_Solution(struct Angle *angle);//姿态解算
void HMC5883L_Init(void);
float HMC5883_Get_Angle(float *mx,float *my,float *mz);
void gyzero_err(void);//陀螺仪零偏矫正


//上位机函数
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short roll,short pitch,short yaw);
void usart1_report_offset(short acc_x,short acc_y,short acc_z,short gyro_x,short gyro_y,short gyro_z);
void send_feikong_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short mx,short my,short mz, short roll, short pitch, short yaw);


#endif
