#include "includes.h"

void MPU_Init2()//main.c中用的是该函数
{
	I2C_Sand_Byte(0xD0,0x6B,0x00);
	I2C_Sand_Byte(0xD0,0x19,0x00);
	I2C_Sand_Byte(0xD0,0x1A,0x06);
	I2C_Sand_Byte(0xD0,0x1B,0x18);//设置角度量程(2000度/秒)
	I2C_Sand_Byte(0xD0,0x1C,0x00);//设置加速度量程(+-2g)
}


void MPU_Init()//初始化MPU
{
	I2C_Sand_Byte(0xD0,0x6B,0x80);//复位
	//延迟100毫秒
	newdelay1(100);
	I2C_Sand_Byte(0xD0,0x6B,0x01);//唤醒，选时钟源
	I2C_Sand_Byte(0xD0,0x6C,0x00);//不需要待机和低功耗
	I2C_Sand_Byte(0xD0,0x38,0x00);//禁止中断
	I2C_Sand_Byte(0xD0,0x19,0x00);//低通滤波不分频
	I2C_Sand_Byte(0xD0,0x1A,0x04);
	I2C_Sand_Byte(0xD0,0x1B,0x18);//设置角度量程(500度/秒)
	I2C_Sand_Byte(0xD0,0x1C,0x08);
	
}	

int16_t MPU_dataout(uint8_t readaddr1,uint8_t readaddr2)
{
	int16_t data = 0x0000;
	data |= I2C_Read_Byte(0xD0,readaddr1);
	data = data<<8;
	data |= I2C_Read_Byte(0xD0,readaddr2);
	return data;
}

void MPU_getdata()//以十六进制输出
{
	OLED_ShowHexNum2(1,1,MPU_dataout(0x43,0x44),4);//陀螺仪
	OLED_ShowHexNum2(1,6,MPU_dataout(0x45,0x46),4);
	OLED_ShowHexNum2(1,11,MPU_dataout(0x47,0x48),4);		
	OLED_ShowHexNum2(3,1,MPU_dataout(0x3B,0x3C),4);//加速度
	OLED_ShowHexNum2(3,6,MPU_dataout(0x3D,0x3E),4);
	OLED_ShowHexNum2(3,11,MPU_dataout(0x3F,0x40),4);
}


//HMC5883L获取磁场传感器数据
#define HMC5883L_Addr 0x3C
float HMC_angle;	//电子罗盘输出角度，与地磁北极的偏转角度
void HMC5883L_Init(void)
{
	I2C_Sand_Byte(0xD0,0x37,0x02);	//通过MPU6050读取HMC数据需要打开BYPASS模式
	I2C_Sand_Byte(HMC5883L_Addr,0x00,0x58);//写寄存器A，30Hz数据输出、采样平均数0
	I2C_Sand_Byte(HMC5883L_Addr,0x01,0x20);//写寄存器B，传感器量程+-1.3Ga、增益1090高斯
	I2C_Sand_Byte(HMC5883L_Addr,0x02,0x00);//写寄存器C，连续数据输出
	
}	


float HMC5883_Get_Angle(float *mx,float *my,float *mz)
{
	uint8_t i;
	float HMC_Angle;
	short Recive_Data[6];
	for(i=0; i<6; i++)
    {
        Recive_Data[i] = I2C_Read_Byte(HMC5883L_Addr, i+3) ;  //get data
    }
	//单位为高斯
	int16_t dxra,dyra,dzra;
	dxra = Recive_Data[0]<<8 | Recive_Data[1];
	*mx = (float)dxra /1090;//Combine MSB and LSB of X Data output register
	
	dzra = (Recive_Data[2]<<8 | Recive_Data[3]);
	*mz = (float)dzra /1090;//Combine MSB and LSB of Z Data output register
	
    dyra = (Recive_Data[4]<<8 | Recive_Data[5]);
    *my = (float)dyra /1090;//Combine MSB and LSB of Y Data output register
	
	
	HMC_Angle = atan2((double)*my,(double)*mx) * (180 / 3.14159265) + 180; // angle in degrees
	return HMC_Angle;
}
//







//姿态解算(陀螺仪，加速度计，互补滤波以及梯度下降数据融合)
float g = 9.8f;
#define kP 1.6f
#define kI 0.001f
#define T 0.00124f
#define halfT 0.5f*T
#define u 0.05f		//梯度下降算法步长
#define a 0.1f		//互补滤波的权重

//初始化四元数参数
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float exInt = 0,eyInt = 0,ezInt = 0;
float pi = 3.1415f;
float gx_last = 0 ,gy_last = 0,gz_last = 0;
float norm = 0.0f;
float beta = 0.033f;	//发散速率
float s0, s1, s2, s3;

float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
float gx,gy,gz,ax,ay,az,mx,my,mz;
float w_xyz;//衡量角速度大小
float gyrox_err=0,gyroy_err=0,gyroz_err=0;//零偏矫正
int Initial_attitude=0;

//地磁极系数校正
//x = (x-mxoffset)/Kx
float Kx = 0.4864; //尺度因子
float Ky = 0.5031;
float Kz = 0.4284;
float mxoffset = -0.1142f,myoffset = 0.1277f,mzoffset=-0.0953f;//磁力计漂移

float axoffset = 0.03f,ayoffset = -0.03f,azoffset = 0.00f;//加速度计漂移



#define RAD_TO_DEG 57.295780
#define DEG_TO_RAD 0.017453
#define ACCEL_CORRECTOR 0.000061
#define GYRO_CORRECTOR 0.007634


//PID参数初始化
float EXP_ANGLE_ROLL = 0;//期望角度值
float EXP_ANGLE_PITCH = 0;
float EXP_ANGLE_YAW = 0;


float Roll_shell_kp=250;//外环PID参数             
float Roll_shell_ki=0;//

float Pitch_shell_kp=280;//
float Pitch_shell_ki=0;//

float Yaw_shell_kp=1.5;//       
float Yaw_shell_ki=0;//

float Pitch_i,Roll_i,Yaw_i;//积分项
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;//外环总输出


float Pitch_core_kp=0.040;//内环PID参数
float Pitch_core_ki=0.002;
float Pitch_core_kd=0.002;

float Roll_core_kp=0.040;
float Roll_core_ki=0.002;
float Roll_core_kd=0.002;

float Yaw_core_kp=0.046;
float Yaw_core_ki=0.002;
float Yaw_core_kd=0.012;

float Pitch_core_i,Roll_core_i,Yaw_core_i;//积分项
float Pitch_core_d,Roll_core_d,Yaw_core_d;//微分项
float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环输出

float last_diff_gx = 0,last_diff_gy = 0,last_diff_gz = 0;



void gyzero_err(void)//陀螺仪零偏矫正,计算初始姿态
{
	float gx_sum=0,gy_sum=0,gz_sum=0;
	for(int i=0;i<100;i++)
	{
		//XYZ轴角速度
		gx_sum +=((MPU_dataout(0x43,0x44))/16.4);//零偏
		gy_sum +=((MPU_dataout(0x45,0x46))/16.4);
		gz_sum +=((MPU_dataout(0x47,0x48))/16.4);//陀螺仪Z轴旋转正方向与磁力计相反？？
	}

//	gyrox_err = 0.01f*gx_sum;
//	gyroy_err = 0.01f*gy_sum;
//	gyroz_err = 0.01f*gz_sum;
	gyrox_err = -3.369f;
	gyroy_err = -0.31f;
	gyroz_err = -1.404f;
	
	
	//XYZ轴加速度
	ax = (MPU_dataout(0x3B,0x3C))/16384.0f - axoffset;
	ay = (MPU_dataout(0x3D,0x3E))/16384.0f - ayoffset;
	az = (MPU_dataout(0x3F,0x40))/16384.0f;	

	//XYZ轴地磁计
	HMC5883_Get_Angle(&mx,&my,&mz);
	//地磁极零偏校准
	mx = (mx - mxoffset)/Kx;
	my = (my - myoffset)/Ky;
	mz = (mz - mzoffset)/Kz;

	float roll_0 = atan2(ay,az);
	float pitch_0 = asin(-ax);
	
	float mx_0_earth = mx*cos(pitch_0) + my*(sin(roll_0)*sin(pitch_0)) + mz*(cos(roll_0)*sin(pitch_0));
	float my_0_earth =my*cos(roll_0) - mz*sin(roll_0);
	//float yaw_0 = atan2(my_0_earth,mx_0_earth);
	float yaw_0 = -atan2(my_0_earth,mx_0_earth);
//	yaw_0 = HMC5883_Get_Angle(&mxx,&myy,&mzz);

	//欧拉角转四元数
	q0 = cos(roll_0/2)*cos(pitch_0/2)*cos(yaw_0/2) + sin(roll_0/2)*sin(pitch_0/2)*sin(yaw_0/2);
	q1 = sin(roll_0/2)*cos(pitch_0/2)*cos(yaw_0/2) - cos(roll_0/2)*sin(pitch_0/2)*sin(yaw_0/2); 
	q2 = cos(roll_0/2)*sin(pitch_0/2)*cos(yaw_0/2) + sin(roll_0/2)*cos(pitch_0/2)*sin(yaw_0/2);
	q3 = -sin(roll_0/2)*sin(pitch_0/2)*cos(yaw_0/2) + cos(roll_0/2)*cos(pitch_0/2)*sin(yaw_0/2);
	
}

	


void attitude_Solution(struct Angle * angle)
{
	
		//角速度读数
		int16_t GYR_X = MPU_dataout(0x43,0x44);
		int16_t GYR_Y = MPU_dataout(0x45,0x46);
		int16_t GYR_Z = MPU_dataout(0x47,0x48);
		
		//float gx,gy,gz,ax,ay,az,mx,my,mz;
		//XYZ轴角速度
		gx=((MPU_dataout(0x43,0x44))/16.4) - gyrox_err;//零偏
		gy=((MPU_dataout(0x45,0x46))/16.4) - gyroy_err;
		gz=((MPU_dataout(0x47,0x48))/16.4) - gyroz_err;//陀螺仪Z轴旋转正方向与磁力计相反？？
		
		//XYZ轴加速度
		ax = (MPU_dataout(0x3B,0x3C))/16384.0f - axoffset;
		ay = (MPU_dataout(0x3D,0x3E))/16384.0f - ayoffset;
		az = (MPU_dataout(0x3F,0x40))/16384.0f;	

		//XYZ轴地磁计
		HMC5883_Get_Angle(&mx,&my,&mz);
	
		//地磁极零偏校准
		mx = (mx - mxoffset)/Kx;
		my = (my - myoffset)/Ky;
		mz = (mz - mzoffset)/Kz;
		
		
		if(ax*ay*az==0)
			return;
		

		
		float q0_last = q0;
		float q1_last = q1;
		float q2_last = q2;
		float q3_last = q3;
		
		
		
		//RK1
		float qDot0 = 0.5f*(-gx*q1_last - gy*q2_last - gz*q3_last);
		float qDot1 = 0.5f*(gx*q0_last + gz*q2_last - gy*q3_last);
		float qDot2 = 0.5f*(gy*q0_last - gz*q1_last + gx*q3_last);
		float qDot3 = 0.5f*(gz*q0_last + gy*q1_last - gx*q2_last);
	
//		q0 = q0_last+halfT*(-gx*q1_last - gy*q2_last - gz*q3_last);
//		q1 = q1_last+halfT*(gx*q0_last + gz*q2_last - gy*q3_last);
//		q2 = q2_last+halfT*(gy*q0_last - gz*q1_last + gx*q3_last);
//		q3 = q3_last+halfT*(gz*q0_last + gy*q1_last - gx*q2_last);
	
		
//		//RK4
//		float gx_mid =0.5f*(gx+gx_last);
//		float gy_mid =0.5f*(gy+gy_last);
//		float gz_mid =0.5f*(gz+gz_last);
//			
//		
//		float k1_0 = 0.5f*(-gx_last*q1_last - gy_last*q2_last - gz_last*q3_last);
//		float k1_1 = 0.5f*(gx_last*q0_last + gz_last*q2_last - gy_last*q3_last);
//		float k1_2 = 0.5f*(gy_last*q0_last - gz_last*q1_last + gx_last*q3_last);
//		float k1_3 = 0.5f*(gz_last*q0_last + gy_last*q1_last - gx_last*q2_last);
//		
//		
//		float k2_0 = 0.5f*(-gx_mid* (q1_last + halfT*k1_1) - gy_mid* (q2_last + halfT*k1_2) - gz_mid* (q3_last + halfT*k1_3));
//		float k2_1 = 0.5f*(gx_mid* (q0_last + halfT*k1_0) + gz_mid* (q2_last + halfT*k1_2) - gy_mid* (q3_last + halfT*k1_3));
//		float k2_2 = 0.5f*(gy_mid* (q0_last + halfT*k1_0) - gz_mid* (q1_last + halfT*k1_1) + gx_mid* (q3_last + halfT*k1_3));
//		float k2_3 = 0.5f*(gz_mid* (q0_last + halfT*k1_0) + gy_mid* (q1_last + halfT*k1_1) - gx_mid* (q2_last + halfT*k1_2));
//		
//		float k3_0 = 0.5f*(-gx_mid* (q1_last + halfT*k2_1) - gy_mid* (q2_last + halfT*k2_2) - gz_mid* (q3_last + halfT*k2_3));
//		float k3_1 = 0.5f*(gx_mid* (q0_last + halfT*k2_0) + gz_mid* (q2_last + halfT*k2_2) - gy_mid* (q3_last + halfT*k2_3));
//		float k3_2 = 0.5f*(gy_mid* (q0_last + halfT*k2_0) - gz_mid* (q1_last + halfT*k2_1) + gx_mid* (q3_last + halfT*k2_3));
//		float k3_3 = 0.5f*(gz_mid* (q0_last + halfT*k2_0) + gy_mid* (q1_last + halfT*k2_1) - gx_mid* (q2_last + halfT*k2_2));
//		
//		float k4_0 = 0.5f*(-gx* (q1_last + T*k3_1) - gy* (q2_last + T*k3_2) - gz* (q3_last + T*k3_3));
//		float k4_1 = 0.5f*(gx* (q0_last + T*k3_0) + gz* (q2_last + T*k3_2) - gy* (q3_last + T*k3_3));
//		float k4_2 = 0.5f*(gy* (q0_last + T*k3_0) - gz* (q1_last + T*k3_1) + gx* (q3_last + T*k3_3));
//		float k4_3 = 0.5f*(gz* (q0_last + T*k3_0) + gy* (q1_last + T*k3_1) - gx* (q2_last + T*k3_2));
//		
//		
//		q0 = q0 + (T*0.166f) * (k1_0 + 2*k2_0 + 2*k3_0 + k4_0);
//		q1 = q1 + (T*0.166f) * (k1_1 + 2*k2_1 + 2*k3_1 + k4_1);
//		q2 = q2 + (T*0.166f) * (k1_2 + 2*k2_2 + 2*k3_2 + k4_2);
//		q3 = q3 + (T*0.166f) * (k1_3 + 2*k2_3 + 2*k3_3 + k4_3);
//		
		
	
		//加速度输出数据归一化
		norm = 1.0f/sqrt(ax*ax + ay*ay + az*az);
		ax = ax *norm;
		ay = ay *norm;
		az = az *norm;
//		
		//地磁计数据归一化
		norm = 1.0f/sqrt(mx*mx + my*my + mz*mz);
		mx = mx *norm;
		my = my *norm;
		mz = mz *norm;
		
//		//四元数归一化
//		norm = 1.0f/sqrt(q0*q0 + q1*q1 + q2*q2+q3*q3);
//		q0 = q0*norm;
//		q1 = q1*norm;
//		q2 = q2*norm;
//		q3 = q3*norm;
		

		
		
	//MAGDWICK
	
//		//计算地球坐标系中地磁bx和bz
//		float hx = mx*q0*q0 - 2*q0*my*q3 + 2*q0*mz*q2 + mx*q1*q1 + 2*q1*my*q2 + 2*q1*mz*q3 - mx*q2*q2 - mx*q3*q3;
//		float hy = 2*q0*mx*q3 + my*q0*q0 - 2*q0*mz*q1 + 2*q1*mx*q2 - my*q1*q1 + my*q2*q2 + 2*q2*mz*q3 - my*q3*q3;
//		float bx = sqrt(hx * hx + hy * hy);
//		float bz = -2*q0*mx*q2 + 2*q0*my*q1 + mz*q0*q0 + 2*q1*mx*q3 - mz*q1*q1 + 2*q2*my*q3 - mz*q2*q2 + mz*q3*q3;
//		float _2bx = 2.0f*bx;
//		float _2bz = 2.0f*bz;
//		
//		//加速度和地磁梯度
//		float s0 = -2*q2 * (2.0f * q1*q3 - 2*q0*q2 - ax) + 2*q1 * (2.0f * q0*q1 + 2*q2*q3 - ay) - bz * q2 * (bx * (0.5f - q2*q2 - q3*q3) + bz * (q1*q3 - q0*q2) - mx) + (-bx * q3 + bz*q1) * (bx * (q1*q2 - q0*q3) + bz * (q0*q1 + q2*q3) - my) + bx * q2 * (bx * (q0*q2 + q1*q3) + bz * (0.5f - q1*q1 - q2*q2) - mz);
//		float s1 = 2*q3 * (2.0f * q1*q3 - 2*q0*q2 - ax) + 2*q0 * (2.0f * q0*q1 + 2*q2*q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1*q1 - 2.0f * q2*q2 - az) + bz * q3 * (bx * (0.5f - q2*q2 - q3*q3) + bz * (q1*q3 - q0*q2) - mx) + (bx * q2 + bz * q0) * (bx * (q1*q2 - q0*q3) + bz * (q0*q1 + q2*q3) - my) + (bx * q3 - _2bz * q1) * (bx * (q0*q2 + q1*q3) + bz * (0.5f - q1*q1 - q2*q2) - mz);
//		float s2 = -2*q0 * (2.0f * q1*q3 - 2*q0*q2 - ax) + 2*q3 * (2.0f * q0*q1 + 2*q2*q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1*q1 - 2.0f * q2*q2 - az) + (-_2bx * q2 - bz * q0) * (bx * (0.5f - q2*q2 - q3*q3) + bz * (q1*q3 - q0*q2) - mx) + (bx * q1 + bz * q3) * (bx * (q1*q2 - q0*q3) + bz * (q0*q1 + q2*q3) - my) + (bx * q0 - _2bz * q2) * (bx * (q0*q2 + q1*q3) + bz * (0.5f - q1*q1 - q2*q2) - mz);
//		float s3 = 2*q1 * (2.0f * q1*q3 - 2*q0*q2 - ax) + 2*q2 * (2.0f * q0*q1 + 2*q2*q3 - ay) + (-_2bx * q3 + bz*q1) * (bx * (0.5f - q2*q2 - q3*q3) + bz * (q1*q3 - q0*q2) - mx) + (-bx * q0 + bz * q2) * (bx * (q1*q2 - q0*q3) + bz * (q0*q1 + q2*q3) - my) + bx * q1 * (bx * (q0*q2 + q1*q3) + bz * (0.5f - q1*q1 - q2*q2) - mz);
//		
		
		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
		
		
		// Reference direction of Earth's magnetic field
		float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		
		
		
//		_2bx = 0.5500;// bx指向北
//		_2bz = 0.8351; //bz指向地
		
		norm = 1.0f/sqrt(_2bx*_2bx + _2bz*_2bz);
		_2bx = _2bx *norm;
		_2bz = _2bz *norm;
		

		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
 
		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);


		float invnorm = sqrt(s0*s0+s1*s1+s2*s2+s3*s3);
		float lamda;
		if(invnorm < 0.01f)
		{
			lamda = 0.03f;
		}else{
			lamda = 40.03f;
		}
		beta = (lamda/invnorm+0.01f)*0.5f;
		
//		norm = 1.0f/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= norm;
//		s1 *= norm;
//		s2 *= norm;
//		s3 *= norm;
//	
	
		
		
		qDot0 -= beta * s0;
		qDot1 -= beta * s1;
		qDot2 -= beta * s2;
		qDot3 -= beta * s3;
		
		
		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot0 * T;
		q1 += qDot1 * T;
		q2 += qDot2 * T;
		q3 += qDot3 * T;
		
		
		norm = 1.0f/sqrt(q0*q0 + q1*q1 + q2*q2+q3*q3);
		q0 = q0*norm;
		q1 = q1*norm;
		q2 = q2*norm;
		q3 = q3*norm;

		
		
	//	

		
		
		
		//四元数转欧拉角
		//横滚角(ROLL)
		angle->roll = atan2(2.0f*(q0*q1 + q2*q3),1-2*q1*q1-2*q2*q2)*57.29577;
		//俯仰角(Pitch)
		angle->pitch = asin(2.0f*(q0*q2 - q1*q3))*57.29577;
		//偏航角(Yaw)
		//angle->yaw = atan2(2.0f*(q0*q3+q2*q1),1-2*q2*q2-2*q3*q3)*57.29577;
		angle->yaw = -atan2(2.0f*(q0*q3+q2*q1),1-2*q2*q2-2*q3*q3)*57.29577;

		//发送数据给上位机
		//mpu6050_send_data(ax,ay,az,gx,gy,gz);
		usart1_report_imu(angle->roll,angle->pitch,angle->yaw);
		send_feikong_data(100*ax,100*ay,100*az,100*gx,100*gy,100*gz,100*mx,100*my,100*mz,100*(angle->roll),100*(angle->pitch),10*(angle->yaw));
		
		
//	//PID控制
//		
//		//外环角度控制,偏差=目标期望角度-传感器实测角度 
//		//积分
//		Roll_i+=(EXP_ANGLE_ROLL - angle->roll);
//		Pitch_i+=(EXP_ANGLE_PITCH - angle->pitch);
//		Yaw_i+=(EXP_ANGLE_YAW - angle->yaw);
//		//输出
//		Roll_shell_out  = Roll_shell_kp*(EXP_ANGLE_ROLL - angle->roll) + Roll_shell_ki*Roll_i;
//		Pitch_shell_out = Pitch_shell_kp*(EXP_ANGLE_PITCH - angle->pitch) + Pitch_shell_ki*Pitch_i;
//		Yaw_shell_out  = Yaw_shell_kp*(EXP_ANGLE_YAW - angle->yaw) + Yaw_shell_ki*Yaw_i;
//		
//		
//		//内环角速度控制
//		//积分
//		Roll_core_i+=(Roll_shell_out - angle->roll);
//		Pitch_core_i+=(Pitch_shell_out - angle->pitch);
//		Yaw_core_i+=(Yaw_shell_out - angle->yaw);
//		
//		
//		Pitch_core_out = Pitch_core_kp * (Pitch_shell_out - gy) + Pitch_core_ki*Pitch_core_i + Pitch_core_kd * (Pitch_shell_out - gy  - last_diff_gy);
//		Roll_core_out = Roll_core_kp * (Roll_shell_out - gx) + Roll_core_ki*Roll_core_i + Roll_core_kd * (Roll_shell_out - gx  - last_diff_gx);
//		Yaw_core_out = Yaw_core_kp * (Yaw_shell_out - gz) + Yaw_core_ki*Yaw_core_i + Yaw_core_kd * (Yaw_shell_out - gz  - last_diff_gz);
//		
//		last_diff_gy = Pitch_shell_out - gy;
//		last_diff_gx = Roll_shell_out - gx;
//		last_diff_gz = Yaw_shell_out - gz;
//		
//		
//		
//		gx_last = gx;
//		gy_last = gy;
//		gz_last = gz;
}

	



//发送数据给匿名上位机
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len)
{
	uint8_t send_buf[32]={0x00};
	uint8_t i;
	if(len>28) return;//超过28个字节，无效
	send_buf[len+3]=0;//校验位置零
	send_buf[0]=0x88;//帧头0x88
	send_buf[1]=fun;//命令帧FUN
	send_buf[2]=len;//数据长度帧LEN
	for(i=0;i<len;i++)
		send_buf[i+3]=data[i];
	for(i=0;i<len+3;i++)
		send_buf[len+3] += send_buf[i];//计算数据校验位SUM
	for(i=0;i<len+4;i++)
		USART_sand_byte(send_buf[i]);//发送数据到串口1
	
}



//发送加速度传感器和陀螺仪传感器数据给上位机
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	uint8_t buf[12];
	buf[0]=(aacx>>8)&0xFF;
	buf[1]=aacx&0xFF;
	buf[2]=(aacy>>8)&0xFF;
	buf[3]=aacy&0xFF;
	buf[4]=(aacz>>8)&0xFF;
	buf[5]=aacz&0xFF;
	
	buf[6]=(gyrox>>8)&0xFF;
	buf[7]=gyrox&0xFF;
	buf[8]=(gyroy>>8)&0xFF;
	buf[9]=gyroy&0xFF;
	buf[10]=(gyroz>>8)&0xFF;
	buf[11]=gyroz&0xFF;
	
	usart1_niming_report(0xA1,buf,12);
}


//上报解算后的姿态数据给上位机
void usart1_report_imu(short roll,short pitch,short yaw)
{
	
	uint8_t tbuf[16];
    unsigned char *p;
    p=(unsigned char *)&pitch;
    tbuf[0]=(unsigned char)(*(p+3));
    tbuf[1]=(unsigned char)(*(p+2));
    tbuf[2]=(unsigned char)(*(p+1));
    tbuf[3]=(unsigned char)(*(p+0));
     
    p=(unsigned char *)&roll;
    tbuf[4]=(unsigned char)(*(p+3));
    tbuf[5]=(unsigned char)(*(p+2));
    tbuf[6]=(unsigned char)(*(p+1));
    tbuf[7]=(unsigned char)(*(p+0));
     
    p=(unsigned char *)&yaw;
    tbuf[8]=(unsigned char)(*(p+3));
    tbuf[9]=(unsigned char)(*(p+2));
    tbuf[10]=(unsigned char)(*(p+1));
    tbuf[11]=(unsigned char)(*(p+0));
	usart1_niming_report(0xA2,tbuf,12);
}


//飞控模型

void send_feikong_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz,short mx,short my,short mz,short roll, short pitch, short yaw)
{
    unsigned char buf[28]; 

    memset(buf, 0, 28);
	//加速度计
    buf[0]  = (aacx >> 8) & 0xff;
    buf[1]  = aacx & 0xff;

    buf[2]  = (aacy >> 8) & 0xff;
    buf[3]  = aacy & 0xff;

    buf[4]  = (aacz >> 8) & 0xff;
    buf[5]  = aacz & 0xff;
	//陀螺仪
    buf[6]  = (gyrox >> 8) & 0xff;
    buf[7]  = gyrox & 0xff;

    buf[8]  = (gyroy >> 8) & 0xff;
    buf[9]  = gyroy & 0xff;

    buf[10] = (gyroz >> 8) & 0xff;
    buf[11] = gyroz & 0xff;
	//磁力计
    buf[12]  = (mx >> 8) & 0xff;
    buf[13]  = mx & 0xff;

    buf[14]  = (my >> 8) & 0xff;
    buf[15]  = my & 0xff;

    buf[16] = (mz >> 8) & 0xff;
    buf[17] = mz & 0xff;
	//欧拉角
    buf[18] = (roll >> 8) & 0xff;
    buf[19] = roll & 0xff;

    buf[20] = (pitch >> 8) & 0xff;
    buf[21] = pitch & 0xff;

    buf[22] = (yaw >> 8) & 0xff;
    buf[23] = yaw & 0xff;
	

    usart1_niming_report(0xaf, buf, 28);
}



//功能：发送offset给上位机
void usart1_report_offset(short acc_x,short acc_y,short acc_z,short gyro_x,short gyro_y,short gyro_z)
{
	uint8_t buf[28]={0x00};
	
	buf[0]=0xAC;
	
	buf[1]=(acc_x>>8)&0xFF;
	buf[2]=acc_x&0xFF;
	buf[3]=(acc_y>>8)&0xFF;
	buf[4]=acc_y&0xFF;
	buf[5]=(acc_z>>8)&0xFF;
	buf[6]=acc_z&0xFF;	
	
	buf[7]=(gyro_x>>8)&0xFF;
	buf[8]=gyro_x&0xFF;
	buf[9]=(gyro_y>>8)&0xFF;
	buf[10]=gyro_y&0xFF;
	buf[11]=(gyro_z>>8)&0xFF;
	buf[12]=gyro_z&0xFF;

	usart1_niming_report(0xAC,buf,28);
}











