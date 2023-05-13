/*
imu型号：bmi270

更换通信方式必须重新上电！！！！！！！！！！！！！！！！！！！！！！！！！

使用SPI通信：
info.tpye = D_SPI2,

使用IIC通信：
info.tpye = D_IIC,

	.info.flag.rpy 	 = IMU_ENABLE,
	.info.flag.wGryo = IMU_ENABLE,

陀螺仪坐标系：
power为主控电源位置，按下图摆放找坐标系
新主控：
正面射出z轴，朝右x轴，朝上y轴
 ------------
|            | 
|power       |
|            |
 ------------

旧主控：
背面射出z轴，朝右x轴，朝上y轴
 ------------
|            | 
|            |
|power       |
|            |
|            |
 ------------

注意：
控制时要保证角度、速度、扭矩处于同一坐标系

四元数：
th = angle/180*pi/2;
a = cos(th);
b = ax(1)*sin(th);
c = ax(2)*sin(th);
d = ax(3)*sin(th);

imu包内置功能：
1.装配位置坐标系设置
2.四元数计算自身系转世界坐标系角速度
3.差分计算世界坐标系角速度
4.自带四通道低通滤波
5.rpy角度解算



 前
     ^
  x /|\
y z  |

*/
#include "RP_CONFIG.h"
#include "imu.h"
#include "arm_math.h"
#include "string.h"

#if !defined  (IMU_POTOCAL_TYPE)
  #define IMU_POTOCAL_TYPE    IMU_IIC
#endif 

#if !defined  (IMU_POSE_ANGLE)
  #define IMU_POSE_ANGLE  ((float)0) 
#endif 

#if !defined  (IMU_POSE_AX)
  #define IMU_POSE_AX    ((float)1) 
#endif 

#if !defined  (IMU_POSE_AY)
  #define IMU_POSE_AY    ((float)0) 
#endif 

#if !defined  (IMU_POSE_AZ)
  #define IMU_POSE_AZ    ((float)0) 
#endif 

#if !defined  (IMU_PID_KP)
  #define IMU_PID_KP    ((float)5) 
#endif 
	
extern SPI_HandleTypeDef hspi2;

static void imu_init(struct imu_struct *self);
static void imu_heart_beat(struct imu_struct *imu);
static void imu_updata(struct imu_struct *self);
static void imu_spi_get_data(struct imu_struct *self);
static void imu_iic_get_data(struct imu_struct *self);

static void  AccGyr_2_EulerAngle(struct imu_struct *self);
static void  RotationTransform_IMU2Pose(struct imu_struct *self);
static void  QuaternionGyr_2_WorldGyro(struct imu_struct *self);
static void  Difference_2_WorldGyro(struct imu_struct *self);
static float IMU_Lpf(float data,float data_pre,float a);
static void  IMU_LpfCenter(struct imu_struct *self);

static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

struct bmi2_dev bmi270;

BMI_t bmi_client = {

	.dev  = &bmi270,
	.init = &bmi_init,	

};


imu_t imu = {

	.bmi = &bmi_client,
	
	.info.state = IMU_OFFLINE,	
	.info.init_flag       = 0,
	.info.offline_max_cnt = 50,	
	
	//通信方式选择
	.info.tpye  = IMU_POTOCAL_TYPE,
	
	//解算使能
	.info.flag.rpy 	      = IMU_ENABLE,     //使能rpy解算
	.info.flag.wGryo      = IMU_ENABLE,    //使能世界角速度解算
	.info.flag.wGryo_type = IMU_DISENABLE,//世界角速度解算方式：四元数ENABLE  角度差分DISEABLE
	.info.flag.gyrOffset  = IMU_ENABLE,  //使能校正
	
	//imu装配位置，绕（ax，ay，az）轴旋转angle（角度制）
	.info.pose.angle = IMU_POSE_ANGLE,
	.info.pose.ax    = IMU_POSE_AX,
	.info.pose.ay    = IMU_POSE_AY,
	.info.pose.az    = IMU_POSE_AZ,
	
	//四元数收敛的pid系数  T：数据更新周期单位s
	.algo.KP = IMU_PID_KP,
	.algo.KI = 0.f,
	.algo.T  = 0.001f,
	
	//校正最大次数
	.algo.correct_cnt_max = 1000,
	
	//低通滤波系数，0-1 越大滤波越强
	.algo.ACC_LPF_a  = 0,
	.algo.GYR_LPF_a  = 0,
	.algo.RPY_LPF_a  = 0,
	.algo.wGYR_LPF_a = 0,

	.init       = &imu_init,
	.updata     = &imu_updata,
  .heart_beat = &imu_heart_beat,

};


#define IMU_GRAVITY_EARTH  (9.80665f)

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (IMU_GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale))) * (val);
}

float inVSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float imu_half_cycle(float angle,float max)
{
	if(i_abs(angle) > (max/2))
	{	
		if(angle >= 0)
			angle += -max;		
		else
			angle +=  max;
	}
	return angle;
}


/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/



void imu_init(struct imu_struct *self)
{
	int8_t rslt;

	float q0_init,q1_init,q2_init,q3_init;
	float d2r,norm;
	
	self->info.state = IMU_OFFLINE;

	self->bmi->read  = bmi2_get_regs;
  self->bmi->write = bmi2_set_regs;
	
	if(self->info.tpye == IMU_SPI){

		self->bmi->drive_type = BMI2_SPI_INTF;

	}
	else if(self->info.tpye == IMU_IIC){
	
		self->bmi->drive_type = BMI2_I2C_INTF;
		
	}
	
	rslt = self->bmi->init(self->bmi->dev,self->bmi->drive_type);

	while(rslt)
	{
			self->info.init_cycle++;
			self->info.state = IMU_INIT_ERR;
			rslt = self->bmi->init(self->bmi->dev,self->bmi->drive_type);
		
			if(self->info.init_cycle > 250)return;
	}
	
	if(!rslt){
	
		self->info.state = IMU_ONLINE;
		self->info.init_flag = 1;	
		
		if(self->info.flag.gyrOffset == IMU_ENABLE){
		
			self->info.correct = IMU_NO;
		}
		else{
		
			self->info.correct = IMU_OK;
		}
		
	  self->solve_rpy = AccGyr_2_EulerAngle;
		self->solve_lpf = IMU_LpfCenter;
		self->solve_s2p = RotationTransform_IMU2Pose;
		
		if(self->info.flag.wGryo_type == IMU_ENABLE)
			self->solve_gyr = QuaternionGyr_2_WorldGyro;
		
		if(self->info.flag.wGryo_type == IMU_DISENABLE)
			self->solve_gyr = Difference_2_WorldGyro;				
		
		//获取初始四元数
		//th = angle/180*pi/2;
		d2r = (float)self->info.pose.angle/180.f*3.141592654f/2;
		
		q0_init =                    arm_cos_f32(d2r);
		q1_init = self->info.pose.ax*arm_sin_f32(d2r);
		q2_init = self->info.pose.ay*arm_sin_f32(d2r);
		q3_init = self->info.pose.az*arm_sin_f32(d2r);
	
		norm = inVSqrt(q0_init*q0_init + q1_init*q1_init + q2_init*q2_init + q3_init*q3_init);
		q0_init = q0_init *norm;
		q1_init = q1_init *norm;
		q2_init = q2_init *norm;
		q3_init = q3_init *norm;		
		
		self->data.qInit.q0 = self->data.q.q0 = q0_init;
		self->data.qInit.q1 = self->data.q.q1 = q1_init;
		self->data.qInit.q2 = self->data.q.q2 = q2_init;
		self->data.qInit.q3 = self->data.q.q3 = q3_init;
				
		self->data.qInit.q0 = self->data.q.q0;
		self->data.qInit.q1 = self->data.q.q1;
		self->data.qInit.q2 = self->data.q.q2;
		self->data.qInit.q3 = self->data.q.q3;		

	}
	
	self->info.init_time = HAL_GetTick();
}



void imu_updata(struct imu_struct *self)
{
	static uint16_t correct_cnt = 0;	
	
	if(self->info.init_flag == 0)
		return;
	
	/*获取raw data*/
	self->data.acc_gyr_pre = self->data.acc_gyr;
	
	if(self->info.tpye == IMU_IIC)
	{
		imu_iic_get_data(self);
	
	}
	else if(self->info.tpye == IMU_SPI)
	{
		imu_spi_get_data(self);
	
	}
	
	/*零飘处理*/
	if(self->info.correct == IMU_OK){
	
		self->data.acc_gyr.gyr_x -= self->data.gyr_offset.gyr_x;
		self->data.acc_gyr.gyr_y -= self->data.gyr_offset.gyr_y;
		self->data.acc_gyr.gyr_z -= self->data.gyr_offset.gyr_z;
	
	}
	else if(self->info.correct == IMU_ING){
	
		if(correct_cnt < self->algo.correct_cnt_max)
		{
			self->data.gyr_offset.gyr_x += self->data.acc_gyr.gyr_x/(float)self->algo.correct_cnt_max;
			self->data.gyr_offset.gyr_y += self->data.acc_gyr.gyr_y/(float)self->algo.correct_cnt_max;
			self->data.gyr_offset.gyr_z += self->data.acc_gyr.gyr_z/(float)self->algo.correct_cnt_max;
		
			correct_cnt++;

		}
		else{

			if((i_abs(self->data.gyr_offset.gyr_x) > 0.01f)
			|| (i_abs(self->data.gyr_offset.gyr_y) > 0.01f)
			|| (i_abs(self->data.gyr_offset.gyr_z) > 0.01f)){
			
				self->data.gyr_offset.gyr_x = 0;
			  self->data.gyr_offset.gyr_y = 0;
			  self->data.gyr_offset.gyr_z = 0;
				
			}
			
		  self->info.correct = IMU_OK;
		}
	}	
	else if(self->info.correct == IMU_NO){
	
		correct_cnt = 0; 
		
		self->data.gyr_offset.gyr_x = 0;
		self->data.gyr_offset.gyr_y = 0;
		self->data.gyr_offset.gyr_z = 0;

		self->info.correct = IMU_ING;
	}
	
	
	
	
	
	
	/*转换装配坐标系*/
	self->solve_s2p(self);
	
	
	/*获取rpy*/
	if(self->info.flag.rpy   == IMU_ENABLE){
		
		self->data.rpy_pre = self->data.rpy;	
		self->solve_rpy(self);
		
	}
	else{
	
		memset(&self->data.rpy, 0, sizeof(self->data.rpy));
	}

	
	/*获取世界角速度*/
	if(self->info.flag.wGryo   == IMU_ENABLE){
	
		self->data.worldGyr_pre = self->data.worldGyr;
		self->solve_gyr(self);
	
	}
	else{
	
		memset(&self->data.worldGyr, 0, sizeof(self->data.worldGyr));
	}
	
	
	/*低通滤波*/
	self->solve_lpf(self);
	
	self->info.offline_cnt = 0;
}


void imu_heart_beat(struct imu_struct *imu){

	IMU_Info_t *info = &imu->info;
	
	info->offline_cnt++;
	if(info->offline_cnt > info->offline_max_cnt) 
	{
		info->offline_cnt = info->offline_max_cnt;
		info->state = IMU_OFFLINE;
	}
	else 
	{
		if(info->state == IMU_OFFLINE)
			info->state = IMU_ONLINE;
	}
	info->state = IMU_ONLINE;

}



void imu_spi_get_data(struct imu_struct *self)
{
	
	uint8_t len=12,reg = 0x0c|0x80;
	uint8_t data[len + 1];
	
	BMI_CS_LOW();	
	HAL_SPI_Transmit(&hspi2, &reg,  1, 1000);
	HAL_SPI_Receive(&hspi2, data, len+1, 1000);
	BMI_CS_HIG();
	
	for(uint8_t i=0;i<len+1;i++){
	
		data[i] = data[i+1];
	
	}
	
	self->data.rawData.acc_x	 = (int16_t)data[0] | ( (int16_t)data[1] << 8);
	self->data.rawData.acc_y	 = (int16_t)data[2] | ( (int16_t)data[3] << 8);
	self->data.rawData.acc_z	 = (int16_t)data[4] | ( (int16_t)data[5] << 8);
	
	self->data.rawData.gyr_x	= (int16_t)data[6]  | ( (int16_t)data[7] << 8);
	self->data.rawData.gyr_y  = (int16_t)data[8]  | ( (int16_t)data[9] << 8);
	self->data.rawData.gyr_z  = (int16_t)data[10] | ( (int16_t)data[11]<< 8);	

	
	
	self->data.acc_gyr.acc_x = lsb_to_mps2(self->data.rawData.acc_x,2,imu.bmi->dev->resolution);   //分钟转秒
	self->data.acc_gyr.acc_y = lsb_to_mps2(self->data.rawData.acc_y,2,imu.bmi->dev->resolution);
	self->data.acc_gyr.acc_z = lsb_to_mps2(self->data.rawData.acc_z,2,imu.bmi->dev->resolution);	
	
	self->data.acc_gyr.gyr_x = lsb_to_dps(self->data.rawData.gyr_x,2000,imu.bmi->dev->resolution) * 0.0174f;  //转弧度
	self->data.acc_gyr.gyr_y = lsb_to_dps(self->data.rawData.gyr_y,2000,imu.bmi->dev->resolution) * 0.0174f;
	self->data.acc_gyr.gyr_z = lsb_to_dps(self->data.rawData.gyr_z,2000,imu.bmi->dev->resolution) * 0.0174f;
		

}


void imu_iic_get_data(struct imu_struct *self)
{
	uint8_t data[12];
	uint8_t reg;
	
	reg = 0x0c;
	
	IIC_Read_Cicle(reg,data,12);
	
	self->data.rawData.acc_x	= (int16_t)data[0] | ( (int16_t)data[1] << 8);
	self->data.rawData.acc_y	= (int16_t)data[2] | ( (int16_t)data[3] << 8);
	self->data.rawData.acc_z	= (int16_t)data[4] | ( (int16_t)data[5] << 8);
	
	self->data.rawData.gyr_x	= (int16_t)data[6]  | ( (int16_t)data[7] << 8);
	self->data.rawData.gyr_y  = (int16_t)data[8]  | ( (int16_t)data[9] << 8);
	self->data.rawData.gyr_z  = (int16_t)data[10] | ( (int16_t)data[11]<< 8);	

	self->data.acc_gyr.acc_x = lsb_to_mps2(self->data.rawData.acc_x,2,imu.bmi->dev->resolution);   //分钟转秒
	self->data.acc_gyr.acc_y = lsb_to_mps2(self->data.rawData.acc_y,2,imu.bmi->dev->resolution);
	self->data.acc_gyr.acc_z = lsb_to_mps2(self->data.rawData.acc_z,2,imu.bmi->dev->resolution);	
	
	self->data.acc_gyr.gyr_x = lsb_to_dps(self->data.rawData.gyr_x,2000,imu.bmi->dev->resolution) * 0.0174f;  //转弧度
	self->data.acc_gyr.gyr_y = lsb_to_dps(self->data.rawData.gyr_y,2000,imu.bmi->dev->resolution) * 0.0174f;
	self->data.acc_gyr.gyr_z = lsb_to_dps(self->data.rawData.gyr_z,2000,imu.bmi->dev->resolution) * 0.0174f;
}






/*--------------------------------------------算法部分-----------------------------------------------*/
/*--------------------------------------------算法部分-----------------------------------------------*/
/*--------------------------------------------算法部分-----------------------------------------------*/
/*--------------------------------------------算法部分-----------------------------------------------*/


/*

旋转矩阵转换imu坐标2装配坐标

*/
void RotationTransform_IMU2Pose(struct imu_struct *self)
{
	
	RP_QuaternionMartix_RightMult(self->data.qInit.q0,
															  self->data.qInit.q1,
															  self->data.qInit.q2,
															  self->data.qInit.q3,
	
	                              self->data.acc_gyr.acc_x,
	                              self->data.acc_gyr.acc_y,
	                              self->data.acc_gyr.acc_z,
	
														 	 &self->data.acc_gyr.acc_x,
														 	 &self->data.acc_gyr.acc_y,
														 	 &self->data.acc_gyr.acc_z);	

	RP_QuaternionMartix_RightMult(self->data.qInit.q0,
															  self->data.qInit.q1,
															  self->data.qInit.q2,
															  self->data.qInit.q3,
	
	                              self->data.acc_gyr.gyr_x,
	                              self->data.acc_gyr.gyr_y,
	                              self->data.acc_gyr.gyr_z,
																
														 	 &self->data.acc_gyr.gyr_x,
														 	 &self->data.acc_gyr.gyr_y,
														 	 &self->data.acc_gyr.gyr_z);
}

/*

	四元数解算世界坐标系的角速度
左乘
*/
void QuaternionGyr_2_WorldGyro(struct imu_struct *self)
{

	RP_QuaternionMartix_LeftMult(self->data.q.q0,
															 self->data.q.q1,
															 self->data.q.q2,
															 self->data.q.q3,
	
	                             self->data.acc_gyr.gyr_x,
	                             self->data.acc_gyr.gyr_y,
	                             self->data.acc_gyr.gyr_z,															 
	                             &self->data.worldGyr.x,
															 &self->data.worldGyr.y,
															 &self->data.worldGyr.z);

}


/*
	四元数+PID 解算RPY


*/


/**
    @param
    @Kp
        越大表示越信任加速度，但快速晃动时，yaw轴角度可能会变化或者快速漂移。Kp越大，初始化的时候数据稳定越快。
    @Ki
        越小积分误差越小
    @halfT
        解算周期的一半，比如1ms解算1次则halfT为0.0005f

*/
float exInt,eyInt,ezInt;

void AccGyr_2_EulerAngle(struct imu_struct *self)
{ 
  float q0, q1, q2, q3;	
	float q0temp,q1temp,q2temp,q3temp; 
	
	float norm,sintemp,costemp;
	
  float vx, vy, vz;
  float gx,gy,gz,ax,ay,az;	
  float ex, ey, ez;	
	
	float Kp = self->algo.KP;
	float Ki = self->algo.KI;
	float halfT = self->algo.T/2;

	q0 = self->data.q.q0;
	q1 = self->data.q.q1;
	q2 = self->data.q.q2;
	q3 = self->data.q.q3;
	
	ax = self->data.acc_gyr.acc_x;
	ay = self->data.acc_gyr.acc_y;
	az = self->data.acc_gyr.acc_z;
	
	gx = self->data.acc_gyr.gyr_x;
	gy = self->data.acc_gyr.gyr_y;
	gz = self->data.acc_gyr.gyr_z;
	
//	if(i_abs(gz) < 0.005f)gz = 0;

	if(ax * ay *az == 0)
	{
		return;
	}		

	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 *norm;
	q1 = q1 *norm;
	q2 = q2 *norm;
	q3 = q3 *norm;	
	
	norm = inVSqrt(ax*ax + ay*ay + az*az);
	ax = ax *norm;
	ay = ay *norm;
	az = az *norm;
	
	vx = 2*(q1*q3 - q0*q2); 
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	ex = (ay*vz - az*vy) ;   
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	exInt = exInt + ex * Ki;   
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt; 
	
	q0temp=q0;  
	q1temp=q1;  
	q2temp=q2;  
	q3temp=q3; 
	
	q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
	q1 = q1temp + ( q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
	q2 = q2temp + ( q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
	q3 = q3temp + ( q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
	
	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 *norm;
	q1 = q1 *norm;
	q2 = q2 *norm;
	q3 = q3 *norm;
	
	self->data.q.q0 = q0;
	self->data.q.q1 = q1;
	self->data.q.q2 = q2;
	self->data.q.q3 = q3;
	
  /*-合成角-*/
//	self->data.rpy.roll  =  atan2(2 * q2 * q3 + 2 * q0 * q1,q0*q0 - q1 * q1 -  q2 * q2 + q3 *q3)* 57.3f;
//	self->data.rpy.pitch = -asin(2 * q1 * q3 - 2 * q0* q2)*57.3f;
//	self->data.rpy.yaw   =  atan2(2*(q1*q2 + q0*q3),q0*q0 +q1*q1-q2*q2 -q3*q3)*57.3f ;

	sintemp = 2 * q1 * q3 -2 * q0* q2;
	arm_sqrt_f32(1 - sintemp * sintemp, &costemp);
	
	arm_atan2_f32(2 * q2 * q3 + 2 * q0 * q1, q0*q0 - q1 * q1 - q2 * q2 + q3 * q3, &self->data.rpy.roll);
  arm_atan2_f32(sintemp, costemp, &self->data.rpy.pitch);
	arm_atan2_f32(2 *(q1 * q2 + q0* q3),     q0*q0 + q1 * q1 - q2 * q2 - q3 * q3, &self->data.rpy.yaw);

	self->data.rpy.roll  *=  57.295773f;
	self->data.rpy.pitch *= -57.295773f;
	self->data.rpy.yaw   *=  57.295773f;
}




/*

	差分角度解算世界坐标系的角速度

*/
void Difference_2_WorldGyro(struct imu_struct *self)
{
	float err[3];
	
	err[0] = self->data.rpy.roll  - self->data.rpy_pre.roll;
	err[1] = self->data.rpy.pitch - self->data.rpy_pre.pitch;
	err[2] = self->data.rpy.yaw   - self->data.rpy_pre.yaw;

	for(char i = 0; i < 3; i++){
	
		err[i] = imu_half_cycle(err[i],360);
	
	}

	self->data.worldGyr.x = err[0]/self->algo.T;
	self->data.worldGyr.y = err[1]/self->algo.T;
	self->data.worldGyr.z = err[2]/self->algo.T;
	
}



/*

低通滤波结算

*/
float IMU_Lpf(float data,float data_pre,float a)
{
	if(i_abs(a) > 1)a =i_abs(a)/10;
	return data*(1-a)+data_pre*a;
}

void IMU_LpfCenter(struct imu_struct *self)
{
	if(self->algo.ACC_LPF_a != 0){
		
		self->data.acc_gyr.acc_x = IMU_Lpf(self->data.acc_gyr.acc_x,self->data.acc_gyr_pre.acc_x,self->algo.ACC_LPF_a);
		self->data.acc_gyr.acc_y = IMU_Lpf(self->data.acc_gyr.acc_y,self->data.acc_gyr_pre.acc_y,self->algo.ACC_LPF_a);
	  self->data.acc_gyr.acc_z = IMU_Lpf(self->data.acc_gyr.acc_z,self->data.acc_gyr_pre.acc_z,self->algo.ACC_LPF_a);
	}
	if(self->algo.GYR_LPF_a != 0){
		
		self->data.acc_gyr.gyr_x = IMU_Lpf(self->data.acc_gyr.gyr_x,self->data.acc_gyr_pre.gyr_x,self->algo.GYR_LPF_a);
		self->data.acc_gyr.gyr_y = IMU_Lpf(self->data.acc_gyr.gyr_y,self->data.acc_gyr_pre.gyr_y,self->algo.GYR_LPF_a);
	  self->data.acc_gyr.gyr_z = IMU_Lpf(self->data.acc_gyr.gyr_z,self->data.acc_gyr_pre.gyr_z,self->algo.GYR_LPF_a);
	}	
	if(self->algo.RPY_LPF_a != 0){
		
		self->data.rpy.roll  = IMU_Lpf(self->data.rpy.roll, self->data.rpy_pre.roll, self->algo.RPY_LPF_a);
		self->data.rpy.pitch = IMU_Lpf(self->data.rpy.pitch,self->data.rpy_pre.pitch,self->algo.RPY_LPF_a);
	  self->data.rpy.yaw   = IMU_Lpf(self->data.rpy.yaw,  self->data.rpy_pre.yaw,  self->algo.RPY_LPF_a);
	}
	if(self->algo.wGYR_LPF_a != 0){
		
		self->data.worldGyr.x = IMU_Lpf(self->data.worldGyr.x,self->data.worldGyr_pre.x,self->algo.wGYR_LPF_a);
		self->data.worldGyr.y = IMU_Lpf(self->data.worldGyr.y,self->data.worldGyr_pre.y,self->algo.wGYR_LPF_a);
	  self->data.worldGyr.z = IMU_Lpf(self->data.worldGyr.z,self->data.worldGyr_pre.z,self->algo.wGYR_LPF_a);
	}	
	
}







/*-----------------------------------------------------------------------------------*/

/*
旋转 左乘
*/
void RP_QuaternionMartix_LeftMult(float q0,float q1,float q2,float q3,float x,float y,float z,float *ax,float *ay,float *az)
{
	
	float a,b,c,d;
	
	a = q0;
	b = q1;
	c = q2;
	d = q3;
	
	*ax = z*(2*a*c + 2*b*d) - y*(2*a*d - 2*b*c) + x*(a*a + b*b - c*c - d*d);
	*ay = x*(2*a*d + 2*b*c) - z*(2*a*b - 2*c*d) + y*(a*a - b*b + c*c - d*d);
	*az = y*(2*a*b + 2*c*d) - x*(2*a*c - 2*b*d) + z*(a*a - b*b - c*c + d*d);

}

/*
映射 右乘
*/
void RP_QuaternionMartix_RightMult(float q0,float q1,float q2,float q3,float x,float y,float z,float *ax,float *ay,float *az)
{
	
	float a,b,c,d;
	
	a = q0;
	b = q1;
	c = q2;
	d = q3;
	
	*ax = y*(2*a*d + 2*b*c) - z*(2*a*c - 2*b*d) + x*(a*a + b*b - c*c - d*d);
	*ay = z*(2*a*b + 2*c*d) - x*(2*a*d - 2*b*c) + y*(a*a - b*b + c*c - d*d);
	*az = x*(2*a*c + 2*b*d) - y*(2*a*b - 2*c*d) + z*(a*a - b*b - c*c + d*d);

}

void RP_Quaternion_2_EulerAngle(float q0,float q1,float q2,float q3,float *r,float *p,float *y)
{

	float norm,sintemp,costemp;

	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 *norm;
	q1 = q1 *norm;
	q2 = q2 *norm;
	q3 = q3 *norm;
	
	sintemp = 2 * q1 * q3 -2 * q0* q2;
	arm_sqrt_f32(1 - sintemp * sintemp, &costemp);
	
	arm_atan2_f32(2 * q2 * q3 + 2 * q0 * q1, q0*q0 - q1 * q1 - q2 * q2 + q3 * q3, r);
  arm_atan2_f32(sintemp, costemp, p);
	arm_atan2_f32(2 *(q1 * q2 + q0* q3),     q0*q0 + q1 * q1 - q2 * q2 - q3 * q3, y);
	
	*p = -*p; 

}

void RP_EulerAngle_2_Quaternion(float *q0,float *q1,float *q2,float *q3,double r,double p,double y)
{
    double cy = arm_cos_f32(y * 0.5);
    double sy = arm_sin_f32(y * 0.5);
    double cp = arm_cos_f32(p * 0.5);
    double sp = arm_sin_f32(p * 0.5);
    double cr = arm_cos_f32(r * 0.5);
    double sr = arm_sin_f32(r * 0.5);
 
    *q0 = cy * cp * cr + sy * sp * sr;
    *q1 = cy * cp * sr - sy * sp * cr;
    *q2 = sy * cp * sr + cy * sp * cr;
    *q3 = sy * cp * cr - cy * sp * sr;
}




//草稿纸
#if 0
		//获取旋转矩阵，imu坐标2云台坐标
//		self->data.transf[0] = q0_init*q0_init + q1_init*q1_init - q2_init*q2_init - q3_init*q3_init;
//		self->data.transf[1] = 2*(q1_init * q2_init - q0_init * q3_init);
//		self->data.transf[2] = 2*(q0_init * q2_init + q1_init * q3_init);
//		self->data.transf[3] = 2*(q0_init * q3_init + q1_init * q2_init);
//		self->data.transf[4] = q0_init*q0_init - q1_init*q1_init + q2_init*q2_init - q3_init*q3_init;
//		self->data.transf[5] = 2*(q2_init * q3_init - q0_init * q1_init);
//		self->data.transf[6] = 2*(q1_init * q3_init - q0_init * q2_init);
//		self->data.transf[7] = 2*(q0_init * q1_init + q2_init * q3_init);
//		self->data.transf[8] = q0_init*q0_init - q1_init*q1_init - q2_init*q2_init + q3_init*q3_init;
		
		
		
/*
 将自身的坐标系映射到世界
 左乘
*/
void RP_Quaternion_Self2World(float q0,float q1,float q2,float q3,float *x,float *y,float *z)
{
	
	float a,b,c,d;
	float wx,wy,wz;
	
	a = q0;
	b = q1;
	c = q2;
	d = q3;
	
	wx = *z *(2*a*c + 2*b*d) - *y *(2*a*d - 2*b*c) + *x *(a*a + b*b - c*c - d*d);
	wy = *x *(2*a*d + 2*b*c) - *z *(2*a*b - 2*c*d) + *y *(a*a - b*b + c*c - d*d);
	wz = *y *(2*a*b + 2*c*d) - *x *(2*a*c - 2*b*d) + *z *(a*a - b*b - c*c + d*d);
	
	*x = wx;
	*y = wy;
  *z = wz;

}


/*
 将世界的坐标系映射到自身
 右乘
*/
void RP_RotationTransform_IMU2Pose(float q0,float q1,float q2,float q3,float *x,float *y,float *z)
{
	float source[3];
	float solve[3];	
	float trans[9];

	arm_matrix_instance_f32 transMatrix;
  arm_matrix_instance_f32 sourceMatrix;
  arm_matrix_instance_f32 solveMatrix;

	source[0] = *x;
	source[1] = *y;
	source[2] = *z;

	//获取旋转矩阵，imu坐标2云台坐标
	trans[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	trans[1] = 2*(q1 * q2 - q0 * q3);
	trans[2] = 2*(q0 * q2 + q1 * q3);
	trans[3] = 2*(q0 * q3 + q1 * q2);
	trans[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	trans[5] = 2*(q2 * q3 - q0 * q1);
	trans[6] = 2*(q1 * q3 - q0 * q2);
	trans[7] = 2*(q0 * q1 + q2 * q3);
	trans[8] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	arm_mat_init_f32(&transMatrix,3,3,trans);
	arm_mat_init_f32(&sourceMatrix,1, 3, source);
	arm_mat_init_f32(&solveMatrix, 1, 3, solve);
	
	arm_mat_mult_f32(&sourceMatrix, &transMatrix, &solveMatrix);

	*x = solve[0];
	*y = solve[1];
	*z = solve[2];

}

#endif
#if 0
//Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
//{
//    // Abbreviations for the various angular functions
//    double cy = cos(yaw * 0.5);
//    double sy = sin(yaw * 0.5);
//    double cp = cos(pitch * 0.5);
//    double sp = sin(pitch * 0.5);
//    double cr = cos(roll * 0.5);
//    double sr = sin(roll * 0.5);
// 
//    Quaternion q;
//    q.w = cy * cp * cr + sy * sp * sr;
//    q.x = cy * cp * sr - sy * sp * cr;
//    q.y = sy * cp * sr + cy * sp * cr;
//    q.z = sy * cp * cr - cy * sp * sr;
// 
//    return q;
//}
#endif

#if 0
//	float source[6];
//	float solve[6];	

//	//前面为A矩阵，后面为B矩阵 AxB = C 以下使用右乘
//	arm_matrix_instance_f32 transMatrix;
//  arm_matrix_instance_f32 sourceMatrix;
//  arm_matrix_instance_f32 solveMatrix;

//	memcpy(source,&self->data.acc_gyr,sizeof(self->data.acc_gyr));

//	arm_mat_init_f32(&transMatrix,3,3,self->data.transf);
//	
//	arm_mat_init_f32(&sourceMatrix,1, 3, source);
//	arm_mat_init_f32(&solveMatrix, 1, 3, solve);
//	arm_mat_mult_f32(&sourceMatrix, &transMatrix, &solveMatrix);

//	arm_mat_init_f32(&sourceMatrix,1, 3, source+3);
//	arm_mat_init_f32(&solveMatrix, 1, 3, solve+3);
//	arm_mat_mult_f32(&sourceMatrix, &transMatrix, &solveMatrix);

//	memcpy(&self->data.acc_gyr,solve,sizeof(self->data.acc_gyr));

#endif
#if 0

//	self->data.rpy.roll  =  atan2(2 * q2 * q3 + 2 * q0 * q1,q0*q0 - q1 * q1 -  q2 * q2 + q3 *q3)* 57.3f;
//	self->data.rpy.pitch = -asin(2 * q1 * q3 - 2 * q0* q2)*57.3f;
//	self->data.rpy.yaw   =  atan2(2*(q1*q2 + q0*q3),q0*q0 +q1*q1-q2*q2 -q3*q3)*57.3f ;
  //asin(x) = atan(x/sqrt(1-x*x))

/**
    @brief  坐标变换采用Z-Y-X欧拉角描述，即从陀螺仪坐标系向云台坐标系变换中，坐标系按照绕陀螺仪Z轴、Y轴、X轴的顺序旋转
						每一次旋转的参考坐标系为当前陀螺仪坐标系
    @param
    @arz
        陀螺仪x轴与roll轴之间的夹角，单位为度
    @ary
        陀螺仪x轴与yaw轴之间的夹角，单位为度
    @arx
        陀螺仪y轴与yaw轴之间的夹角，单位为度
*/
//float q0_init = 0.0f, q1_init = 1.0f, q2_init = 0.0f, q3_init = 0.0f;
float arz_ = -90.0f;
float ary_ = 0.0f;
float arx_ = 180.0f;
float arz, ary, arx;
arm_matrix_instance_f32 Trans;
arm_matrix_instance_f32 Src;
arm_matrix_instance_f32 Dst;
float trans[9];
float gyro_in[3];
float gyro_out[3];
float acc_in[3];
float acc_out[3];
/**
  * @brief  陀螺仪坐标变换初始化，若不需要变换可在imu.c中imu_init将其注释
  * @param  
  * @retval 
  */
void transform_init(void)
{
	/* 角度单位转换（to弧度） */
	arz = arz_ * (double)0.017453;
	ary = ary_ * (double)0.017453;
	arx = arx_ * (double)0.017453;

	/* 旋转矩阵赋值（三个旋转矩阵叠加） */
	trans[0] = arm_cos_f32(arz)*arm_cos_f32(ary);
	trans[1] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) - arm_sin_f32(arz)*arm_cos_f32(arx);
	trans[2] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) + arm_sin_f32(arz)*arm_sin_f32(arx);
	trans[3] = arm_sin_f32(arz)*arm_cos_f32(ary);
	trans[4] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) + arm_cos_f32(arz)*arm_cos_f32(arx);
	trans[5] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) - arm_cos_f32(arz)*arm_sin_f32(arx);
	trans[6] = -arm_sin_f32(ary);
	trans[7] = arm_cos_f32(ary)*arm_sin_f32(arx);
	trans[8] = arm_cos_f32(ary)*arm_cos_f32(arx);
	
	arm_mat_init_f32(&Trans, 3, 3, (float *)trans); //3x3变换矩阵初始化
}

/**
  * @brief  将陀螺仪坐标变换为云台坐标，若不需要变换可在imu.c中imu_update将其注释
  * @brief  坐标变换采用Z-Y-X欧拉角描述，即从陀螺仪坐标系向云台坐标系变换中，坐标系按照绕陀螺仪Z轴、Y轴、X轴的顺序旋转
	*					每一次旋转的参考坐标系为当前陀螺仪坐标系
  * @param[in]  (int16_t) gx,  gy,  gz,  ax,  ay,  az
  * @param[out] (float *) ggx, ggy, ggz, aax, aay, aaz
	*/
void Vector_Transform(int16_t gx, int16_t gy, int16_t gz,\
	                    int16_t ax, int16_t ay, int16_t az,\
	                    float *ggx, float *ggy, float *ggz,\
											float *aax, float *aay, float *aaz)
{
	/* 陀螺仪赋值 */
	gyro_in[0] = (float)gx, gyro_in[1] = (float)gy, gyro_in[2] = (float)gz;
	
	/* 加速度计赋值 */
	acc_in[0] = (float)ax, acc_in[1] = (float)ay, acc_in[2] = (float)az;
	
	/* 陀螺仪坐标变换 */
	arm_mat_init_f32(&Src, 1, 3, gyro_in);
	arm_mat_init_f32(&Dst, 1, 3, gyro_out);
	arm_mat_mult_f32(&Src, &Trans, &Dst);
	*ggx = gyro_out[0], *ggy = gyro_out[1], *ggz = gyro_out[2];
	
	/* 加速度计坐标变换 */
	arm_mat_init_f32(&Src, 1, 3, acc_in);
	arm_mat_init_f32(&Dst, 1, 3, acc_out);
	arm_mat_mult_f32(&Src, &Trans, &Dst);
	*aax = acc_out[0], *aay = acc_out[1], *aaz = acc_out[2];
	
}

#endif






#if 0
//extern TIM_HandleTypeDef htim4;
////1->1us;
//void TIM4_Init_Handle(uint16_t timeout)
//{

//  /* USER CODE BEGIN TIM4_Init 0 */

//  /* USER CODE END TIM4_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM4_Init 1 */

//  /* USER CODE END TIM4_Init 1 */
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = timeout * 84/2 - 1;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 2 - 1;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
//  {
////    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
//  {
////    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
////    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM4_Init 2 */
//	
//  /* USER CODE END TIM4_Init 2 */
//	
//}
#endif
