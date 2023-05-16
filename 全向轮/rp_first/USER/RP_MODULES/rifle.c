
#include "DEVICES.h"
#include "RP_FUNCTION.h"
#include "rifle.h"



void Rifle_ModifyLock(rifle *self,rifle_Lock type);
void Rifle_ModifyFri(rifle *self,rifle_state state);
void Rifle_ModifyMagazine(rifle *self,rifle_state state);
void Rifle_ModifyShootType(rifle *self,rifle_ShootType type,uint16_t number);

void Rifle_Updata(rifle *self);
void Rifle_BoxCtrl(rifle *self);
void Rifle_FrictionCtrl(rifle *self);
void Rifle_MagazineCtrl(rifle *self);


rifle gun = {


	.info.MotorState    = RIFLE_MOTOR_ERR,
	.info.Lock          = RIFLE_LOCK,
	.info.FriEnable     = RIFLE_NO,
  .info.FriSpeedReach = RIFLE_NO,
	.info.FriStucking   = RIFLE_NO,
	.info.BoxStucking   = RIFLE_NO,	
	.info.HeatEnable    = RIFLE_NO,	
	.info.ShootReady    = RIFLE_NO,
	.info.Shooting      = RIFLE_NO,
	.info.Magazine      = RIFLE_NO,
	
	.friTable.Speed0  = 0,
  .friTable.Speed15 = 4400,
	.friTable.Speed18 = 4700,
	.friTable.Speed20 = 4950,
	.friTable.Speed22 = 5200,
	.friTable.Speed30 = 7020,

	.ModifyLock = Rifle_ModifyLock,
	.ModifyFri  = Rifle_ModifyFri,
	.ModifyMagazine  = Rifle_ModifyMagazine,
	.ModifyShootType = Rifle_ModifyShootType,
	
	.Updata  = Rifle_Updata,
	.BoxCtrl = Rifle_BoxCtrl ,
	.FrictionCtrl = Rifle_FrictionCtrl,
	.MagazineCtrl = Rifle_MagazineCtrl,
};


/** @FUN  修改允许控制状态
  * @type RIFLE_UNLOCK  RIFLE_LOCK
  */
void Rifle_ModifyLock(rifle *self,rifle_Lock type)
{
	if(type == RIFLE_LOCK && self->info.Lock == RIFLE_UNLOCK){
	
		self->info.Lock = RIFLE_LOCK;
		self->time.LockTime = HAL_GetTick();
	}
	else if(type == RIFLE_UNLOCK && self->info.Lock == RIFLE_LOCK){
	
		self->info.Lock = RIFLE_UNLOCK;
		self->time.unLockTime = HAL_GetTick();
	}	
}
 
/** @FUN  使能摩擦轮
  * @state RIFLE_OK RIFLE_NO
  */
void Rifle_ModifyFri(rifle *self,rifle_state state)
{
	if(state != self->info.FriEnable && state == RIFLE_OK){
	
		self->time.FriEnableTime = HAL_GetTick();	
	}

	self->info.FriEnable = state;

}

/** @FUN 修改发射模式
  * @number 连射时是拨盘速度 指定数量时是子弹数 
  */
void Rifle_ModifyShootType(rifle *self,rifle_ShootType type,uint16_t number)
{
	//处于发射中禁止修改发射类型
	if(self->info.ShootType == RIFLE_SHOOT_SET && self->info.Shooting == RIFLE_ING){
	
		return;
	}	
	
	self->info.ShootType = type;
	self->data.BoxNumSet = number;

}

/** @FUN  弹仓状态
  * @state 关闭 RIFLE_OK  未关闭 RIFLE_NO
  */
void Rifle_ModifyMagazine(rifle *self,rifle_state state)
{
	if(self->info.Magazine != state){
	
		self->info.Magazine = state;
		return;
	}
}






/** @FUN  更新数据
  * 
  */
void Rifle_Updata(rifle *self)
{
	if(motor[FRI_R].state.work_state == M_ONLINE && motor[FRI_L].state.work_state == M_ONLINE && motor[BOX].state.work_state == M_ONLINE){
			
			self->info.MotorState = RIFLE_MOTOR_ONLINE;
	}
	else{
	
		self->info.MotorState = RIFLE_MOTOR_ERR;
	}

	self->data.FriTempL = motor[FRI_L].rx_info.temperature;
	self->data.FriTempR = motor[FRI_R].rx_info.temperature;
	
	self->data.FriRpmL = motor[FRI_L].rx_info.speed;
	self->data.FriRpmR = motor[FRI_R].rx_info.speed;	
	
	self->data.BoxTorque   = motor[BOX].rx_info.torque;	
	self->data.BoxSpeed    = motor[BOX].rx_info.speed;
	self->data.BoxPosition = motor[BOX].rx_info.angle_sum;

	
	self->data.SpeedLimitPre = self->data.SpeedLimit;
	self->data.SpeedLimit    = judge.data.game_robot_status.shooter_id1_17mm_speed_limit;
	
	self->data.HeatLimit     = judge.data.game_robot_status.shooter_id1_17mm_cooling_limit;

	self->data.Heat = judge.data.power_heat_data.shooter_id1_17mm_cooling_heat;
	
	self->data.SpeedPre = self->data.Speed;
	self->data.Speed    = judge.data.shoot_data.bullet_speed;


	if(self->data.SpeedLimitPre != self->data.SpeedLimit){
	
		self->data.SpeedLowNum  = 0;	

		self->data.SpeedUpNum   = 0;	
		self->data.SpeedDownNum = 0;	
	}
	
	//由于裁判系统未断电时会保留数据，可能多计算一颗
	if(self->data.SpeedPre != self->data.Speed && self->data.Speed){
	
		self->data.BulletNum++;
		
		//速度过低
		if(self->data.Speed < self->data.SpeedLimit - 2.f){
		
			self->data.SpeedLowNum++;
		}
		//超速
		if(self->data.Speed >= self->data.SpeedLimit){
		
			self->data.SpeedHightCom = self->data.SpeedHightCom + (self->data.SpeedLimit - self->data.Speed - 1)*120;
		
			self->data.SpeedLowNum = 0;
		}
		//微调
		if(self->data.Speed < self->data.SpeedLimit - 1.05f){
		
			self->data.SpeedUpNum++;
			self->data.SpeedDownNum = 0;
		}
		else if(self->data.Speed > self->data.SpeedLimit - 0.95f){
		
			self->data.SpeedUpNum = 0;
			self->data.SpeedDownNum++;
		}
	}
	
	//判断摩擦轮速度是否达到 摩擦轮是否达堵住
	if(self->info.FriEnable == RIFLE_OK){
	
		if(abs(self->data.FriSet - self->data.FriRpmR) < 800 && abs(self->data.FriSet - self->data.FriRpmL) < 800){
		
				self->info.FriSpeedReach = RIFLE_OK;
		}
		else{
		
			self->info.FriSpeedReach = RIFLE_NO;
			
			if(HAL_GetTick() - self->time.FriEnableTime > 2000){
			
				self->info.FriStucking = RIFLE_OK;
			}
		}
	}
	else{
	
		self->info.FriStucking = RIFLE_NO;
		self->info.FriSpeedReach = RIFLE_NO;
	}

	
	
	//判断热量允许
	if(judge.info.state == JUDGE_ONLINE){
		
		self->data.HeatEnableNum = (self->data.HeatLimit - self->data.Heat)/10;
		
		if(self->data.HeatEnableNum > 1){
		
			self->info.HeatEnable = RIFLE_OK;
		}
		else{
		
			self->info.HeatEnable = RIFLE_NO;
		}
	
	}
	else{
	
		self->data.HeatEnableNum = 999;
		
		self->info.HeatEnable = RIFLE_OK;
	}
	
	if(self->info.Shooting == RIFLE_ING){
	
		if(abs(self->data.BoxSpeed < 50) && abs(self->data.BoxTorque > 5000)){
		
			if(HAL_GetTick() - self->time.StuckJudegStart > 400){
			
				self->info.BoxStucking = RIFLE_OK;
			}
		}	
		else{

			self->time.StuckJudegStart = HAL_GetTick();
		}
	}
	
	if(self->info.BoxStucking == RIFLE_NO){
	
		self->time.StuckStart = HAL_GetTick();
	}
	else{
	
		if(HAL_GetTick() - self->time.StuckStart > 2000){
		
			self->info.BoxStucking = RIFLE_NO;
		}
	}
	
	
	
	//判断是否达到射击条件
	if(self->info.FriSpeedReach == RIFLE_OK && self->info.BoxStucking == RIFLE_OK && self->info.HeatEnable == RIFLE_OK){
	
		self->info.ShootReady = RIFLE_OK;
	}
}

/** @FUN  摩擦轮控制 设定速度 温度调整 速度反馈
  * 
  */
void Rifle_FrictionCtrl(rifle *self)
{
	if(self->info.FriEnable == RIFLE_NO){
		
		self->data.FriSet = self->friTable.Speed0;
		return;
	}
	else if(self->info.FriEnable == RIFLE_OK){
	
		switch(self->data.SpeedLimit){
		
			case 0:
				self->data.FriSet = self->friTable.Speed0;
				break;
			case 15:
				self->data.FriSet = self->friTable.Speed15;
				break;
			case 18:
				self->data.FriSet = self->friTable.Speed18;
				break;		
			case 20:
				self->data.FriSet = self->friTable.Speed20;
				break;	
			case 22:
				self->data.FriSet = self->friTable.Speed22;
				break;			
			case 30:
				self->data.FriSet = self->friTable.Speed30;
				break;	
			default:
				if(self->data.SpeedLimit < 35 && self->data.SpeedLimit){
				
					self->data.FriSet = self->friTable.Speed15 + (self->data.SpeedLimit - 15) * 110;
				}
				else{
				
					self->data.FriSet = self->friTable.Speed30;
				}
				break;
		}
		
		if(judge.info.state == JUDGE_ONLINE){
		
			self->data.FriSet = self->data.FriSet
													+ self->data.SpeedLowNum/3*120
													+ self->data.SpeedHightCom
													- self->data.SpeedUpNum/2*5
													+ self->data.SpeedDownNum/2*5;
		}
		
		//温度，取平均值
		self->data.FriSet = self->data.FriSet - 80 * (self->data.FriTempL+self->data.FriTempR)/20;

	}
}


#if 0

/** @FUN  发射模式设置 指定数量 连射
  * @number 连射时是拨盘速度 指定数量时是子弹数 
  * @note 优先级：堵转 位置 速度
  *       位置控制结束后要手动将状态改为停止状态才能进行下一次射击
  */
void Rifle_ShootSet(rifle *self,rifle_ShootType type,uint16_t number)
{

	//堵转处理中，优先处理
	if(self->info.BoxStucking == RIFLE_ING){
	
		if(abs(self->data.BoxPositionSet - self->data.BoxPosition) < 100){
		
			self->info.BoxStucking = RIFLE_NO;
		}
		
		return;
	}
	
	//发生堵转，设定位置
	if(self->info.BoxStucking == RIFLE_OK){
	
		self->info.Shooting = RIFLE_NO;
		
		if(self->data.BoxPositionSet < 0 || self->data.BoxSpeedSet < 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition + 8191 * 4.5f;
		}
		else if(self->data.BoxPositionSet > 0 || self->data.BoxSpeedSet > 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f;
		}
		self->info.BoxStucking = RIFLE_ING;
		
		self->info.ShootType = RIFLE_SHOOT_STUCK;
		
		return;
	}
	
	//不发生堵转情况，正常运行
	
	//不满足控制条件 已完成射击 立刻停止
	if(self->info.ShootReady == RIFLE_NO || self->info.Shooting == RIFLE_OK){
	
		type = RIFLE_SHOOT_STOP;
		
	}

	//满足控制条件

	//处于位置发射状态 禁止打断 
	if(self->info.ShootType == RIFLE_SHOOT_SET && self->info.Shooting == RIFLE_ING){
	
		if(abs(self->data.BoxPositionSet - self->data.BoxPosition) < 100){
		
			self->info.Shooting = RIFLE_OK;
		}		
		return;
	}

	//不处于位置发射状态

	//控制类型：停止
	if(type == RIFLE_SHOOT_STOP){
	
		self->data.BoxSpeedSet = 0;
		self->info.Shooting = RIFLE_NO;

		self->info.ShootType = RIFLE_SHOOT_STOP;
	}
	else{

		//1、速度控制
		if(type == RIFLE_SHOOT_STAY){

			self->info.Shooting = RIFLE_ING;

			self->data.BoxSpeedSet = -number;
			
			self->info.ShootType = RIFLE_SHOOT_STAY;

			return;
		}


		//2、位置控制
		if(type == RIFLE_SHOOT_SET && self->info.Shooting == RIFLE_NO){
			
			self->info.Shooting = RIFLE_ING;
			
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f * number;

			self->info.ShootType = RIFLE_SHOOT_SET;
			
		}	
	}
}
#endif

/** @FUN  发射模式
  * @note 优先级：堵转 位置 速度
  *       
  */
void Rifle_BoxCtrl(rifle *self)
{

	//堵转处理中，优先处理
	if(self->info.BoxStucking == RIFLE_ING){
	
		if(abs(self->data.BoxPositionSet - self->data.BoxPosition) < 100){
		
			self->info.BoxStucking = RIFLE_NO;
		}
		
		return;
	}
	
	//发生堵转，设定位置
	if(self->info.BoxStucking == RIFLE_OK){
	
		self->info.Shooting = RIFLE_NO;
		
		if(self->data.BoxPositionSet < 0 || self->data.BoxSpeedSet < 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition + 8191 * 4.5f;
		}
		else if(self->data.BoxPositionSet > 0 || self->data.BoxSpeedSet > 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f;
		}
		self->info.BoxStucking = RIFLE_ING;
		
		self->info.ShootType = RIFLE_SHOOT_STUCK;
		
		return;
	}
	
	//不发生堵转情况，正常运行
	
	//不满足控制条件 已完成射击 立刻停止
	if(self->info.ShootReady == RIFLE_NO || self->info.Shooting == RIFLE_OK){
	
		self->info.ShootType = RIFLE_SHOOT_STOP;
		self->info.Shooting  = RIFLE_NO;
		
	}

	//满足控制条件

	//处于位置发射状态 禁止打断 完成后Shooting = RIFLE_OK
	if(self->info.ShootType == RIFLE_SHOOT_SET && self->info.Shooting == RIFLE_ING){
		
		if(abs(self->data.BoxPositionSet - self->data.BoxPosition) < 100){
		
			self->info.Shooting = RIFLE_OK;
		}		
		else if(abs(self->data.BoxPositionSet - self->data.BoxPosition) < 8192*4.5f*2){
		
			self->info.BoxType = RIFLE_POSITION;
		}
		
		return;
	}

	//不处于位置发射状态

	//控制类型：停止输出
	if(self->info.ShootType == RIFLE_SHOOT_STOP){
	
		self->data.BoxSpeedSet = 0;
		self->info.Shooting = RIFLE_NO;
		self->info.BoxType = RIFLE_SPEED;

	}
	else{

		//1、持续输出
		if(self->info.ShootType == RIFLE_SHOOT_STAY){

			self->info.Shooting = RIFLE_ING;

			self->data.BoxSpeedSet = -self->data.BoxNumSet;

			self->info.BoxType = RIFLE_SPEED;
			
			return;
		}


		//2、指定数量
		if(self->info.ShootType == RIFLE_SHOOT_SET){
			
			self->info.Shooting = RIFLE_ING;
			
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f * self->data.BoxNumSet;

			self->data.BoxSpeedSet = 6500;
			
			//输量大于等于3的时候先使用速度控制
			if(self->data.BoxNumSet > 2){
						
				self->info.BoxType = RIFLE_SPEED;
			}
			else{
			
				self->info.BoxType = RIFLE_POSITION;
			}
		}	
	}
}
/** @FUN  电机控制 存在电机失联时 拨盘不控制
  *  
  */
void Rifle_Ctrl(rifle *self)
{
	int16_t Rifle_CANBuff[4];
	
	if(self->info.Lock == RIFLE_LOCK)
	{
		if(HAL_GetTick() - self->time.LockTime < 1000){
		
			Rifle_CANBuff[motor[FRI_R].id.buff_p] = motor[FRI_R].c_speed(&motor[FRI_R],0);
			Rifle_CANBuff[motor[FRI_L].id.buff_p] = motor[FRI_L].c_speed(&motor[FRI_L],0);
			
			Rifle_CANBuff[motor[BOX].id.buff_p] = motor[BOX].c_speed(&motor[BOX],0);			
		}

		motor[FRI_R].tx(&motor[FRI_R],Rifle_CANBuff);	
		
		return;
	}
	else if(self->info.Lock == RIFLE_UNLOCK)
	{
		Rifle_CANBuff[motor[FRI_R].id.buff_p] = motor[FRI_R].c_speed(&motor[FRI_R], self->data.FriSet);
		Rifle_CANBuff[motor[FRI_L].id.buff_p] = motor[FRI_L].c_speed(&motor[FRI_L],-self->data.FriSet);
		
		if(self->info.BoxType == RIFLE_SPEED){
		
			Rifle_CANBuff[motor[BOX].id.buff_p] = motor[BOX].c_speed(&motor[BOX],self->data.BoxSpeedSet);				
		}
		else if(self->info.BoxType == RIFLE_POSITION){
		
			Rifle_CANBuff[motor[BOX].id.buff_p] = motor[BOX].c_posit(&motor[BOX],self->data.BoxPositionSet);	
		}
		
		if(self->info.MotorState == RIFLE_MOTOR_ERR){
		
			Rifle_CANBuff[motor[BOX].id.buff_p] = 0;
		}

		motor[FRI_R].tx(&motor[FRI_R],Rifle_CANBuff);	
		
		return;
	}
}

void Rifle_MagazineCtrl(rifle *self)
{
	if(self->info.Magazine == RIFLE_OK){
	
		
		return;
	}
	else if(self->info.Magazine == RIFLE_NO){
	
		
		return;
	}
}







