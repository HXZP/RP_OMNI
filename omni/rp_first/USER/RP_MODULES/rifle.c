/*

	舵机目前使用PA8


*/

#include "rifle.h"
#include "DEVICES.h"
#include "RP_FUNCTION.h"
#include "RP_CONFIG.h"


#define MAGAZINE_CCR_CLOSE 48
#define MAGAZINE_CCR_OPEN  127




void Rifle_ModifyLock(rifle *self,rifle_Lock type);

void Rifle_ModifyState(rifle_state *self,rifle_state state);

void Rifle_ModifyFri(rifle *self,rifle_state state);
void Rifle_ModifyMagazine(rifle *self,rifle_state state);
void Rifle_ModifyShootType(rifle *self,rifle_ShootType type,uint16_t number);

void Rifle_Updata(rifle *self);
void Rifle_Resolving(rifle *self);
void Rifle_BoxCtrl(rifle *self);
void Rifle_FrictionCtrl(rifle *self);
void Rifle_MagazineCtrl(rifle *self);
void Rifle_Ctrl(rifle *self);

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
	
	.info.AutoSetEnable = RIFLE_NO,
	
	.friTable.Speed0  = 0,
  .friTable.Speed15 = 4400,
	.friTable.Speed18 = 4700,
	.friTable.Speed20 = 4950,
	.friTable.Speed22 = 5200,
	.friTable.Speed30 = 7020,

	.ModifyLock = Rifle_ModifyLock,
	
	.ModifyState = Rifle_ModifyState,
	
	.ModifyFri  = Rifle_ModifyFri,
	.ModifyMagazine  = Rifle_ModifyMagazine,
	.ModifyShootType = Rifle_ModifyShootType,
	
	.Updata    = Rifle_Updata,
	.Resolving = Rifle_Resolving,
	.Ctrl      = Rifle_Ctrl,
	
	.BoxCtrl = Rifle_BoxCtrl,
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
 

/** @FUN  修改状态
  * @state  RIFLE_OK   RIFLE_NO
  */
void Rifle_ModifyState(rifle_state *self,rifle_state state)
{
	if(*self != state){
	
		*self = state;

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

	}
}

/** @FUN  单发模式
  * @state 关闭 RIFLE_OK  未关闭 RIFLE_NO
  */
void Rifle_ModifyAutoSet(rifle *self,rifle_state state)
{
	if(self->info.AutoSetEnable != state){
	
		self->info.AutoSetEnable = state;

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
	
	if(self->info.MotorState == RIFLE_MOTOR_ERR){
		
		self->ModifyLock(self,RIFLE_LOCK);
	}

	self->data.FriTempL = motor[FRI_L].rx_info.temperature;
	self->data.FriTempR = motor[FRI_R].rx_info.temperature;
	
	self->data.FriRpmL = motor[FRI_L].rx_info.speed;
	self->data.FriRpmR = motor[FRI_R].rx_info.speed;	
	
	self->data.BoxTorque   = motor[BOX].rx_info.torque;	
	self->data.BoxSpeed    = motor[BOX].rx_info.speed;
	self->data.BoxPosition = motor[BOX].rx_info.angle_sum;

	self->data.BoxSpeedErr = self->data.BoxSpeedSet - self->data.BoxSpeed;
	self->data.BoxPositionErr = self->data.BoxPositionSet - self->data.BoxPosition;	
	
	self->data.SpeedLimitPre = self->data.SpeedLimit;
	self->data.SpeedPre = self->data.Speed;
	
	//裁判系统在线处理
	if(judge.info.state == JUDGE_ONLINE){

		self->data.HeatLimit = judge.data.game_robot_status.shooter_id1_17mm_cooling_limit;

		self->data.Heat = judge.data.power_heat_data.shooter_id1_17mm_cooling_heat;	

		self->data.SpeedLimit = judge.data.game_robot_status.shooter_id1_17mm_speed_limit;

		self->data.Speed = judge.data.shoot_data.bullet_speed;

		self->data.HeatEnableNum = (self->data.HeatLimit - self->data.Heat)/10;

	}
	//裁判系统不在线处理
	else{
	
		self->data.HeatEnableNum = 999;
		
		self->data.SpeedLimit = 15;
		
	}
	//忽略热量
	if(self->info.IgnoreHeat == RIFLE_OK){
	
		self->data.HeatEnableNum = 999;
	
	}
	
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
		
		//统计
		self->data.SpeedDistribute[(uint16_t)(self->data.Speed)]++;
		
		
		
	}
	
	//判断摩擦轮速度是否达到 摩擦轮是否达堵住
	if(self->info.FriEnable == RIFLE_OK){
	
		if(abs(self->data.FriSet - abs(self->data.FriRpmR)) < 800 && abs(self->data.FriSet - abs(self->data.FriRpmL)) < 800){
		
				self->info.FriSpeedReach = RIFLE_OK;
			  self->info.FriStucking   = RIFLE_NO;
		}
		else{
		
			self->info.FriSpeedReach = RIFLE_NO;
			
			//太久没到达认为摩擦轮堵住
			if(HAL_GetTick() - self->time.FriEnableTime > 1000){
			
				self->info.FriStucking = RIFLE_OK;
			}
		}
	}
	else{
	
		self->info.FriStucking   = RIFLE_NO;
		self->info.FriSpeedReach = RIFLE_NO;
	}

	//判断热量允许
	if(self->data.HeatEnableNum > 1){
	
		self->info.HeatEnable = RIFLE_OK;
	}
	else{
	
		self->info.HeatEnable = RIFLE_NO;
	}
		
	
	//堵转判断
	if(self->info.Shooting == RIFLE_ING){
	
		if(abs(self->data.BoxSpeed) < 50 && abs(self->data.BoxTorque) > 5000){
		
			if(HAL_GetTick() - self->time.StuckJudegStart > 400){
			
				self->info.BoxStucking = RIFLE_OK;
				
				self->time.StuckJudegStart = HAL_GetTick();
			}
		}
	}
	else{
	
		self->time.StuckJudegStart = HAL_GetTick();
	}
	
	//堵转过久处理
	if(self->info.BoxStucking == RIFLE_NO){
	
		self->time.StuckStart = HAL_GetTick();
	}
	else{
	
		if(HAL_GetTick() - self->time.StuckStart > 1000){
		
			self->info.BoxStucking = RIFLE_NO;
		}
	}
	
	//判断是否达到射击条件
	if(self->info.FriSpeedReach == RIFLE_OK && self->info.BoxStucking == RIFLE_NO && self->info.HeatEnable == RIFLE_OK){
	
			self->info.ShootReady = RIFLE_OK;
	}
	else{
	
		self->info.ShootReady = RIFLE_NO;
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

/** @FUN  发射模式
  * @note 优先级：堵转 发射未就绪 位置发射状态 速度发射 发射完成状态
  *       
  */

//float sda;
void Rifle_BoxCtrl(rifle *self)
{
	float set,mes,err;
	
	set = self->data.BoxPositionSet;
	mes = self->data.BoxPosition;
	
	/* 发生堵转情况，异常 */
	
	//堵转处理中，优先处理
	if(self->info.BoxStucking == RIFLE_ING){
	
		//堵转处理结束 关闭标志位 关闭发射状态 立刻切到速度控制状态
		if(abs(self->data.BoxPositionSet - self->data.BoxPosition) < 100){
		
			self->info.BoxStucking = RIFLE_NO;
			
		  self->data.BoxSpeedSet = 0;
		
//		  self->info.Shooting = RIFLE_NO;
		
		  self->info.BoxType = RIFLE_SPEED;			
		}
		
		return;
	}
	
	//发生堵转，设定位置
	if(self->info.BoxStucking == RIFLE_OK){
	
		if(self->data.BoxPositionSet - self->data.BoxPosition < 0 || self->data.BoxSpeedSet < 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition + 8191 * 4.5f;
		}
		else if(self->data.BoxPositionSet - self->data.BoxPosition > 0 || self->data.BoxSpeedSet > 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f;
		}
		
		self->info.BoxStucking = RIFLE_ING;
		
		self->info.ShootType = RIFLE_SHOOT_STUCK;
		
		self->info.BoxType = RIFLE_POSITION;
		
//		self->time.StuckJudegStart = HAL_GetTick();
		
		return;
	}
	
	/* 不发生堵转 */
	
	//未就绪
	if(self->info.ShootReady == RIFLE_NO){
	
		self->info.ShootType = RIFLE_SHOOT_STOP;
		self->info.Shooting  = RIFLE_NO;

	}
	//位置控制未结束 禁止打断 完成后Shooting = RIFLE_OK 立刻返回不做其他决策
	else if(self->info.ShootType == RIFLE_SHOOT_SET && self->info.Shooting == RIFLE_ING){
		
		err = abs(set - mes);

		if(err < 1000){

			self->info.Shooting = RIFLE_OK;
		}		
		else if(err < 8192*4.5f*2){

			self->info.BoxType = RIFLE_POSITION;
		}

		if(HAL_GetTick() - self->time.ShootSetTime > 500){

			self->info.Shooting = RIFLE_OK;
		}
		
		return;
	}
	//持续控制
	else if(self->info.ShootType == RIFLE_SHOOT_STAY){
	
	}	
	//位置控制结束
	else if(self->info.ShootType == RIFLE_SHOOT_SET && self->info.Shooting == RIFLE_OK){
	
		self->info.ShootType = RIFLE_SHOOT_STOP;
		
		if(self->info.AutoSetEnable == RIFLE_OK){
		
			self->info.Shooting  = RIFLE_NO;
		}
		else if(self->info.AutoSetEnable == RIFLE_NO){
		
			self->info.Shooting  = RIFLE_OK;
		}
		
		return;

	}


	
	/* 控制类型决策 */
	
	//0、停止输出
	if(self->info.ShootType == RIFLE_SHOOT_STOP){
	
		self->data.BoxSpeedSet = 0;
		
		self->info.Shooting = RIFLE_NO;
		
		self->info.BoxType = RIFLE_SPEED;
		
		return;
	}
	
//	//处于发射中，禁止修改目标
//	if(self->info.Shooting != RIFLE_NO){
//	
//		return;
//	}

	//1、持续输出
	else if(self->info.ShootType == RIFLE_SHOOT_STAY){

		self->info.Shooting = RIFLE_ING;

		self->data.BoxSpeedSet = -self->data.BoxNumSet;

		self->info.BoxType = RIFLE_SPEED;
		
		return;
	}
	//2、指定数量
	else if(self->info.ShootType == RIFLE_SHOOT_SET){
		
		self->info.Shooting = RIFLE_ING;
		
		if(self->data.BoxNumSet >= self->data.HeatEnableNum){
		
			self->data.BoxNumSet = self->data.HeatEnableNum - 1;	
		}
		
		if(self->data.BoxNumSet < 0){
		
			self->data.BoxNumSet = 0;
		}
		
		
		self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f * self->data.BoxNumSet;

		self->data.BoxSpeedSet = -6500;
		
		//输量大于等于3的时候先使用速度控制
		if(self->data.BoxNumSet > 2){
					
			self->info.BoxType = RIFLE_SPEED;
		}
		else{
		
			self->info.BoxType = RIFLE_POSITION;
		}
		
		self->time.ShootSetTime = HAL_GetTick();
	}
}



void Rifle_MagazineCtrl(rifle *self)
{
	if(self->info.Magazine == RIFLE_OK){
	
		self->data.MagazineCCR = MAGAZINE_CCR_OPEN;
//		magazine.modifyCCR(&magazine,48);
		return;
	}
	else if(self->info.Magazine == RIFLE_NO){
	
		self->data.MagazineCCR = MAGAZINE_CCR_CLOSE;
//		magazine.modifyCCR(&magazine,127);
		return;
	}
}


/* @FUN 所有逻辑解算
 * 
 **/
void Rifle_Resolving(rifle *self)
{
	self->FrictionCtrl(self);
	self->MagazineCtrl(self);
	self->BoxCtrl(self);
}



/** @FUN  电机控制 存在电机失联时 拨盘不控制
  *  
  */

void Rifle_Ctrl(rifle *self)
{
  int16_t Rifle_CANBuff[4] = {0,0,0,0};

	
	if(self->info.Lock == RIFLE_LOCK)
	{
		
		if(HAL_GetTick() - self->time.LockTime < 1000){
		
			self->data.MagazineCCR = MAGAZINE_CCR_CLOSE;
			
			magazine.modifyCCR(&magazine,self->data.MagazineCCR);
			
			Rifle_CANBuff[motor[FRI_R].id.buff_p] = motor[FRI_R].c_speed(&motor[FRI_R],0);
			Rifle_CANBuff[motor[FRI_L].id.buff_p] = motor[FRI_L].c_speed(&motor[FRI_L],0);
			
			Rifle_CANBuff[motor[BOX].id.buff_p] = motor[BOX].c_speed(&motor[BOX],0);		

			if(RIFLE_GLOBAL)motor[FRI_R].tx(&motor[FRI_R],Rifle_CANBuff);	
		}
		else{
		
			magazine.modifyCCR(&magazine,0);
			magazine.sleep(&magazine);	
		}

	}
	else if(self->info.Lock == RIFLE_UNLOCK)
	{
		
		magazine.weak(&magazine);
		
		magazine.modifyCCR(&magazine,self->data.MagazineCCR);
		
		Rifle_CANBuff[motor[FRI_R].id.buff_p] = motor[FRI_R].c_speed(&motor[FRI_R], self->data.FriSet);
		Rifle_CANBuff[motor[FRI_L].id.buff_p] = motor[FRI_L].c_speed(&motor[FRI_L],-self->data.FriSet);
		
		if(self->info.BoxType == RIFLE_SPEED){
		
			Rifle_CANBuff[motor[BOX].id.buff_p] = motor[BOX].c_speed(&motor[BOX],self->data.BoxSpeedSet);				
		}
		else if(self->info.BoxType == RIFLE_POSITION){
		
			Rifle_CANBuff[motor[BOX].id.buff_p] = motor[BOX].c_posit(&motor[BOX],self->data.BoxPositionSet);	
		}

		if(RIFLE_GLOBAL)motor[FRI_R].tx(&motor[FRI_R],Rifle_CANBuff);	
		

	}
}




