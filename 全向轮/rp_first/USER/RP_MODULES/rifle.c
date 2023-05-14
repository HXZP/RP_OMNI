
#include "DEVICES.h"
#include "RP_FUNCTION.h"
#include "rifle.h"

rifle gun = {



	.friTable.Speed0 = 0,


};


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

	self->data.HeatEnableNum = (self->data.HeatLimit - self->data.Heat)/10;
	
	//判断热量允许
	if(judge.info.state == JUDGE_ONLINE){
	
		if(self->data.HeatEnableNum > 1){
		
			self->info.HeatEnable = RIFLE_OK;
		}
		else{
		
			self->info.HeatEnable = RIFLE_NO;
		}
	
	}
	else{
	
		self->info.HeatEnable = RIFLE_OK;
	}
	
	//判断是否达到射击条件
	if(self->info.FriSpeedReach == RIFLE_OK && self->info.BoxStucking == RIFLE_OK && self->info.HeatEnable == RIFLE_OK){
	
		self->info.ShootReady = RIFLE_OK;
	}
}

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
	}
}


void Rifle_ShootSet(rifle *self,rifle_ShootType type)
{
	if(self->info.BoxStucking == RIFLE_ING){
	
		return;
	}
	
	if(self->info.BoxStucking == RIFLE_OK){
	
		if(self->data.BoxPositionSet < 0 || self->data.BoxSpeedSet < 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition + 8191 * 4.5f;
		}
		else if(self->data.BoxPositionSet > 0 || self->data.BoxSpeedSet > 0){
		
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f;
		}
		self->info.BoxStucking = RIFLE_ING;
		
		return;
	}	
	
	
	if(self->info.ShootReady == RIFLE_NO){
	
		self->data.BoxSpeedSet = 0;
	}
	else{
		
		if(type == RIFLE_SHOOT_STOP){
		
			self->data.BoxSpeedSet = 0;
		}
		else if(type == RIFLE_SHOOT_ONCE){
			
			self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f;
		}
		else if(type == RIFLE_SHOOT_STAY){
			
			if((self->data.HeatLimit - self->data.Heat > 50) && judge.info.state == JUDGE_ONLINE){
				
				self->data.BoxSpeedSet = 4000;
			}
			else{
				
				self->data.BoxSpeedSet = 2000;
			}				
		}
		else if(type == RIFLE_SHOOT_7){
		
			if(self->data.HeatEnableNum > 6){
			
				self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f * 6;
			}
			else if(self->data.HeatEnableNum <= 6){
			
				self->data.BoxPositionSet = self->data.BoxPosition - 8191 * 4.5f * self->data.HeatEnableNum;
			}
		}
	}
}












