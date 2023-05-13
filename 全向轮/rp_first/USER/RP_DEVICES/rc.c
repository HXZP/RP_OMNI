/*
*		遥控以及键盘

顺序:相同符号之间按绝对值从大到小
tw_step[4]：拨轮产生跳变信号，可以产生四个，拨一次只能产生一个信号，复位后信号才能产生
rc_wake：拨杆安全标志位，全部拨杆归位后才可以使用


*
*/
#include "rc.h"
#include "stdbool.h"
#include "string.h"


static void rc_init(rc_t *self);
static void rc_heart_beat(rc_t *self);
static void rc_updata(rc_t *self , uint8_t *rxBuf);
	
void RC_ResetData(rc_t *self);
bool RC_ResetJudge(rc_t *self);
void RC_Mouse_Speed(rc_t *self);
void RC_KeyMouseReceive(rc_t *self);
rc_t rc = {

  .init       = rc_init,
  .updata     = rc_updata,
	.heart_beat = rc_heart_beat,
	.key        = RC_KeyMouseReceive,
	
	//顺序:相同符号之间按绝对值从大到小
	.info.tw_step_value[0] =  600,
	.info.tw_step_value[1] =  300,
	.info.tw_step_value[2] = -600,
	.info.tw_step_value[3] = -300,	
	
	.info.state = RC_OFFLINE,
	.info.offline_cnt     = 0,
	.info.offline_max_cnt = 100,
};


void rc_init(rc_t *self)
{
	memset(&self->data,0,sizeof(self->data));
	
	self->info.rc_wake = 0;
	self->info.state = RC_OFFLINE;
}



/*遥控数据清空*/
void RC_ResetData(rc_t *self)
{
	memset(&self->data,0,sizeof(self->data));
}

/*遥控死区*/
float RC_DeathZoom(float input, float center, float death)
{
	if(r_abs(input - center) < death)
		return center;
	return input;
}
/*遥控归中查询函数*/
bool RC_ResetJudge(rc_t *self)
{
	if((RC_DeathZoom(self->data.ch0, 0, 1) == 0) && 
		 (RC_DeathZoom(self->data.ch1, 0, 1) == 0) && 
		 (RC_DeathZoom(self->data.ch2, 0, 1) == 0) && 
		 (RC_DeathZoom(self->data.ch3, 0, 1) == 0)   )	
	{
		return true;
	}
	return false;		
}


//int16_t thumbwheel_pre,thumbwheel_record;
void rc_updata(rc_t *self , uint8_t *rxBuf)
{
	
	static int16_t thumbwheel_pre,thumbwheel_record;
	
	thumbwheel_pre = self->data.thumbwheel;
	
	self->data.ch0 = (rxBuf[0]    | rxBuf[1] << 8) & 0x07FF;//0x07FF刚好11位，移位后，将11位后的数与去
	self->data.ch1 = (rxBuf[1]>>3 | rxBuf[2] << 5) & 0x07FF;
	self->data.ch2 = (rxBuf[2]>>6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
	self->data.ch3 = (rxBuf[4]>>1 | rxBuf[5] << 7) & 0x07FF;
	self->data.s2  = (rxBuf[5]>>4) & 0x0003;
	self->data.s1  = (rxBuf[5]>>6) & 0x0003;
		
	self->data.mouse.x       = rxBuf[6]  | (rxBuf[7 ] << 8);
	self->data.mouse.y       = rxBuf[8]  | (rxBuf[9 ] << 8);
	self->data.mouse.z       = rxBuf[10] | (rxBuf[11] << 8);
	self->data.mouse.press_l = rxBuf[12];
	self->data.mouse.press_r = rxBuf[13];
	self->data.kb.key_code   = rxBuf[14] | (rxBuf[15] << 8);	
	
	self->data.thumbwheel    = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	
	self->data.ch0 -= 1024;
	self->data.ch1 -= 1024;
	self->data.ch2 -= 1024;
	self->data.ch3 -= 1024;

	self->data.thumbwheel -= 1024;


	/*拨杆通道值防止越界*/
	if(r_abs(self->data.ch0) > 660 ||
	   r_abs(self->data.ch1) > 660 ||
	   r_abs(self->data.ch2) > 660 ||
	   r_abs(self->data.ch3) > 660){
	
		self->data.ch0 = 0; 
		self->data.ch1 = 0;
		self->data.ch2 = 0;
		self->data.ch3 = 0;		
  }
		 
	/*拨轮产生阶跃信号*/
	if(r_abs(thumbwheel_pre) < r_abs(self->data.thumbwheel) && r_abs(thumbwheel_record) < r_abs(self->data.thumbwheel)){
		
		thumbwheel_record = self->data.thumbwheel;
	}
	if(self->data.thumbwheel == 0 && thumbwheel_record != 0){
	
			for(char i = 0;i < 4;i++){
			
				if(self->info.tw_step_value[i] > 0 && thumbwheel_record > 0){
				
					if(thumbwheel_record >= self->info.tw_step_value[i]){
						
						self->data.tw_step[i] = !self->data.tw_step[i];
						thumbwheel_record = 0;
					}
				}
				if(self->info.tw_step_value[i] < 0 && thumbwheel_record < 0){
				
					if(thumbwheel_record <= self->info.tw_step_value[i]){
						
						self->data.tw_step[i] = !self->data.tw_step[i];
						thumbwheel_record = 0;
					}
				}
				
			}

		
		thumbwheel_record = 0;
	}
	

	if(RC_ResetJudge(self))self->info.rc_wake = 1;
	
	if(!self->info.rc_wake){
		
		self->data.ch0 = 0; 
		self->data.ch1 = 0;
		self->data.ch2 = 0;
		self->data.ch3 = 0;		
	}	
	
	RC_Mouse_Speed(self);
	
	self->info.offline_cnt = 0;

	
}




		
	/*心跳包 在失联检查函数中调用*/
static void rc_heart_beat(rc_t *self)
{
	rc_info_t *info = &self->info;

	info->offline_cnt++;
	if(info->offline_cnt > info->offline_max_cnt)//每次等待一段时间后自动离线
	{
		info->offline_cnt = info->offline_max_cnt;
		info->state = RC_OFFLINE;
	} 
	else //每次接收成功就清空计数
	{
		/* 离线->在线 */
		if(info->state == RC_OFFLINE)
		{
			info->state = RC_ONLINE;
			self->info.rc_wake = 0;
			RC_ResetData(self);  //重新连接后复位所有遥控数据
			
		}
	}
}	


					   	/*键盘部分*/

/*
*	按下检测
*	按下操作
*/
void FirstGetInto_KEY_PRESS(KEY_Info_t *str)
{
  if(str->prev_KEY_PRESS != str->KEY_PRESS)
  {
    str->state_cnt = 0;
    str->prev_KEY_PRESS = str->KEY_PRESS;
  }
}

void KEY_State_Judge(KEY_Info_t *str , uint8_t KEY_PRESS , int change_tim ,int long_change_tim)
{
  str->KEY_PRESS = KEY_PRESS;
  FirstGetInto_KEY_PRESS(str);
  switch(KEY_PRESS)
  {
    case KEY_UP:  {
      if(str->prev_State != UP) 
      {
        str->state_cnt++;
        if(str->state_cnt >= change_tim)  
        {
          str->State = RELAX;
          str->prev_State = RELAX;
          if(str->state_cnt >= change_tim + 1)  //抬起不分长短抬
          {
            str->State = UP;
            str->prev_State = UP;
          }
        }
      }else{str->state_cnt = 0;}
    }break;
    
    case KEY_DOWN:    {
      if(str->prev_State != DOWN) 
      {
        str->state_cnt++;
        if(str->state_cnt >= change_tim)  
        {
          str->State = PRESS;
          str->prev_State = PRESS;
          if(str->state_cnt >= change_tim + 1)
          {
            str->State = SHORT_DOWN;
            str->prev_State = SHORT_DOWN;
            if(str->state_cnt >= long_change_tim)  
            {
              str->State = DOWN;
              str->prev_State = DOWN;
            }
          }
        }
      }else{str->state_cnt = 0;}
    }break;
  }
}


						/*键鼠值转为遥控通道值*/
	

/**
 * @brief 鼠标卡尔曼
 * @note  已加滑动滤波
 * @param 
 */
float Mouse_X_UPDATA, 
			Mouse_Y_UPDATA, 
			Mouse_Z_UPDATA;

float Mouse_X_Last, 
			Mouse_Y_Last, 
			Mouse_Z_Last;

float RC_SF(float t,float *slopeFilter,float res)
{
  for(int i = SF_LENGTH-1;i>0;i--)
  {
    slopeFilter[i] = slopeFilter[i-1];
  }slopeFilter[0] = t;
	
  for(int i = 0;i<SF_LENGTH;i++)
  {
    res += slopeFilter[i];
  }
	return (res/SF_LENGTH);
}


void RC_Mouse_Speed(rc_t *self)
{
	Mouse_X_Last = self->data.ch[0];
	Mouse_Y_Last = self->data.ch[1];
	
  if(r_abs(MOUSE_X_MOVE_SPEED > Xmax))
		self->data.ch[0] = 0;
  else 
		self->data.ch[0] = RC_SF(MOUSE_X_MOVE_SPEED,self->keyMouse.MouseSF.SFX,0);
	
	self->data.ch[0] = (0.7f * self->data.ch[0] + 0.3f * Mouse_X_Last);
	
  if(r_abs(MOUSE_Y_MOVE_SPEED > Ymax))
		self->data.ch[1] = 0;
  else 
		self->data.ch[1] = RC_SF(MOUSE_Y_MOVE_SPEED,self->keyMouse.MouseSF.SFY,0);
	
	self->data.ch[1] = (0.7f * self->data.ch[1] + 0.3f * Mouse_Y_Last);	
	
	
	if(self->data.ch[0]> 660)self->data.ch[0]= 660;
	if(self->data.ch[0]<-660)self->data.ch[0]=-660;
	
	if(self->data.ch[1]> 660)self->data.ch[1]= 660;
	if(self->data.ch[1]<-660)self->data.ch[1]=-660;
	
}



/*=======================================================*/
/*键盘值转为遥控通道值 begin*/

void Sim_Channel(float *CH, char dir,float num)//1up 0down
{
	if( dir)*CH+=num;
	if(!dir)*CH-=num;
	
	if(*CH > CH_MAX)     *CH = CH_MAX;
	if(*CH < CH_OFFSET  )*CH = CH_OFFSET;
}

/*-

缓慢减速
模拟遥控器
wsad中间通道值，最后计算结果赋予rc中模拟通道值Ch[]

-*/
float W_Chan,
      S_Chan,
      A_Chan,
      D_Chan;

float CH_UP   = 1.f;
float CH_DOWN = 3.f;

void RC_Key2Channel(rc_t *self)
{
	int16_t CH;
	
	
	if(self->data.kb.bit.W)
		Sim_Channel(&W_Chan, 1, CH_UP);
	else     
		Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(self->data.kb.bit.S)
		Sim_Channel(&S_Chan, 1, CH_UP);
	else		 
		Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(self->data.kb.bit.A)
		Sim_Channel(&A_Chan, 1, CH_UP);
	else     
		Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(self->data.kb.bit.D)
		Sim_Channel(&D_Chan, 1, CH_UP);
	else     
		Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	CH = W_Chan - S_Chan;	
	if(r_abs(CH) > CH_MAX)
	{
		if(CH > 0)CH = CH_MAX;
		if(CH < 0)CH = CH_MIN;
	}
	self->data.ch[3] = CH;
	
	CH = D_Chan - A_Chan;
	if(r_abs(CH) > CH_MAX)
	{
		if(CH > 0)CH = CH_MAX;
		if(CH < 0)CH = CH_MIN;
	}
	self->data.ch[2] = CH;	
}

/*键盘值转为遥控通道值 end*/

void RC_KeyMouseReceive(rc_t *self)
{
	KEY_State_Judge(&self->keyMouse.Mouse_L ,MOUSE_LEFT, CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_L); 
	KEY_State_Judge(&self->keyMouse.Mouse_R ,MOUSE_RIGH, CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_R); 
	
	KEY_State_Judge(&self->keyMouse.W ,KEY_W, CHANGE_TIM , LONG_CHANGE_TIM_W);
	KEY_State_Judge(&self->keyMouse.S ,KEY_S, CHANGE_TIM , LONG_CHANGE_TIM_S);
	KEY_State_Judge(&self->keyMouse.D ,KEY_D, CHANGE_TIM , LONG_CHANGE_TIM_D);
	KEY_State_Judge(&self->keyMouse.A ,KEY_A, CHANGE_TIM , LONG_CHANGE_TIM_A);
	
	KEY_State_Judge(&self->keyMouse.Q ,KEY_Q, CHANGE_TIM , LONG_CHANGE_TIM_Q);
	KEY_State_Judge(&self->keyMouse.E ,KEY_E, CHANGE_TIM , LONG_CHANGE_TIM_E);
	KEY_State_Judge(&self->keyMouse.C ,KEY_C, CHANGE_TIM , LONG_CHANGE_TIM_C);
	KEY_State_Judge(&self->keyMouse.F ,KEY_F, CHANGE_TIM , LONG_CHANGE_TIM_F);
	KEY_State_Judge(&self->keyMouse.B ,KEY_B, CHANGE_TIM , LONG_CHANGE_TIM_B);
	KEY_State_Judge(&self->keyMouse.R ,KEY_R, CHANGE_TIM , LONG_CHANGE_TIM_R);
	KEY_State_Judge(&self->keyMouse.X ,KEY_X, CHANGE_TIM , LONG_CHANGE_TIM_X);
	KEY_State_Judge(&self->keyMouse.Z ,KEY_Z, CHANGE_TIM , LONG_CHANGE_TIM_Z);
	KEY_State_Judge(&self->keyMouse.G ,KEY_G, CHANGE_TIM , LONG_CHANGE_TIM_G);
	KEY_State_Judge(&self->keyMouse.V ,KEY_V, CHANGE_TIM , LONG_CHANGE_TIM_V);
	
	KEY_State_Judge(&self->keyMouse.CTRL, KEY_CTRL,  CHANGE_TIM , LONG_CHANGE_TIM_CTRL);
	KEY_State_Judge(&self->keyMouse.SHIFT,KEY_SHIFT, CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);
	
	RC_Key2Channel(self);
}






