/*

×÷±×Èí¼þ ÃØ¼¼

*/


#include "auto.h"
#include "DEVICES.h"

auto_t cheat;






void Auto_ModifyType(auto_t *self,auto_VisionType type)
{
	if(self->info.type != type){
	
		self->info.type = type;
	}
}



void Auto_Updata(auto_t *self)
{
	if(vision.info.state == VISION_OFFLINE){

		cheat.info.UpperState = AUTO_ERR;
	}
  else{
	
		cheat.info.UpperState = AUTO_ONLINE;	
	}






}



























