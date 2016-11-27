#include "nonvolatile.h"
#include "flash.h"
#include <Arduino.h>



//we use this so we can hard code calibration table
// be sure to set the last word as status flag
// this save time calibrating each time we do a code build
#ifdef NZS_FAST_CAL
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[16640]={  //allocates 33280 bytes
#else
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[256]={  //allocates 512 bytes
#endif
<<<<<<< HEAD:firmware/stepper_nano_zero/nonvolatile.cpp

		25125,25451,25805,26119,26469,26783,27123,27428,27769,28076,28410,28710,29046,29349,29680,29980,30316,30624,30962,31267,31611,31925,32268,32579,32929,33248,33596,33911,34263,34580,34925,35234,35578,35890,36227,36531,36869,37178,37516,37821,38163,38476,38818,39128,39476,39792,40137,40450,40802,41120,41469,41784,42135,42453,42801,43114,43464,43778,44123,44433,44779,45089,45428,45734,46073,46381,46720,47029,47374,47690,48038,48353,48708,49029,49379,49695,50044,50361,50700,51004,51341,51643,51971,52268,52602,52906,53241,53546,53891,54203,54547,54859,55209,55527,55877,56191,56544,56863,57212,57528,57883,58206,58561,58882,59239,59562,59911,60220,60563,60873,61208,61509,61846,62152,62490,62797,63140,63454,63797,64108,64451,64763,65098,65398,198,499,826,1122,1453,1758,2092,2398,2739,3053,3398,3710,4060,4379,4729,5046,5401,5725,6081,6405,6766,7096,7456,7777,8135,8458,8807,9119,9465,9779,10119,10424,10768,11077,11413,11719,12061,12369,12705,13008,13346,13650,13980,14277,14608,14907,15233,15528,15857,16155,16484,16781,17114,17421,17756,18061,18404,18718,19057,19367,19714,20028,20372,20683,21031,21347,21694,22010,22368,22692,23050,23376,23742,24074,24438,24763,
=======
>>>>>>> 2226a588d97e2a1798ec7a1ea41cc1382f29fba3:firmware/nonvolatile.cpp
		0xFFFF //for the valid flag

};

//#if sizeof(nvm_t)>sizeof(NVM_flash)
//#error "nvm_t structure larger than allocated memory"
//#endif

//FLASH_ALLOCATE(NVM_flash, sizeof(nvm_t));


bool nvmWriteCalTable(void *ptrData, uint32_t size)
{
	bool x=true;
	flashWrite(&NVM->CalibrationTable,ptrData,size);
	return true;
}

bool nvmWritePID(int32_t Kp, int32_t Ki, int32_t Kd,int32_t Threshold)
{
	PIDparams_t pid;

	pid.Kp=Kp;
	pid.Ki=Ki;
	pid.Kd=Kd;
	pid.Threshold=Threshold;
	pid.parametersVaild=true;

	flashWrite((void *)&NVM->PIDparams,&pid,sizeof(pid));
	return true;
}

bool nvmWriteSystemParms(int32_t currentMa, int32_t currentHoldMa, int32_t errorLimit)

{
	SystemParams_t params;

	params.currentMa=currentMa;
	params.currentHoldMa=currentHoldMa;
	params.errorLimit=errorLimit;

	params.parametersVaild=true;

	flashWrite((void *)&NVM->SystemParams,&params,sizeof(params));
	return true;
}


