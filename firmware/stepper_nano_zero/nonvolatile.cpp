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

		//40715,41044,41404,41733,42096,42425,42774,43090,43429,43730,44055,44351,44672,44970,45296,45598,45933,46243,46585,46900,47245,47559,47899,48205,48536,48837,49165,49462,49789,50090,50420,50722,51055,51362,51696,52001,52341,52654,52997,53314,53665,53988,54343,54665,55019,55343,55697,56020,56377,56708,57067,57397,57759,58085,58438,58753,59094,59404,59737,60037,60370,60675,61008,61312,61645,61952,62285,62586,62919,63226,63558,63864,64198,64507,64845,65153,65493,274,617,933,1274,1588,1929,2237,2575,2883,3218,3526,3863,4174,4513,4827,5169,5489,5842,6166,6526,6860,7227,7559,7920,8252,8610,8932,9285,9607,9958,10275,10624,10940,11282,11590,11924,12228,12556,12855,13182,13482,13810,14110,14444,14752,15096,15414,15769,16102,16465,16796,17156,17482,17832,18147,18486,18793,19126,19423,19749,20046,20366,20659,20985,21289,21622,21932,22277,22598,22949,23270,23623,23945,24294,24607,24953,25267,25613,25931,26281,26605,26957,27277,27624,27939,28277,28579,28906,29207,29532,29828,30158,30465,30806,31122,31475,31803,32163,32490,32848,33175,33523,33836,34172,34476,34802,35095,35418,35715,36044,36348,36686,37002,37351,37669,38024,38350,38704,39025,39377,39698,40047,40364,
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

bool nvmWrite_sPID(float Kp, float Ki, float Kd)
{
	PIDparams_t pid;

	pid.Kp=Kp;
	pid.Ki=Ki;
	pid.Kd=Kd;
	pid.parametersVaild=true;

	flashWrite((void *)&NVM->sPID,&pid,sizeof(pid));
	return true;
}

bool nvmWrite_vPID(float Kp, float Ki, float Kd)
{
	PIDparams_t pid;

	pid.Kp=Kp;
	pid.Ki=Ki;
	pid.Kd=Kd;
	pid.parametersVaild=true;

	flashWrite((void *)&NVM->vPID,&pid,sizeof(pid));
	return true;
}

bool nvmWrite_pPID(float Kp, float Ki, float Kd)
{
	PIDparams_t pid;

	pid.Kp=Kp;
	pid.Ki=Ki;
	pid.Kd=Kd;
	pid.parametersVaild=true;

	flashWrite((void *)&NVM->pPID,&pid,sizeof(pid));
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


