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
		40504,40841,41170,41504,41834,42169,42497,42833,43162,43498,43825,44162,44490,44826,45153,45490,45818,46154,46481,46816,47144,47478,47805,48139,48            3,49454,49777,50107,50430,50758,51079,51407,51726,52052,52369,52695,53012,53337,53653,53978,54292,54617,54931,55255,55569,55894,56210,56535,56849,            816,58132,58458,58776,59104,59422,59752,60069,60399,60719,61049,61371,61701,62023,62355,62679,63011,63334,63668,63991,64325,64650,64985,65310,111,            433,1762,2097,2426,2760,3088,3423,3753,4087,4417,4753,5082,5418,5749,6083,6414,6750,7081,7415,7747,8081,8412,8745,9077,9410,9740,10073,10403,10738            1728,12061,12388,12720,13047,13380,13706,14035,14360,14692,15015,15344,15669,15997,16319,16649,16970,17299,17618,17949,18269,18598,18918,19248,195            ,20546,20865,21194,21514,21842,22160,22488,22804,23133,23450,23778,24093,24420,24737,25063,25378,25706,26021,26349,26664,26991,27307,27635,27952,2            28,29246,29577,29896,30227,30548,30881,31203,31537,31860,32197,32521,32857,33183,33520,33847,34185,34512,34850,35178,35516,35845,36182,36511,36847            7842,38179,38508,38844,39174,39510,39839,40174,
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


