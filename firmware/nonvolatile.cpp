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
//		41659,41992,42330,42662,42999,43335,43675,44008,44349,44686,45026,45360,45700,46033,46370,46701,47036,47367,47701,48027,48359,48685,49015,49336,49663,49984,50310,50629,50953,51273,51599,51918,52244,52568,52896,53219,53549,53877,54210,54537,54873,55206,55543,55874,56213,56546,56882,57210,57542,57869,58195,58515,58837,59156,59477,59792,60111,60428,60749,61062,61382,61698,62016,62325,62639,62951,63262,63567,63876,64182,64491,64794,65105,65411,188,492,806,1118,1433,1746,2066,2385,2710,3030,3359,3684,4014,4340,4673,5003,5336,5663,6001,6335,6674,7010,7352,7691,8033,8370,8716,9057,9398,9735,10077,10413,10750,11079,11414,11743,12073,12398,12728,13053,13381,13704,14034,14360,14690,15015,15347,15674,16007,16334,16670,17000,17335,17664,18002,18335,18673,19005,19347,19684,20023,20359,20699,21038,21378,21709,22047,22381,22716,23044,23377,23705,24034,24358,24689,25014,25344,25670,26000,26328,26659,26985,27316,27645,27977,28302,28633,28959,29289,29611,29938,30262,30588,30907,31232,31553,31877,32194,32518,32837,33162,33480,33807,34129,34457,34777,35105,35427,35755,36074,36399,36721,37046,37367,37693,38018,38346,38670,39002,39330,39664,39994,40328,40660,40997,41326,
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


