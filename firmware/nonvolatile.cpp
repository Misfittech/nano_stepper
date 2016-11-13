#include "nonvolatile.h"
#include "flash.h"
#include <Arduino.h>



//we use this so we can hard code calibration table
// be sure to set the last word as status flag
// this save time calibrating each time we do a code build
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[256]={
		25611,25928,26279,26591,26937,27253,27599,27908,28249,28564,28908,29223,29564,29876,30223,30528,30866,31181,31527,31837,32177,32490,32833,33141,33479,33792,34135,34442,34781,35095,35437,35745,36086,36405,36753,37065,37410,37729,38081,38395,38745,39066,39420,39737,40087,40409,40765,41081,41430,41750,42103,42416,42761,43077,43426,43734,44075,44387,44731,45037,45373,45685,46026,46329,46664,46973,47312,47614,47948,48255,48592,48893,49226,49533,49870,50171,50502,50812,51152,51456,51790,52100,52443,52749,53086,53398,53745,54054,54394,54710,55058,55369,55713,56031,56381,56695,57042,57361,57713,58024,58370,58687,59036,59345,59688,60001,60346,60654,60992,61304,61647,61952,62288,62597,62940,63246,63582,63893,64236,64542,64880,65193,5,314,653,968,1314,1623,1965,2280,2629,2938,3281,3597,3946,4258,4604,4921,5273,5588,5936,6256,6610,6925,7275,7595,7950,8265,8614,8933,9287,9601,9946,10264,10614,10925,11267,11582,11927,12235,12573,12886,13229,13536,13872,14183,14525,14832,15169,15481,15824,16131,16468,16781,17123,17430,17767,18078,18420,18725,19059,19370,19713,20018,20354,20663,21010,21315,21654,21969,22316,22625,22968,23285,23635,23946,24291,24609,24959,25271,
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


