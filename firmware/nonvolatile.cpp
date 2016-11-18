#include "nonvolatile.h"
#include "flash.h"
#include <Arduino.h>



//we use this so we can hard code calibration table
// be sure to set the last word as status flag
// this save time calibrating each time we do a code build
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[256]={
		58220,58535,58886,59200,59540,59856,60205,60518,60855,61171,61517,61829,62166,62479,62825,63135,63472,63786,64133,64445,64781,65098,65443,222,559,876,1221,1532,1868,2182,2526,2835,3172,3486,3832,4145,4485,4803,5152,5469,5814,6135,6489,6810,7155,7478,7834,8154,8501,8824,9182,9500,9846,10167,10521,10837,11179,11496,11845,12155,12493,12807,13151,13460,13795,14107,14449,14757,15090,15399,15738,16043,16372,16678,17014,17317,17644,17949,18286,18590,18919,19227,19570,19877,20211,20522,20868,21178,21515,21831,22179,22491,22831,23148,23496,23810,24150,24465,24813,25123,25461,25774,26118,26428,26762,27074,27416,27722,28056,28365,28705,29011,29341,29647,29984,30287,30616,30921,31257,31560,31887,32194,32531,32835,33165,33475,33817,34124,34460,34774,35118,35431,35770,36087,36437,36751,37095,37414,37767,38086,38434,38758,39117,39439,39791,40117,40478,40802,41153,41478,41837,42156,42503,42824,43177,43491,43832,44149,44495,44806,45143,45457,45803,46112,46448,46761,47106,47415,47751,48064,48410,48721,49057,49372,49718,50030,50366,50681,51029,51340,51676,51990,52337,52646,52984,53297,53643,53953,54290,54603,54950,55260,55597,55911,56259,56570,56909,57223,57574,57884,
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


