/**********************************************************************
 *      Author: tstern
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/
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
		//insert the getcal command calibration data here
	//23420,23765,24076,24426,24731,25081,25391,25738,26049,26396,26705,27056,27365,27716,28025,28377,28686,29036,29348,29700,30008,30357,30670,31022,31330,31679,31991,32343,32649,32998,33308,33657,33964,34311,34621,34968,35273,35619,35930,36276,36581,36923,37232,37576,37882,38224,38532,38877,39182,39524,39832,40176,40480,40822,41129,41473,41778,42120,42428,42775,43078,43421,43728,44075,44380,44724,45034,45382,45687,46033,46341,46688,46994,47339,47648,47995,48302,48648,48957,49305,49611,49956,50265,50614,50919,51265,51573,51925,52232,52578,52888,53236,53543,53887,54199,54544,54852,55200,55510,55859,56164,56511,56819,57168,57475,57821,58132,58481,58790,59139,59448,59796,60105,60453,60763,61112,61420,61769,62079,62430,62738,63085,63394,63747,64051,64398,64710,65061,65367,182,492,840,1145,1494,1804,2153,2457,2805,3116,3463,3769,4113,4422,4770,5076,5420,5728,6077,6381,6725,7030,7377,7680,8022,8330,8677,8979,9323,9629,9976,10279,10622,10929,11276,11580,11926,12232,12581,12885,13233,13541,13888,14193,14543,14848,15200,15506,15855,16165,16518,16825,17172,17485,17836,18142,18493,18804,19155,19465,19813,20124,20474,20782,21130,21442,21792,22098,22448,22760,23109,
	0xFFFF //for the valid flag

};
static_assert (sizeof(nvm_t)<sizeof(NVM_flash), "nvm_t structure larger than allocated memory");



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

bool nvmWriteSystemParms(SystemParams_t &systemParams)
{
	systemParams.parametersVaild=true;

	flashWrite((void *)&NVM->SystemParams,&systemParams,sizeof(systemParams));
	return true;
}

bool nvmWriteMotorParms(MotorParams_t &motorParams)
{
	motorParams.parametersVaild=true;

	flashWrite((void *)&NVM->motorParams,&motorParams,sizeof(motorParams));
	return true;
}

bool nvmErase(void)
{
	bool data=false;
	uint16_t cs=0;

	flashWrite((void *)&NVM->CalibrationTable.status,&data,sizeof(data));
	flashWrite((void *)&NVM->sPID.parametersVaild ,&data,sizeof(data));
	flashWrite((void *)&NVM->vPID.parametersVaild ,&data,sizeof(data));
	flashWrite((void *)&NVM->pPID.parametersVaild ,&data,sizeof(data));
	flashWrite((void *)&NVM->motorParams.parametersVaild ,&data,sizeof(data));
	flashWrite((void *)&NVM->SystemParams.parametersVaild ,&data,sizeof(data));
#ifdef NZS_FAST_CAL
	flashWrite((void *)&NVM->FastCal.checkSum,&cs,sizeof(cs));
#endif
}

