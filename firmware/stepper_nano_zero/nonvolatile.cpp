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
	//13167,13466,13801,14110,14453,14752,15089,15401,15744,16047,16385,16701,17043,17348,17686,17999,18341,18640,18975,19284,19624,19925,20264,20577,20924,21231,21575,21896,22250,22562,22914,23239,23601,23916,24271,24599,24959,25273,25625,25948,26304,26615,26961,27280,27628,27933,28272,28585,28926,29225,29560,29870,30212,30512,30850,31164,31511,31818,32162,32483,32835,33145,33489,33807,34152,34455,34791,35099,35436,35733,36064,36370,36707,37006,37342,37655,37999,38306,38650,38970,39323,39635,39982,40304,40657,40968,41316,41637,41993,42304,42653,42975,43327,43635,43977,44289,44631,44931,45262,45570,45905,46203,46535,46845,47189,47493,47833,48152,48504,48814,49161,49481,49829,50136,50472,50784,51126,51422,51752,52060,52400,52699,53035,53350,53697,54006,54349,54670,55023,55336,55687,56010,56366,56682,57034,57361,57720,58036,58390,58715,59075,59390,59737,60055,60403,60706,61044,61355,61697,61999,62335,62648,62992,63299,63641,63957,64306,64614,64956,65273,85,392,732,1046,1389,1692,2026,2338,2677,2977,3315,3629,3974,4280,4625,4946,5296,5607,5955,6275,6629,6940,7290,7612,7968,8283,8633,8956,9310,9617,9959,10273,10616,10916,11250,11559,11897,12194,12527,12835,
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

