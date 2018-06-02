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
#include "Flash.h"  //thanks to Kent Larsen for pointing out the lower case error
#include <Arduino.h>




//we use this so we can hard code calibration table
// be sure to set the last word as status flag
// this save time calibrating each time we do a code build
#ifdef NZS_FAST_CAL
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[16767]={  //allocates 33280 bytes
#else
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[256]={  //allocates 512 bytes
#endif
    		//insert the getcal command calibration data here
//41926,42263,42597,42935,43263,43600,43932,44267,44597,44929,45259,45592,45912,46247,46576,46903,47227,47561,47886,48218,48543,48872,49201,49530,49856,50188,50516,50845,51167,51499,51826,52154,52480,52809,53142,53474,53800,54136,54468,54804,55138,55477,55813,56155,56486,56835,57178,57513,57849,58192,58530,58871,59204,59546,59885,60220,60550,60886,61220,61556,61882,62218,62544,62872,63197,63521,63847,64171,64488,64809,65132,65450,231,549,867,1188,1500,1819,2133,2450,2761,3076,3392,3712,4022,4340,4655,4975,5289,5607,5922,6244,6560,6878,7192,7514,7829,8146,8467,8785,9098,9418,9735,10059,10369,10691,11008,11329,11645,11964,12283,12603,12918,13239,13559,13879,14195,14514,14834,15154,15469,15790,16108,16429,16742,17066,17383,17705,18023,18343,18665,18991,19310,19631,19956,20283,20604,20932,21258,21588,21913,22245,22577,22914,23240,23583,23916,24256,24593,24933,25276,25617,25954,26299,26641,26985,27321,27663,28000,28344,28678,29017,29353,29692,30020,30355,30687,31022,31351,31681,32014,32340,32665,32996,33325,33658,33982,34313,34642,34971,35298,35630,35959,36292,36616,36948,37276,37611,37937,38272,38601,38936,39268,39602,39939,40272,40603,40942,41278,41614,

			0xFFFF
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

