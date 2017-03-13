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
//58297,58627,58957,59283,59607,59932,60255,60577,60900,61224,61546,61869,62191,62514,62837,63161,63486,63814,64142,64472,64804,65138,65472,273,610,945,1278,1608,1935,2258,2580,2901,3222,3546,3871,4198,4528,4860,5191,5525,5860,6195,6529,6865,7199,7535,7870,8204,8540,8875,9209,9543,9876,10208,10538,10865,11190,11512,11831,12149,12465,12783,13101,13420,13743,14069,14395,14725,15057,15390,15724,16058,16390,16723,17054,17382,17707,18032,18354,18675,18997,19321,19644,19972,20301,20633,20968,21305,21642,21980,22316,22651,22985,23318,23649,23983,24319,24655,24994,25332,25667,25999,26326,26649,26965,27279,27590,27899,28210,28524,28839,29159,29483,29810,30138,30468,30795,31121,31444,31763,32081,32396,32709,33021,33334,33649,33965,34285,34609,34934,35263,35595,35928,36263,36598,36933,37269,37605,37941,38277,38615,38954,39295,39637,39980,40324,40665,41002,41337,41667,41992,42312,42629,42944,43255,43566,43879,44191,44504,44820,45138,45459,45781,46104,46427,46750,47071,47390,47708,48022,48336,48649,48962,49277,49593,49913,50234,50558,50884,51210,51538,51867,52195,52526,52858,53192,53527,53864,54201,54540,54878,55216,55555,55897,56239,56584,56930,57276,57620,57961,
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

