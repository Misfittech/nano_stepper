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
		//22064,22378,22734,23065,23433,23756,24120,24456,24827,25150,25512,25841,26203,26520,26869,27191,27543,27853,28197,28513,28861,29169,29508,29823,30172,30477,30819,31136,31483,31790,32131,32447,32794,33099,33439,33755,34102,34409,34748,35061,35405,35709,36046,36357,36701,37005,37341,37655,37999,38302,38639,38952,39295,39599,39938,40251,40598,40906,41250,41566,41921,42230,42575,42896,43247,43557,43902,44223,44575,44881,45225,45541,45889,46192,46530,46843,47187,47489,47824,48138,48479,48783,49122,49436,49783,50088,50427,50739,51083,51384,51719,52028,52366,52668,53003,53314,53660,53965,54310,54626,54981,55292,55640,55966,56324,56640,56992,57319,57681,57998,58352,58685,59051,59371,59732,60062,60424,60739,61089,61408,61759,62066,62409,62725,63074,63381,63725,64042,64394,64701,65043,65357,167,467,803,1111,1450,1751,2087,2399,2745,3049,3392,3711,4063,4372,4720,5042,5396,5708,6059,6384,6744,7055,7407,7728,8081,8387,8727,9036,9375,9670,10000,10302,10635,10930,11255,11559,11892,12185,12513,12814,13147,13438,13762,14060,14389,14675,14996,15294,15621,15908,16228,16529,16858,17148,17478,17785,18124,18425,18762,19077,19425,19730,20073,20390,20739,21049,21393,21714,
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

