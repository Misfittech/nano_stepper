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

	//6745,7063,7416,7741,8099,8416,8770,9094,9445,9759,10105,10419,10761,11064,11400,11707,12046,12347,12682,12991,13327,13630,13968,14277,14616,14918,15254,15559,15896,16196,16530,16834,17170,17473,17807,18115,18453,18757,19095,19406,19749,20058,20401,20716,21066,21379,21727,22045,22401,22718,23071,23394,23753,24072,24429,24752,25110,25427,25778,26095,26446,26756,27100,27412,27755,28062,28400,28709,29049,29353,29689,29997,30336,30639,30976,31287,31628,31936,32277,32591,32937,33248,33591,33907,34252,34561,34904,35218,35563,35872,36213,36526,36871,37181,37527,37844,38195,38511,38863,39185,39543,39863,40218,40542,40897,41217,41568,41889,42242,42556,42904,43223,43570,43880,44221,44533,44875,45178,45515,45822,46159,46462,46800,47108,47450,47757,48100,48415,48763,49075,49423,49738,50084,50394,50735,51045,51386,51687,52022,52326,52662,52965,53301,53608,53950,54255,54598,54912,55261,55572,55922,56239,56589,56904,57252,57569,57920,58232,58582,58898,59248,59560,59905,60217,60561,60867,61205,61513,61850,62152,62489,62797,63137,63443,63785,64100,64444,64755,65098,65412,221,527,866,1173,1509,1812,2145,2451,2787,3090,3428,3740,4083,4393,4738,5057,5406,5721,6074,6395,
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

