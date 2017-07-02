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
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[16640]={  //allocates 33280 bytes
#else
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[256]={  //allocates 512 bytes
#endif

#if NZS_LABEL == 'Y'
    		13722,14056,14386,14713,15041,15378,15706,16030,16353,16685,17006,17327,17647,17972,18293,18615,18934,19264,19583,19911,20235,20566,20894,21229,21557,21896,22229,22563,22892,23227,23557,23885,24211,24543,24868,25195,25516,25844,26168,26494,26819,27152,27481,27814,28148,28490,28826,29165,29500,29842,30174,30507,30835,31165,31488,31811,32131,32456,32776,33101,33423,33755,34082,34412,34742,35076,35409,35742,36079,36420,36756,37091,37428,37764,38097,38428,38754,39086,39408,39732,40055,40380,40700,41022,41343,41669,41989,42316,42637,42964,43290,43616,43943,44275,44604,44934,45264,45597,45925,46253,46579,46912,47235,47562,47882,48212,48534,48857,49177,49507,49828,50154,50479,50816,51145,51480,51814,52160,52500,52841,53178,53522,53858,54190,54520,54853,55178,55502,55824,56147,56466,56783,57100,57423,57739,58064,58383,58712,59037,59363,59689,60020,60346,60672,60996,61323,61642,61960,62278,62599,62915,63236,63555,63883,64205,64531,64858,65191,65518,313,642,975,1301,1628,1952,2282,2608,2937,3264,3599,3934,4267,4602,4944,5277,5608,5939,6275,6598,6921,7243,7569,7887,8204,8523,8845,9164,9487,9808,10136,10458,10784,11107,11440,11766,12093,12420,12750,13079,13406,
#endif 
        //insert the getcal command calibration data here
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

