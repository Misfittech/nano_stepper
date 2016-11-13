#ifndef __NONVOLATILE__H__
#define __NONVOLATILE__H__

#include "calibration.h"
typedef struct {
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
	int32_t Threshold;
	bool parametersVaild;
} PIDparams_t;

typedef struct {
	int32_t currentMa;
	int32_t currentHoldMa;
	int32_t errorLimit;
	bool parametersVaild;
} SystemParams_t;

typedef struct {
	FlashCalData_t CalibrationTable;
	PIDparams_t PIDparams;
	SystemParams_t SystemParams;
} nvm_t;


extern  const uint16_t  NVM_flash[256];
#define NVM ((const nvm_t *)NVM_flash)

bool nvmWriteCalTable(void *ptrData, uint32_t size);
bool nvmWritePID(int32_t Kp, int32_t Ki, int32_t Kd, int32_t Threshold);
bool nvmWriteSystemParms(int32_t currentMa, int32_t currentHoldMa, int32_t errorLimit);


#endif // __NONVOLATILE__H__
