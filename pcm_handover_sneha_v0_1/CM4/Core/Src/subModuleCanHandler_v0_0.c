/*
 * subModuleCanHandler_v0_0.c
 *
 *  Created on: Dec 6, 2025
 *      Author: flowtrik-3
 */

#include "stdbool.h"
#include "string.h"
#include "stm32h745xx.h"
#include "stm32h7xx.h"


#define TMCM_HANDLER_DURATION					(uint32_t)100
#define BMS_HANDLER_DURATION					(uint32_t)5000


void subModuleCanHandler()
{
	//static uint32_t cmcmHandlerTick=0;
	static uint32_t tmcmHandlerTick=0;
	static uint32_t bmsHandlerTick=0;

	if(tmcmHandlerTick + TMCM_HANDLER_DURATION < uwTick)
	{
//		tmcm_SM();
		tmcmHandlerTick = uwTick;
	}
	else if(bmsHandlerTick + BMS_HANDLER_DURATION < uwTick)
	{
		bmsHandlerTick = uwTick;
	}

}
