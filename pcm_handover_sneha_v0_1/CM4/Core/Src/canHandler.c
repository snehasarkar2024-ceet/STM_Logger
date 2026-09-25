/*
* can_handler.c
*
*  Created on: Oct 9, 2025
*      Author: flowtrik-3
*/

//#include "../../../CM4/Core/Inc/canHandler.h"

#include "canHandler.h"


uint8_t u8_canTxXfer[8]={0}, u8_canRxXfer[8]={0};
FDCAN_TxHeaderTypeDef canTxHeader;
FDCAN_RxHeaderTypeDef canRxHeader;
uint32_t u32_latestCan1PktTick = (uint32_t)0;
uint32_t u32_latestCan2PktTick = (uint32_t)0;
bool b_selectedCan = CAN_BUS_2;
volatile uint8_t  tmcmCmdAckReceived = 0;

/* Allocate storage specifically into STM32H7 AXI-SRAM */
uint8_t can_log_buffer[CAN_RAM_LOG_SIZE] __attribute__((section(".shared")));

volatile uint32_t dbg_lastIdentifier = 0;
volatile uint8_t  dbg_lastData[8] = {0};
volatile uint8_t canTo4GBuffer[8] = {0};

volatile uint8_t canToEspBuffer[8] = {0};
volatile uint8_t canToEspFlag = 0;

/*Debugging variables for CANTransmit*/
volatile uint8_t dbg_lastTxData[8] = {0};
volatile uint32_t  dbg_lastTxIdentifier = 0;
volatile uint8_t dbg_lastTxSize = 0;
volatile uint8_t counter =0;

volatile uint8_t canTmcmAckFrame[8] = {0};
volatile uint8_t canTmcmAckReady = 0;
volatile uint32_t dbg_tmcmAck_id_count = 0;
volatile uint8_t dbg_last_tmcmAck_data[8] = {0};
volatile uint8_t ackPending = 0;
volatile uint8_t ackPendingHW = 0;        /* waiting for 0x07792010 TMCM ACK before publishing */
volatile uint32_t dbg_RpmPgn_count = 0;
volatile uint32_t dbg_RpmPgn_count1 = 0;
volatile uint32_t lastRpmPgnRxTick = 0;
/* Data variables for 4g*/

volatile uint8_t canDataReady = 0;
volatile uint8_t can4GFrameType = 0;
/* Debug decoded values */
volatile uint16_t dbg_speed;
volatile uint16_t dbg_val2;
volatile uint8_t  dbg_val3;
volatile uint16_t dbg_val4;
volatile uint8_t  dbg_fault;

/* TMCM CANReceive Frames */
volatile uint8_t canFrame1[8] = {0};
volatile uint8_t canFrame2[8] = {0};
volatile uint8_t canFrame1Ready = 0;
volatile uint8_t canFrame2Ready = 0;

/* DCDC 12V */
volatile uint8_t canDcdc12vFrame[8] = {0};
volatile uint8_t canDcdc12vReady = 0;

/* DCDC 48V */
volatile uint8_t canDcdc48vFrame[8] = {0};
volatile uint8_t canDcdc48vReady = 0;

/* BMS Frame Buffer */
volatile uint8_t canBmsFrame[8] = {0};
volatile uint8_t canBmsReady = 0;
volatile uint8_t canBmsFrame1[8] = {0};
volatile uint8_t canBmsReady1 = 0;

/* NEWLY ADDED DEVICE BUFFERS */
volatile uint8_t canDevice1300Frame[8] = {0};
volatile uint8_t canDevice1300Ready = 0;

volatile uint8_t canDevice10FFFrame[8] = {0};
volatile uint8_t canDevice10FFReady = 0;

volatile uint8_t canDevice0600Frame[8] = {0};
volatile uint8_t canDevice0600Ready = 0;

volatile uint8_t canDevice0700Frame[8] = {0};
volatile uint8_t canDevice0700Ready = 0;


#define CAN_4G_QUEUE_SIZE 400
volatile CAN4GPacket_t can4GQueue[CAN_4G_QUEUE_SIZE];

volatile uint16_t can4GHead=0;
volatile uint16_t can4GTail=0;
volatile uint16_t can4GCount=0;

/*variable for canreceivelogdata*/
volatile CANLogPacket_t canLogBuffer[CAN_LOG_BUFFER_SIZE];

volatile uint16_t canLogIndex = 0;
volatile uint16_t canLogHead = 0;
volatile uint16_t canLogTail = 0;
volatile uint16_t canLogCount = 0;
volatile uint16_t count1 = 0;

/* Debug decoded values for Charger / DCDC / BMS */
volatile uint32_t dbg_dcdc12v_id_count = 0;
volatile uint32_t dbg_dcdc48v_id_count = 0;
volatile uint32_t dbg_bms_id_count    = 0;
volatile uint32_t dbg_bms1_id_count    = 0;
volatile uint32_t dbg_dev1300_id_count = 0;
volatile uint32_t dbg_dev10FF_id_count = 0;
volatile uint32_t dbg_dev0600_id_count = 0;
volatile uint32_t dbg_dev0700_id_count = 0;

/* Raw snapshot values for Watch Window validation */
volatile uint8_t dbg_last_dcdc12v_data[8] = {0};
volatile uint8_t dbg_last_dcdc48v_data[8] = {0};
volatile uint8_t dbg_last_bms_data[8]     = {0};
volatile uint8_t dbg_last_bms1_data[8]     = {0};
volatile uint8_t dbg_last_dev1300_data[8] = {0};
volatile uint8_t dbg_last_dev10FF_data[8] = {0};
volatile uint8_t dbg_last_dev0600_data[8] = {0};
volatile uint8_t dbg_last_dev0700_data[8] = {0};


volatile uint8_t canRpmPgnFrame[8] = {0};
volatile uint8_t canRpmPgnFrame1[8]= {0};
volatile uint8_t canRpmPgnFrame2[8]=  {0};
volatile uint8_t dbg_RpmPgn_data[8] =  {0};
volatile uint8_t dbg_RpmPgn_data1[8]=  {0};
volatile uint8_t dbg_RpmPgn_stop[8]=  {0};

volatile uint32_t dbg_TmcmV2_2_count = 0;
volatile uint8_t  canTmcmV2_2Frame[8] = {0};
volatile uint8_t  dbg_TmcmV2_2_data[8]= {0};
volatile uint8_t rpmrecflag =0;


// --- NTP & RTC Broadcast Watch Variables ---
volatile uint32_t dbg_rtc_broadcast_count = 0;         // Increments every time a sync packet is sent
volatile uint8_t  dbg_rtc_broadcast_last_data[8] = {0};  // Holds the last transmitted 8-byte time packet
volatile uint8_t  ntpSyncSuccessFlag = 0;              // Set to 1 in your NTP code when synchronization succeeds
volatile uint8_t  ntpBroadcastSent = 0;                // Ensures the sync packet is only sent once per sync event
volatile uint32_t dbg_rtc_sync_tx_count = 0;
volatile uint8_t  dbg_rtc_sync_last_data[8] = {0};

/*Extra function for RTC monitor*/
volatile uint8_t rtc_live_hour = 0;
volatile uint8_t rtc_live_minute = 0;
volatile uint8_t rtc_live_second = 0;

volatile uint8_t rtc_live_year = 0;
volatile uint8_t rtc_live_month = 0;
volatile uint8_t rtc_live_date = 0;


/* Global Instance of the RAM Log Storage */
CAN_RAM_Storage_t g_canRamStorage = {0};
volatile uint8_t g_sendCanRamDataFlag = 0;
volatile uint32_t dbg_ram_tx_count = 0;
volatile uint32_t dbg_ram_tx_bytes = 0;
volatile uint32_t dbg_ram_last_can_id = 0;

void CheckServerCommand(const char *rxBuffer)
{
    if (rxBuffer == NULL) return;

    /* Check if "DATA_REQ" string is present inside currentRx */
    if (strstr(rxBuffer, "DATA_REQ") != NULL)
    {
        /* Set flag to start sending stored RAM frames in task context */
        g_sendCanRamDataFlag = 1;
    }
}

/**
 * @brief Offloads all CAN frames currently in RAM to ESP via UART formatted as CSV/AT command
 */
void CAN_RAM_FlushToESP(void)
{
    CAN_RAM_Log_t frame;
    char txBuffer[128];

    /* Pop frames from RAM one by one until queue is empty */
    while (CAN_RAM_ReadFrame(&frame))
    {
        /* Format: $CAN,<BusID>,<Timestamp>,<CAN_ID>,<DLC>,<Data Bytes>\r\n */
        int len = snprintf(txBuffer, sizeof(txBuffer),
                           "$CAN,%u,%lu,0x%08X,%u,%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                           frame.busId,
                           (unsigned long)frame.timestamp,
                           (unsigned long)frame.canId,
                           frame.dlc,
                           frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                           frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        if (len > 0)
        {
            /* Transmit formatted string to ESP over UART7 */
            if (HAL_UART_Transmit(&huart7, (uint8_t*)txBuffer, (uint16_t)len, 100) == HAL_OK)
            {
                /* Update Live Watch Telemetry */
                dbg_ram_tx_count++;
                dbg_ram_tx_bytes += (uint32_t)len;
                dbg_ram_last_can_id = frame.canId;
            }
        }
    }

    /* Reset flag after processing */
    g_sendCanRamDataFlag = 0;
}
//void CAN_RAM_FlushToESP(void)
//{
//    CAN_RAM_Log_t frame;
//    char txBuffer[128];
//
//    /* Pop frames from RAM one by one until queue is empty */
//    while (CAN_RAM_ReadFrame(&frame))
//    {
//        /* Format CAN frame into AT command / ASCII string for ESP */
//        /* Format: $CAN,<BusID>,<Timestamp>,<CAN_ID>,<DLC>,<Data Bytes>\r\n */
//        int len = snprintf(txBuffer, sizeof(txBuffer),
//                           "$CAN,%u,%lu,0x%08LX,%u,%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
//                           frame.busId,
//                           (unsigned long)frame.timestamp,
//                           (unsigned long)frame.canId,
//                           frame.dlc,
//                           frame.data[0], frame.data[1], frame.data[2], frame.data[3],
//                           frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
//
//        if (len > 0)
//        {
//            /* Transmit formatted CAN log to ESP over UART */
//            HAL_UART_Transmit(&huart7, (uint8_t*)txBuffer, (uint16_t)len, 100); // Adjust &huart3 to match your ESP UART
//
//        }
//    }
//
//    /* Reset flag after emptying buffer */
//    g_sendCanRamDataFlag = 0;
//}

/**
 * @brief Initialize RAM storage buffer
 */
void CAN_RAM_Init(void)
{
    g_canRamStorage.head = 0;
    g_canRamStorage.tail = 0;
    g_canRamStorage.count = 0;
    g_canRamStorage.overflowCount = 0;
}

/**
 * @brief Push a CAN frame into RAM - STRICT NO-OVERWRITE MODE
 */
void CAN_RAM_StoreFrame(uint32_t canId, uint8_t *pData, uint8_t dlc, uint8_t busId)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (g_canRamStorage.count >= CAN_RAM_LOG_SIZE)
    {
        g_canRamStorage.overflowCount++;
        __set_PRIMASK(primask);
        return;
    }

    uint16_t currentHead = g_canRamStorage.head;

    g_canRamStorage.buffer[currentHead].canId = canId;
    g_canRamStorage.buffer[currentHead].timestamp = HAL_GetTick();
    g_canRamStorage.buffer[currentHead].dlc = (dlc > 8) ? 8 : dlc;
    g_canRamStorage.buffer[currentHead].busId = busId;

    if (pData != NULL) {
        memcpy((void*)g_canRamStorage.buffer[currentHead].data, pData, g_canRamStorage.buffer[currentHead].dlc);
    }

    g_canRamStorage.head = (currentHead + 1) % CAN_RAM_LOG_SIZE;
    g_canRamStorage.count++;

    __set_PRIMASK(primask);
}

/**
 * @brief Pop/Read a CAN frame from RAM in task context (Thread Safe)
 * @param outFrame Pointer to destination structure
 * @return 1 if frame retrieved, 0 if queue empty
 */
uint8_t CAN_RAM_ReadFrame(CAN_RAM_Log_t *outFrame)
{
    if (outFrame == NULL) return 0;

    __disable_irq();

    if (g_canRamStorage.count == 0) {
        __enable_irq();
        return 0; // Queue empty
    }

    uint16_t currentTail = g_canRamStorage.tail;

    /* Copy Frame from RAM */
    outFrame->canId = g_canRamStorage.buffer[currentTail].canId;
    outFrame->timestamp = g_canRamStorage.buffer[currentTail].timestamp;
    outFrame->dlc = g_canRamStorage.buffer[currentTail].dlc;
    outFrame->busId = g_canRamStorage.buffer[currentTail].busId;
    memcpy(outFrame->data, (void*)g_canRamStorage.buffer[currentTail].data, outFrame->dlc);

    /* Advance Tail and decrement count */
    g_canRamStorage.tail = (currentTail + 1) % CAN_RAM_LOG_SIZE;
    g_canRamStorage.count--;

    __enable_irq();

    return 1;
}

/**
 * @brief Get total unread frame count in RAM
 */
uint16_t CAN_RAM_GetCount(void)
{
    return g_canRamStorage.count;
}

/**
 * @brief Flush RAM Queue
 */
void CAN_RAM_Clear(void)
{
    __disable_irq();
    g_canRamStorage.head = 0;
    g_canRamStorage.tail = 0;
    g_canRamStorage.count = 0;
    __enable_irq();
}

/* STORE ALL RECEIVED FRAMES (Interrupt Context) */
void StoreCANLog(uint8_t frameType, uint8_t *data)
{
//    __disable_irq(); // Protect shared variables
count1++;
// Copy data into the current head position
canLogBuffer[canLogHead].frameType = frameType;
memcpy((void*)canLogBuffer[canLogHead].data, data, 8);

canLogHead++;
if(canLogHead >= CAN_LOG_BUFFER_SIZE)
{
	canLogHead = 0;
}

if (canLogCount < CAN_LOG_BUFFER_SIZE)
{
	canLogCount++;
}
else
{
	// Buffer Overflow Safeguard: Drop oldest unread packet to prevent lockup
	canLogTail++;
	if (canLogTail >= CAN_LOG_BUFFER_SIZE)
	{
		canLogTail = 0;
	}
}

//    __enable_irq();
}

/* Thread-safe pop operation for the 4G task loop */
uint8_t CANLog_Pop(CANLogPacket_t *pkt)
{
__disable_irq();

if (canLogCount == 0)
{
	__enable_irq();
	return 0; // Queue is empty
}

// Pull from the tail
pkt->frameType = canLogBuffer[canLogTail].frameType;
memcpy(pkt->data, (void*)canLogBuffer[canLogTail].data, 8);

canLogTail++;
if (canLogTail >= CAN_LOG_BUFFER_SIZE)
{
	canLogTail = 0;
}

canLogCount--; // Safely decrement atomic count

__enable_irq();
return 1; // Successfully popped
}

void CAN4G_Push(uint8_t frameType,
			uint8_t *data,
			uint8_t dlc)
{
__disable_irq();

if(can4GCount>=CAN_4G_QUEUE_SIZE)
{
	__enable_irq();
	return;
}

memcpy(
  (void*)can4GQueue[can4GHead].data,
  data,
  dlc);

can4GQueue[can4GHead].frameType=frameType;

can4GQueue[can4GHead].dlc=dlc;

can4GHead++;

if(can4GHead>=CAN_4G_QUEUE_SIZE)
	can4GHead=0;

can4GCount++;

__enable_irq();
}

uint8_t CAN4G_Pop(CAN4GPacket_t *pkt)
{
__disable_irq();

if(can4GCount==0)
{
	__enable_irq();
	return 0;
}

memcpy(pkt->data,
	   (void*)can4GQueue[can4GTail].data,
	   can4GQueue[can4GTail].dlc);

pkt->frameType=
	can4GQueue[can4GTail].frameType;

pkt->dlc=
	can4GQueue[can4GTail].dlc;

can4GTail++;

if(can4GTail>=CAN_4G_QUEUE_SIZE)
	can4GTail=0;

can4GCount--;

__enable_irq();

return 1;
}





/* TMCM CANReceive Frames 8 + 8 bytes */
void processTmcmMsg(uint8_t u8_rcvdPgn)
{
switch(u8_rcvdPgn)
{
/* ================= FRAME 1 ================= */
case PGN_TMCM_SPEED_CMD_MSG:
{
	if (canRxHeader.DataLength == FDCAN_DLC_BYTES_8)
	{
		/* Decode */
		uint16_t speed =
			((uint16_t)u8_canRxXfer[0] << 8) |
			((uint16_t)u8_canRxXfer[1]);

		uint16_t value2 =
			((uint16_t)u8_canRxXfer[2] << 8) |
			((uint16_t)u8_canRxXfer[3]);

		uint8_t value3 = u8_canRxXfer[4];

		uint16_t value4 =
			((uint16_t)u8_canRxXfer[5] << 8) |
			((uint16_t)u8_canRxXfer[6]);

		uint8_t fault = u8_canRxXfer[7];

		/* Debug */
		dbg_speed  = speed;
		dbg_val2   = value2;
		dbg_val3   = value3;
		dbg_val4   = value4;
		dbg_fault  = fault;

		/* Store Frame 1 */
		memcpy((void*)canFrame1, u8_canRxXfer, 8);
		canFrame1Ready = 1;

		/* ===== ADD 4G path ===== */

//          memcpy((void*)canTo4GBuffer, u8_canRxXfer, 8);
//          can4GFrameType = 1;
//          canDataReady=1;
//            CAN4G_Push(
//                  1,
//                  u8_canRxXfer,
//                  8);
		StoreCANLog(1, u8_canRxXfer);
	}
	break;
}

/* ================= FRAME 2 ================= */
case PGN_TMCM_SPEED_CMD_MSG_2:
{
	if (canRxHeader.DataLength == FDCAN_DLC_BYTES_8)
	{
		memcpy((void*)canFrame2, u8_canRxXfer, 8);
		canFrame2Ready = 1;

		/* ADD FOR 4G */
//            memcpy((void*)canTo4GBuffer,u8_canRxXfer, 8);
//            can4GFrameType = 2;
//            canDataReady=1;
//            CAN4G_Push(2,u8_canRxXfer,8);
		StoreCANLog(2, u8_canRxXfer);
	}
	break;
}

default:
	break;
}
}

void processCmcmMsg(uint8_t u8_rcvdPgn)
{

}

void processBmsMsg(uint8_t u8_rcvdPgn)
{
if (canRxHeader.DataLength != FDCAN_DLC_BYTES_8)
		return;

	switch(u8_rcvdPgn)
	{
		case PGN_BMS_INSTANT_STATUS:   // 0x03
		{
			memcpy((void*)canBmsFrame, u8_canRxXfer, 8);
			canBmsReady = 1;

			/* 4G */
//	            memcpy((void*)canTo4GBuffer,u8_canRxXfer,8);
//	            can4GFrameType = 3;
//	            canDataReady=1;
//	            CAN4G_Push(3,u8_canRxXfer,8);
			StoreCANLog(3, u8_canRxXfer);
			break;
		}

		default:
			break;
	}
}

void processDcdc12vPpmMsg(uint8_t u8_rcvdPgn)
{
if (canRxHeader.DataLength != FDCAN_DLC_BYTES_8)
		return;

	switch(u8_rcvdPgn)
	{
		case PGN_DCDC12V_STATUS_MSG:
		case PGN_DCDC12V_VERSION_NUMBER_MSG:
		{
			memcpy((void*)canDcdc12vFrame, u8_canRxXfer, 8);
			canDcdc12vReady = 1;

			/* 4G */
//					memcpy((void*)canTo4GBuffer, u8_canRxXfer,8);
//					can4GFrameType = 4;
//					canDataReady=1;
			CAN4G_Push(4,u8_canRxXfer,8);
//	            StoreCANLog(4, u8_canRxXfer);
			break;
		}

		default:
			break;
	}

}

void processDcDc48vAcMsg(uint8_t u8_rcvdPgn)
{
if (canRxHeader.DataLength != FDCAN_DLC_BYTES_8)
		return;

	switch(u8_rcvdPgn)
	{
		case PGN_DCDC48V_STATUS_MSG:
		case PGN_DCDC48V_VERSION_NUMBER_MSG:
		{
			memcpy((void*)canDcdc48vFrame, u8_canRxXfer, 8);
			canDcdc48vReady = 1;

			/* 4G */
//	           	memcpy((void*)canTo4GBuffer, u8_canRxXfer,8);
//	           	can4GFrameType = 5;
//	           	canDataReady=1;
			CAN4G_Push(5,u8_canRxXfer,8);
//	            StoreCANLog(5, u8_canRxXfer);
			break;
		}

		default:
			break;
	}

}



//uint8_t canTransmit(uint8_t u8_priority, uint8_t u8_msgPgn, uint8_t u8_srcAddr, uint8_t u8_dstAddr, uint8_t size, uint8_t* dataXfer)
//{
//uint8_t u8_retVal = 255;
//if(size<9)
//{
//	memcpy(&u8_canTxXfer[0], dataXfer, size);
//	canTxHeader.IdType = FDCAN_EXTENDED_ID;
//	canTxHeader.Identifier = (uint32_t)((u8_priority<<24)|(u8_msgPgn<<16)|(u8_dstAddr<<8)|(u8_srcAddr));
//	canTxHeader.DataLength = size;
//	canTxHeader.TxFrameType = FDCAN_DATA_FRAME;
//	canTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//	canTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//	canTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//	canTxHeader.MessageMarker = 0;
//	canTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//
//	memcpy((void*)dbg_lastTxData, dataXfer,size);
//	dbg_lastTxIdentifier= canTxHeader.Identifier;
//	dbg_lastTxSize= size;
//
//	if(b_selectedCan==CAN_BUS_2)
//	{
//		u8_retVal = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &canTxHeader,  &u8_canTxXfer[0]);
//	}
//	else
//	{
//		u8_retVal = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &canTxHeader,  &u8_canTxXfer[0]);
//	}
////		memset(&u8_canTxXfer[0], '\0', 8);
//	return u8_retVal;
//}
//else
//{
////		u8_retVal = 1;
////		return u8_retVal;
//	return 1;
//}
//}
//


//void canReceive()
//{
//__disable_irq(); // 🔒 Interrupts Disabled
//u32_latestCan1PktTick = (uint32_t)uwTick;
//
//uint32_t id = canRxHeader.Identifier;
//
//uint8_t u8_priority, u8_rcvdPgn, u8_dstAddr, u8_srcAddr;
//u8_priority = (uint8_t)((id >> 24) & 0xFF);
//u8_rcvdPgn  = (uint8_t)((id >> 16) & 0xFF);
//u8_dstAddr  = (uint8_t)((id >> 8)  & 0xFF);
//u8_srcAddr  = (uint8_t)((id)       & 0xFF);
//
///* ===================================================== */
///* DIRECT MATCH RAW IDENTIFIERS (Chargers / Converters) */
///* ===================================================== */
//if (id == 0x07791020)   //ack // wc
//{
//	dbg_tmcmAck_id_count++;
//	memcpy((void*)canTmcmAckFrame, u8_canRxXfer, 8);
//	memcpy((void*)dbg_last_tmcmAck_data, u8_canRxXfer, 8); // Visible in Watch Window
//	canTmcmAckReady = 1;                  // Push to server queue using a dedicated Frame Type '10'
//	tmcmCmdAckReceived = 1;               // Signal StartTask07 that command ACK arrived
//	if(ackPendingHW)                      // UART command waiting for HW ACK
//	{
//	    ackPending = 1;                   // now safe to publish to MQTT
//	    ackPendingHW = 0;
//	}
//	CAN4G_Push(10, u8_canRxXfer,8);
//	__enable_irq();
//	return;
//}
//else if (id == 0x07801020) // original can for continous rpm
//   {
//	dbg_RpmPgn_count++;
//	memcpy((void*)canRpmPgnFrame, u8_canRxXfer, 8);
//	memcpy((void*)dbg_RpmPgn_data, u8_canRxXfer, 8); // Visible in Watch Window
//	lastRpmPgnRxTick = HAL_GetTick();
//	rpmrecflag=1;
//	StoreCANLog(1, u8_canRxXfer);
//	__enable_irq();
//	return;
//   }
//else if (id == 0x07811020)  // V2_1 data (PGN 0x80, heartbeat) was 0x07801020
//   {
//	dbg_RpmPgn_count1++;
//	memcpy((void*)canRpmPgnFrame1, u8_canRxXfer, 8);
//	memcpy((void*)dbg_RpmPgn_data1, u8_canRxXfer, 8); // Visible in Watch Window
//	lastRpmPgnRxTick = HAL_GetTick();
//	StoreCANLog(11, u8_canRxXfer);
//	__enable_irq();
//	return;
//   }
//else if (id == 0x7821020)  // V2_1 data (PGN 0x80, heartbeat) was 0x07801020
//   {
//	dbg_RpmPgn_count1++;
//	memcpy((void*)canRpmPgnFrame2, u8_canRxXfer, 8);
////	memcpy((void*)dbg_RpmPgn_data1, u8_canRxXfer, 8); // Visible in Watch Window
//	lastRpmPgnRxTick = HAL_GetTick();
//	StoreCANLog(13, u8_canRxXfer);
//	tmcmProfileAbort=1;
//	__enable_irq();
//	return;
//   }
//else  if (id == 0x18211040)   // 12V DC
//{
//	dbg_dcdc12v_id_count++;
//	memcpy((void*)canDcdc12vFrame, u8_canRxXfer, 8);
//	memcpy((void*)dbg_last_dcdc12v_data, u8_canRxXfer, 8);
//	canDcdc12vReady = 1;
//
//	CAN4G_Push(4, u8_canRxXfer, 8);
//
//	__enable_irq(); //   FIXED: Must unlock before returning!
//	return;
//}
////can logging
///*else  if (id == 0x07812010)   // TMCM V2_2 data (PGN 0x81) was 0x07811020
//{
//	dbg_TmcmV2_2_count++;
//	memcpy((void*)canTmcmV2_2Frame, u8_canRxXfer, 8);
//	memcpy((void*)dbg_TmcmV2_2_data, u8_canRxXfer, 8);
//	StoreCANLog(12, u8_canRxXfer);
//	__enable_irq();
//	return;
//}*/
//
///* ===== ADDED: DIRECT MATCH FOR BMS ===== */
//	else if (id == 0x18030000) //BMS Instantaneous Value(BIV)
//	{
//		dbg_bms_id_count++;
//		memcpy((void*)canBmsFrame, u8_canRxXfer, 8);
//		memcpy((void*)dbg_last_bms_data, u8_canRxXfer, 8); // For IDE Debugging Live Watch
//		canBmsReady = 1;
//		StoreCANLog(3, u8_canRxXfer);
//
//		__enable_irq(); // 🔓 Safely unlock before returning
//		return;
//	}
//
//else if (id == 0x18090000)   // Min and Max cell voltage and Temperature Information
//{
//	dbg_dcdc48v_id_count++;
//	uint8_t copyLen = (canRxHeader.DataLength <= 8) ? canRxHeader.DataLength : 8;
//
//	memset((void*)canDcdc48vFrame, 0, 8);
//	memcpy((void*)canDcdc48vFrame, u8_canRxXfer, copyLen);
//	memcpy((void*)dbg_last_dcdc48v_data, u8_canRxXfer, 8);
//	canDcdc48vReady = 1;
//
//	CAN4G_Push(5, u8_canRxXfer, 8);
//
//	__enable_irq(); //  🔓 FIXED: Must unlock before returning!
//	return;
//}
///* NEWLY REQUESTED TARGET LOGIC IDENTIFIERS */
//	else if (id == 0x18130000)  //BMS enerygy values
//	{
//		dbg_dev1300_id_count++;
//		memcpy((void*)canDevice1300Frame, u8_canRxXfer, 8);
//		memcpy((void*)dbg_last_dev1300_data, u8_canRxXfer, 8);
//		canDevice1300Ready = 1;
//		StoreCANLog(6, u8_canRxXfer); // Type 6 logged
//		__enable_irq();
//		return;
//	}
//	else if (id == 0x1810FF00) //BMS alarm status
//	{
//		dbg_dev10FF_id_count++;
//		memcpy((void*)canDevice10FFFrame, u8_canRxXfer, 8);
//		memcpy((void*)dbg_last_dev10FF_data, u8_canRxXfer, 8);
//		canDevice10FFReady = 1;
//		StoreCANLog(7, u8_canRxXfer); // Type 7 logged
//		__enable_irq();
//		return;
//	}
//	else if (id == 0x18060000) //Battery cell temperature
//	{
//		dbg_dev0600_id_count++;
//		memcpy((void*)canDevice0600Frame, u8_canRxXfer, 8);
//		memcpy((void*)dbg_last_dev0600_data, u8_canRxXfer, 8);
//		canDevice0600Ready = 1;
//		StoreCANLog(8, u8_canRxXfer); // Type 8 logged
//		__enable_irq();
//		return;
//	}
//	else if (id == 0x18070000) //Battery cell volatge
//	{
//		dbg_dev0700_id_count++;
//		memcpy((void*)canDevice0700Frame, u8_canRxXfer, 8);
//		memcpy((void*)dbg_last_dev0700_data, u8_canRxXfer, 8);
//		canDevice0700Ready = 1;
//		StoreCANLog(9, u8_canRxXfer); // Type 9 logged
//		__enable_irq();
//		return;
//	}
//
///* ===================================================== */
///* EXISTING LOGIC (UNCHANGED) */
///* ===================================================== */
//if(u8_dstAddr == PCM_MASTER_ADDR ||
//   u8_dstAddr == BROADCAST_ADDR ||
//   u8_dstAddr == 0x00)
//{
//	switch(u8_srcAddr)
//			{
//			case TMCM_ADDR:
//			{
//			//	processTmcmMsg(u8_rcvdPgn);
//				break;
//			}
//
//			case CMCM_ADDR:
//			{
//				processCmcmMsg(u8_rcvdPgn);
//				break;
//			}
//			case BMS_ADDR:
//			{
//				dbg_bms_id_count++;
//				memcpy((void*)dbg_last_bms_data, u8_canRxXfer, 8);
//				processBmsMsg(u8_rcvdPgn);
//				break;
//			}
//
//
//			case DCDC12V_ADDR:
//						processDcdc12vPpmMsg(u8_rcvdPgn);
//						break;
//
//			case DCDC48V_ADDR:
//			{
//				processDcDc48vAcMsg(u8_rcvdPgn);
//				break;
//			}
//
//			default:
//				break;
//			}
//
//}
//
//__enable_irq(); // 🔓 Normal path unlock
//}

void canReceive()
{
    u32_latestCan1PktTick = (uint32_t)uwTick;

    uint32_t id = canRxHeader.Identifier;
    uint8_t dlc = (uint8_t)canRxHeader.DataLength;

    /* Convert FDCAN DLC enum format to standard byte size if required by your HAL */
    if (dlc > 8) dlc = 8;

    uint8_t activeBus = (b_selectedCan == CAN_BUS_1) ? 1 : 2;

    /* ============================================================= */
    /* 1. STORE ALL RAW CAN FRAMES INTO RAM AUTOMATICALLY            */
    /* ============================================================= */
    CAN_RAM_StoreFrame(id, u8_canRxXfer, dlc, activeBus);

    /* ============================================================= */
    /* 2. SPECIFIC DEVICE ID PROCESSING & TELEMETRY                  */
    /* ============================================================= */

    if (id == 0x07791020) // TMCM ACK
    {
        dbg_tmcmAck_id_count++;
        memcpy((void*)canTmcmAckFrame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_tmcmAck_data, u8_canRxXfer, 8);
        canTmcmAckReady = 1;
        tmcmCmdAckReceived = 1;
        if(ackPendingHW)
        {
            ackPending = 1;
            ackPendingHW = 0;
        }
        CAN4G_Push(10, u8_canRxXfer, dlc);
        return;
    }
    else if (id == 0x07801020) // Continuous RPM
    {
        dbg_RpmPgn_count++;
        memcpy((void*)canRpmPgnFrame, u8_canRxXfer, 8);
        memcpy((void*)dbg_RpmPgn_data, u8_canRxXfer, 8);
        lastRpmPgnRxTick = HAL_GetTick();
        rpmrecflag = 1;
        StoreCANLog(1, u8_canRxXfer);
        return;
    }
    else if (id == 0x07811020) // V2_1 Heartbeat
    {
        dbg_RpmPgn_count1++;
        memcpy((void*)canRpmPgnFrame1, u8_canRxXfer, 8);
        memcpy((void*)dbg_RpmPgn_data1, u8_canRxXfer, 8);
        lastRpmPgnRxTick = HAL_GetTick();
        StoreCANLog(11, u8_canRxXfer);
        return;
    }
    else if (id == 0x07821020) // Profile Abort
    {
        dbg_RpmPgn_count1++;
        memcpy((void*)canRpmPgnFrame2, u8_canRxXfer, 8);
        lastRpmPgnRxTick = HAL_GetTick();
        StoreCANLog(13, u8_canRxXfer);
        //tmcmProfileAbort = 1;
        return;
    }
    else if (id == 0x18211040) // 12V DC
    {
        dbg_dcdc12v_id_count++;
        memcpy((void*)canDcdc12vFrame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_dcdc12v_data, u8_canRxXfer, 8);
        canDcdc12vReady = 1;
        CAN4G_Push(4, u8_canRxXfer, 8);
        return;
    }
    else if (id == 0x18030000) // BMS Instantaneous Value
    {
        dbg_bms_id_count++;
        memcpy((void*)canBmsFrame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_bms_data, u8_canRxXfer, 8);
        canBmsReady = 1;
        StoreCANLog(3, u8_canRxXfer);
        return;
    }
    else if (id == 0x18090000) // Cell Voltage / Temp
    {
        dbg_dcdc48v_id_count++;
        memset((void*)canDcdc48vFrame, 0, 8);
        memcpy((void*)canDcdc48vFrame, u8_canRxXfer, dlc);
        memcpy((void*)dbg_last_dcdc48v_data, u8_canRxXfer, 8);
        canDcdc48vReady = 1;
        CAN4G_Push(5, u8_canRxXfer, 8);
        return;
    }
    else if (id == 0x18130000) // BMS Energy Values
    {
        dbg_dev1300_id_count++;
        memcpy((void*)canDevice1300Frame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_dev1300_data, u8_canRxXfer, 8);
        canDevice1300Ready = 1;
        StoreCANLog(6, u8_canRxXfer);
        return;
    }
    else if (id == 0x1810FF00) // BMS Alarm
    {
        dbg_dev10FF_id_count++;
        memcpy((void*)canDevice10FFFrame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_dev10FF_data, u8_canRxXfer, 8);
        canDevice10FFReady = 1;
        StoreCANLog(7, u8_canRxXfer);
        return;
    }
    else if (id == 0x18060000) // Battery Cell Temp
    {
        dbg_dev0600_id_count++;
        memcpy((void*)canDevice0600Frame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_dev0600_data, u8_canRxXfer, 8);
        canDevice0600Ready = 1;
        StoreCANLog(8, u8_canRxXfer);
        return;
    }
    else if (id == 0x18070000) // Battery Cell Voltage
    {
        dbg_dev0700_id_count++;
        memcpy((void*)canDevice0700Frame, u8_canRxXfer, 8);
        memcpy((void*)dbg_last_dev0700_data, u8_canRxXfer, 8);
        canDevice0700Ready = 1;
        StoreCANLog(9, u8_canRxXfer);
        return;
    }

    /* PGN-based Decoding Fallback */
    uint8_t u8_dstAddr = (uint8_t)((id >> 8) & 0xFF);
    uint8_t u8_srcAddr = (uint8_t)(id & 0xFF);
    uint8_t u8_rcvdPgn = (uint8_t)((id >> 16) & 0xFF);

    if (u8_dstAddr == PCM_MASTER_ADDR || u8_dstAddr == BROADCAST_ADDR || u8_dstAddr == 0x00)
    {
        switch(u8_srcAddr)
        {
            case TMCM_ADDR:    processTmcmMsg(u8_rcvdPgn); break;
            case CMCM_ADDR:    processCmcmMsg(u8_rcvdPgn); break;
            case BMS_ADDR:     dbg_bms_id_count++; processBmsMsg(u8_rcvdPgn); break;
            case DCDC12V_ADDR: processDcdc12vPpmMsg(u8_rcvdPgn); break;
            case DCDC48V_ADDR: processDcDc48vAcMsg(u8_rcvdPgn); break;
            default: break;
        }
    }
}

void canInit()
{
/************Initialize CAN 1***************/
if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
{
	Error_Handler();
}

FDCAN_FilterTypeDef sFilterConfig_fdcan1;

sFilterConfig_fdcan1.IdType = FDCAN_EXTENDED_ID;
sFilterConfig_fdcan1.FilterIndex = 0;
sFilterConfig_fdcan1.FilterType = FDCAN_FILTER_MASK;
sFilterConfig_fdcan1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
sFilterConfig_fdcan1.FilterID1 = 0x00;
sFilterConfig_fdcan1.FilterID2 = 0x7FFFFFFF;

if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig_fdcan1) != HAL_OK)
{
	/* Filter configuration Error */
	Error_Handler();
}

if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
{
	/* Notification Error */
	Error_Handler();
}

/************Initialize CAN 2***************/
if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK)
{
	Error_Handler();
}

FDCAN_FilterTypeDef sFilterConfig_fdcan2;

sFilterConfig_fdcan2.IdType = FDCAN_EXTENDED_ID;
sFilterConfig_fdcan2.FilterIndex = 0;
sFilterConfig_fdcan2.FilterType = FDCAN_FILTER_MASK;
sFilterConfig_fdcan2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
sFilterConfig_fdcan2.FilterID1 = 0x00;
sFilterConfig_fdcan2.FilterID2 = 0x7FFFFFFF;

if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig_fdcan2) != HAL_OK)
{
	/* Filter configuration Error */
	Error_Handler();
}

if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
{
	/* Notification Error */
	Error_Handler();
}
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
						   uint32_t RxFifo0ITs)
{
if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
{
	if (HAL_FDCAN_GetRxMessage(hfdcan,
							   FDCAN_RX_FIFO0,
							   &canRxHeader,
							   u8_canRxXfer) != HAL_OK)
	{
		Error_Handler();
	}

	/* Debug */
	dbg_lastIdentifier = canRxHeader.Identifier;
	memcpy((void*)dbg_lastData, u8_canRxXfer, 8);

	if (hfdcan == &hfdcan1)
	{
		u32_latestCan1PktTick = uwTick;
		b_selectedCan = CAN_BUS_1;
	}
	else if (hfdcan == &hfdcan2)
	{
		u32_latestCan2PktTick = uwTick;
		b_selectedCan = CAN_BUS_2;
	}

	/* Process message */
	counter++;
	canReceive();
}
}


void tmcm_update_motor(uint16_t speed,
	uint16_t value2,
	 uint8_t value3,
	uint16_t value4,
	 uint8_t fault)
{
// Temporary implementation
// Replace with real motor logic

if(speed == 1)
{
	// Start motor
}
else
{
	// Stop motor
}
}

