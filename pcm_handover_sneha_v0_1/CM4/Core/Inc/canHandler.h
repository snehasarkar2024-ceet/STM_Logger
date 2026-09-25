/*
 * can_handler.h
 *
 *  Created on: Oct 9, 2025
 *      Author: flowtrik-3
 */

//#ifndef CAN_HANDLER_CANHANDLER_H_
//#define CAN_HANDLER_CANHANDLER_H_f

#ifndef CAN_HANDLER_CANHANDLER_H_
#define CAN_HANDLER_CANHANDLER_H_

#include "stdbool.h"
#include "string.h"
#include "stm32h745xx.h"
#include "stm32h7xx.h"
#include <stdint.h>
#include "stm32h7xx_hal.h"

//#include "../../../CM4/Core/Inc/bms_handler_v0_0.h"
//#include "../../../CM4/Core/Inc/cmcm_handler_v0_0.h"
//#include "../../../CM4/Core/Inc/dcDc12v_handler.h"
//#include "../../../CM4/Core/Inc/dcDc48v_handler.h"
//#include "../../../CM4/Core/Inc/lte_module_handler_v0_0.h"
//#include "../../../CM4/Core/Inc/redundantPcm_handler.h"
//#include "../../../CM4/Core/Inc/tmcm_handler_v0_0.h"

extern FDCAN_TxHeaderTypeDef hcan1;

#define PCM_MASTER_ADDR								(uint8_t)0x10
#define BROADCAST_ADDR								(uint8_t)0xFF

/*********************TMCM MESSAGES**************************/

#define TMCM_CAN_PRIORITY							(uint8_t)0x07
#define PGN_TMCM_VERSION_NUMBER_MSG					(uint8_t)0x70
#define PGN_TMCM_PERF_STATUS_MSG					(uint8_t)0x74
#define PGN_TMCM_STATUS_MSG							(uint8_t)0x75
#define PGN_TMCM_CONTACTOR_CMD_MSG					(uint8_t)0x76
//#define PGN_TMCM_PLATOON_CMD_MSG					(uint8_t)0x77
#define PGN_TMCM_START_STOP_CMD_MSG					(uint8_t)0x77
#define PGN_TMCM_SPEED_CMD_MSG						(uint8_t)0x78
#define PGN_TMCM_SPEED_CMD_MSG_2                    (uint8_t)0x79
#define ADDR_ASSGN_MSG								(uint8_t)0x24
#define TMCM_ADDR									(uint8_t)0x20
#define RPM_PGN                                     (uint8_t)0x80

#define TMCM_VERS_MSG								((uint8_t)TMCM_CAN_PRIORITY, (uint8_t)PGN_TMCM_VERSION_NUMBER_MSG, (uint8_t)PCM_MASTER_ADDR, (uint8_t)TMCM_ADDR, (uint8_t)sizeof(tmcmVersion_g), (uint8_t*)&tmcmVersion_g);
#define TMCM_SPEED_MSG								((uint8_t)TMCM_CAN_PRIORITY, (uint8_t)PGN_TMCM_SPEED_CMD_MSG, (uint8_t)PCM_MASTER_ADDR, (uint8_t)TMCM_ADDR, (uint8_t)sizeof(tmcmSpeedCmd_g), (uint8_t*)&tmcmSpeedCmd_g);
#define TMCM_CONTACTOR_MSG							((uint8_t)TMCM_CAN_PRIORITY, (uint8_t)PGN_TMCM_CONTACTOR_CMD_MSG, (uint8_t)PCM_MASTER_ADDR, (uint8_t)TMCM_ADDR, (uint8_t)sizeof(tmcmContactorCmd_g), (uint8_t*)&tmcmContactorCmd_g);
#define TMCM_START_STOP_MSG							((uint8_t)TMCM_CAN_PRIORITY, (uint8_t)PGN_TMCM_START_STOP_CMD_MSG, (uint8_t)PCM_MASTER_ADDR, (uint8_t)TMCM_ADDR, (uint8_t)sizeof(tmcmStartStopCmd_g), (uint8_t*)&tmcmStartStopCmd_g);



/*********************CMCM W/ BRAKE MESSAGES**************************/

#define CMCM_CAN_PRIORITY							(uint8_t)0x18
#define PGN_CMCM_VERSION_NUMBER_MSG					(uint8_t)0x30
#define PGN_CMCM_STATUS_MSG							(uint8_t)0x31
#define PGN_CMCM_SENSOR_STATUS_MSG					(uint8_t)0x32
#define PGN_CMCM_ACTION_MSG							(uint8_t)0x33
#define CMCM_ADDR									(uint8_t)0x30

#define CMCM_ACTION_MSG								((uint8_t)CMCM_CAN_PRIORITY, (uint8_t)PGN_CMCM_ACTION_MSG, (uint8_t)PCM_MASTER_ADDR, (uint8_t)CMCM_ADDR, (uint8_t)sizeof(cmcmAction_g), (uint8_t*)&cmcmAction_g);

/*********************BMS MESSAGES**************************/

#define BMS_CAN_PRIORITY							(uint8_t)0x18
#define PGN_BMS_INSTANT_STATUS						(uint8_t)0x03
#define BMS_ADDR									(uint8_t)0x00

/*********************CSIM MESSAGES**************************/




/*********************12V DC DC CONVERTER + PPM MESSAGES**************************/

#define DC_12V_CONV_PPM_CAN_PRIORITY		(uint8_t)0x18
#define PGN_DCDC12V_VERSION_NUMBER_MSG		(uint8_t)0x20
#define PGN_DCDC12V_STATUS_MSG				(uint8_t)0x21
#define DCDC12V_ACTION_CMD_MSG				(uint8_t)0x23
#define ADDR_ASSGN_MSG						(uint8_t)0x24
#define DCDC12V_ADDR						(uint8_t)0x40

/*********************48V DC DC CONVERTER + PPM MESSAGES**************************/

#define DC_48V_CONV_PPM_CAN_PRIORITY		(uint8_t)0x18
#define PGN_DCDC48V_VERSION_NUMBER_MSG		(uint8_t)0xFF
#define PGN_DCDC48V_STATUS_MSG				(uint8_t)0x61 //based on roshan's doc
//#define PGN_DCDC48V_STATUS_MSG				(uint8_t)0xFE
#define DCDC48V_ADDR						(uint8_t)0xFD

#define CAN_BOOTUP_DURATION					(uint16_t)10000
#define CAN_BUS_1							(bool)0
#define CAN_BUS_2							(bool)1
#define CAN_SWITCHOVER_LIMIT				(uint8_t)5
#define MAX_DURATION_THRESHOLD_CAN_MESSAGE	(uint16_t)2500

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern uint32_t u32_latestCan1PktTick;
extern uint32_t u32_latestCan2PktTick;

extern volatile uint8_t canToEspBuffer[8];
extern volatile uint8_t canToEspFlag;
extern volatile uint8_t canTo4GBuffer[8];
extern volatile uint8_t canDataReady;
extern volatile uint8_t can4GFrameType;
extern volatile uint8_t canFrame1Ready;
extern volatile uint8_t canFrame2Ready;
extern volatile uint8_t canFrame1[8];
extern volatile uint8_t canFrame2[8];
#define CAN_TX_INTERVAL_MS 20

/* DCDC 12V */
extern volatile uint8_t canDcdc12vFrame[8];
extern volatile uint8_t canDcdc12vReady;

/* DCDC 48V */
extern volatile uint8_t canDcdc48vFrame[8];
extern volatile uint8_t canDcdc48vReady;

/* BMS Frame Buffer */
extern volatile uint8_t canBmsFrame[8];
extern volatile uint8_t canBmsReady;
extern volatile uint8_t canBmsFrame1[8];
extern volatile uint8_t canBmsReady1;
uint8_t canTransmit(uint8_t u8_priority, uint8_t u8_msgPgn, uint8_t u8_srcAddr, uint8_t u8_dstAddr, uint8_t size, uint8_t* dataXfer);
void canReceive();

extern volatile uint32_t dbg_tmcmAck_id_count;
extern volatile uint8_t  canTmcmAckFrame[8];
extern volatile uint8_t  dbg_last_tmcmAck_data[8];
extern volatile uint8_t  canTmcmAckReady;

extern volatile uint32_t dbg_RpmPgn_count;
extern volatile uint8_t canRpmPgnFrame[8];
extern volatile uint8_t dbg_RpmPgn_data[8];

extern volatile uint8_t canRpmPgnFrame1[8];
extern volatile uint8_t dbg_RpmPgn_data1[8];

extern volatile uint32_t dbg_TmcmV2_2_count;
extern volatile uint8_t  canTmcmV2_2Frame[8];
extern volatile uint8_t  dbg_TmcmV2_2_data[8];

extern volatile uint32_t lastRpmPgnRxTick;
extern volatile uint8_t  tmcmCmdAckReceived;
extern volatile uint8_t  tmcmProfileAbort;
extern volatile uint8_t  tmcmMotorRunning;   /* UART RUN command active */

/* ACK handshake — delay MQTT publish until 0x07792010 arrives */
extern volatile uint8_t  ackPending;
extern volatile uint8_t  ackPendingHW;

typedef struct
{
    uint8_t frameType;
    uint8_t data[8];
    uint8_t dlc;

}CAN4GPacket_t;

#define CAN_LOG_BUFFER_SIZE 1000 //was 300

typedef struct
{
    uint8_t frameType;
    uint8_t data[8];
    uint16_t seq;
    char time_var[30];

} CANLogPacket_t;

uint8_t CAN4G_Pop(CAN4GPacket_t *pkt);
void CAN4G_Push(uint8_t frameType,
                uint8_t *data,
                uint8_t dlc);


extern volatile CANLogPacket_t canLogBuffer[CAN_LOG_BUFFER_SIZE];

extern volatile uint16_t canLogIndex;
extern volatile uint16_t can4GCount;
extern volatile uint16_t canLogHead;
extern volatile uint16_t canLogTail;
extern volatile uint16_t canLogCount;

/* NEWLY ADDED DEVICE BUFFERS */
extern volatile uint8_t canDevice1300Frame[8];
extern volatile uint8_t canDevice1300Ready;

extern volatile uint8_t canDevice10FFFrame[8];
extern volatile uint8_t canDevice10FFReady ;

extern volatile uint8_t canDevice0600Frame[8];
extern volatile uint8_t canDevice0600Ready;

extern volatile uint8_t canDevice0700Frame[8];
extern volatile uint8_t canDevice0700Ready;
extern volatile uint32_t dbg_RpmPgn_count1;
extern volatile uint8_t rpmrecflag;

extern volatile uint32_t dbg_rtc_broadcast_count;
extern volatile uint8_t  dbg_rtc_broadcast_last_data[8];
extern volatile uint8_t  ntpSyncSuccessFlag;
extern volatile uint8_t  ntpBroadcastSent;
extern volatile uint32_t dbg_rtc_sync_tx_count;
extern volatile uint8_t  dbg_rtc_sync_last_data[8];
void canInit(void);
//uint8_t CAN_SendRTCSyncToTarget(void);

/*Extra function for RTC monitor*/
extern volatile uint8_t rtc_live_hour;
extern volatile uint8_t rtc_live_minute;
extern volatile uint8_t rtc_live_second;

extern volatile uint8_t rtc_live_year ;
extern volatile uint8_t rtc_live_month;
extern volatile uint8_t rtc_live_date ;

/* Configurable RAM Storage Queue Size */
#define CAN_RAM_LOG_SIZE    7200 /* Adjust size based on available RAM */
extern uint8_t can_log_buffer[CAN_RAM_LOG_SIZE];

/* Structure to store complete CAN frame details in RAM */
typedef struct {
    uint32_t canId;       /* Extended (29-bit) or Standard (11-bit) CAN ID */
    uint32_t timestamp;   /* HAL_GetTick() timestamp */
    uint8_t  dlc;         /* Data length (0 - 8 bytes) */
    uint8_t  busId;       /* 1 = CAN1, 2 = CAN2 */
    uint8_t  data[8];     /* Payload bytes */
} CAN_RAM_Log_t;

/* Global RAM Storage Structure */
typedef struct {
    volatile CAN_RAM_Log_t buffer[CAN_RAM_LOG_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile uint32_t overflowCount;
} CAN_RAM_Storage_t;

extern CAN_RAM_Storage_t g_canRamStorage;

/* Storage API Prototypes */
void CAN_RAM_Init(void);
void CAN_RAM_StoreFrame(uint32_t canId, uint8_t *pData, uint8_t dlc, uint8_t busId);
uint8_t CAN_RAM_ReadFrame(CAN_RAM_Log_t *outFrame);
uint16_t CAN_RAM_GetCount(void);
void CAN_RAM_Clear(void);

extern volatile uint8_t g_sendCanRamDataFlag;
extern UART_HandleTypeDef huart7;
void CheckServerCommand(const char *rxBuffer);
void CAN_RAM_FlushToESP(void);

extern volatile uint8_t  g_sendCanRamDataFlag;
extern volatile uint32_t dbg_ram_tx_count;    /* Total frames successfully sent to ESP */
extern volatile uint32_t dbg_ram_tx_bytes;    /* Total bytes sent over UART7 */
extern volatile uint32_t dbg_ram_last_can_id; /* Last CAN ID sent to ESP */
#endif /* CAN_HANDLER_CANHANDLER_H_ */
