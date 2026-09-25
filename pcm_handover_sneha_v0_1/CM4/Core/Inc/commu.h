/*
 * commu.h
 *
 *  Created on: 21-Sept-2026
 *      Author: ITEL1
 */
#ifndef INC_COMMU_H_
#define INC_COMMU_H_

#include <stdint.h>

/*4G Variables*/

#define STEP_INIT 0
#define STEP_AT 1
#define STEP_CPIN 2
#define STEP_CSQ 3
#define STEP_CGATT 4
#define STEP_CGDCONT 5
#define STEP_XIIC 6
#define STEP_TIMEOUT_MS 3000
#define KEEP_ALIVE_MS 5000
/*Mqtt variables*/
#define DEVICE_ID "STM_X"
#define MQTT_BASE "stm"

#define MQTT_TOPIC_CMD     MQTT_BASE "/" DEVICE_ID "/cmd"
#define MQTT_TOPIC_DATA    MQTT_BASE "/" DEVICE_ID "/data"
#define MQTT_TOPIC_CAN     MQTT_BASE "/" DEVICE_ID "/can"
#define MQTT_TOPIC_ACK     MQTT_BASE "/" DEVICE_ID "/ack"

#define STEP_NTP_SYNC           20
#define STEP_NTP_WAIT           21
#define STEP_CCLK_QUERY         22
#define STEP_CCLK_WAIT          23
#define STEP_MQTT_CONFIG        25
#define STEP_MQTTMUX_WAIT       26
#define STEP_MQTT_CONNECT       27
#define STEP_MQTT_CONNECT_WAIT  28
#define STEP_MQTT_SUB           29
#define STEP_CONNECTED          30
#define STEP_SEND_ACK_WAIT      31
#define STEP_SEND_DATA_WAIT     50
#define STEP_CSQ_UPDATE       60
#define MQTT_BROKER "121.242.232.220:5010"
extern uint8_t telemetryIndex;
// csq signal strength variables
extern volatile uint8_t gsm_csq;
extern volatile int16_t gsm_rssi;
extern volatile uint8_t modemFaultActive;
//static uint32_t modemFaultTimer;
//volatile uint8_t motorStopSent;
extern volatile uint8_t mqttPacketReady;

/* Change int8_t to uint8_t to match commu.c */
extern uint8_t rxBuffer[5];
extern uint8_t UART5_Data[4096];
extern uint8_t receivedCount;
extern volatile char g_cmdType[20];
extern volatile uint8_t rtcSynced;
extern volatile uint8_t rtcState;
extern volatile uint8_t rtcSyncStarted;
extern volatile uint8_t waitingCCLK;

extern char mqttCmdBuffer[4096];
extern uint8_t mqttCmdReady;
extern uint8_t dataTimer;
extern volatile uint8_t Uart5transmit_errorflag;

/*Queue for 4G modem*/
#define UART5_QUEUE_SIZE    15
#define UART5_LINE_SIZE     4096 //was 200
//void GetRTCTimestamp(char *buffer, size_t max_len);
//void ApplyCCLKToRTC(char *cclk_str);

extern char uart5Queue[UART5_QUEUE_SIZE][UART5_LINE_SIZE];

extern volatile uint8_t uart5QWrite;
extern volatile uint8_t uart5QRead;

extern char currentRx[UART5_LINE_SIZE];
extern volatile uint8_t uart5TxBusy;
extern char dataBuffer[2048];

extern char lastTx[600] ;
extern char lastRx[200];
extern uint8_t responseReady;
extern volatile int step;

extern uint32_t stepTimer;
extern uint8_t tcpRetryCount;

extern uint8_t tcpConnected;
extern uint32_t retryTimer;

extern char ackBuffer[100];
//extern volatile uint8_t ackPending;
extern volatile uint8_t ackSending ;
//extern volatile uint8_t ackPendingHW ;        /* waiting for 0x07792010 TMCM ACK before publishing */
extern volatile uint32_t lastAckHWTick;
extern volatile uint32_t mqttPayloadcount;
extern char mqttPacketBuffer[4096];
extern volatile uint8_t initial_flag;
extern volatile uint32_t canUploadTimer;
extern volatile uint8_t dropcount;
extern volatile float g_cmdValue;
#endif /* INC_COMMU_H_ */
