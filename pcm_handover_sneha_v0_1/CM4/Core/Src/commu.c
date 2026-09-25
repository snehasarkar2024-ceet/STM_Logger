/*
 * commu.c
 *
 *  Created on: 21-Sept-2026
 *      Author: ITEL1
 */
#include "commu.h"
#include "canhandler.h"

uint8_t rxBuffer[5];
uint8_t UART5_Data[4096]={0}; //was 100
uint8_t receivedCount=0;

volatile uint8_t rtcSynced=0;
volatile uint8_t rtcState=0;
volatile uint8_t rtcSyncStarted=0;
volatile uint8_t waitingCCLK=0;
uint8_t telemetryIndex = 0;
volatile uint8_t gsm_csq = 99;
volatile int16_t gsm_rssi = -999;
volatile uint8_t modemFaultActive = 0;
static uint32_t modemFaultTimer = 0;
uint8_t mqttCmdReady=0;
/* In commu.c */
volatile char g_cmdType[20];
char dataBuffer[2048];
uint8_t dataTimer;
volatile uint8_t initial_flag;



/*Queue for 4G modem*/
#define UART5_QUEUE_SIZE    15
#define UART5_LINE_SIZE     4096 //was 200

char uart5Queue[UART5_QUEUE_SIZE][UART5_LINE_SIZE];

volatile uint8_t uart5QWrite=0;
volatile uint8_t uart5QRead=0;

char currentRx[UART5_LINE_SIZE];
volatile uint8_t uart5TxBusy=0;

volatile uint8_t mqttPacketReady = 0;
volatile uint8_t dropcount;
volatile uint8_t Uart5transmit_errorflag=0;
char mqttPacketBuffer[4096];

char lastTx[600] = {0};
char lastRx[200] = {0};
uint8_t responseReady = 0;
volatile int step;

uint32_t stepTimer = 0;
uint8_t tcpRetryCount = 0;

uint8_t tcpConnected = 0;
uint32_t retryTimer = 0;
char ackBuffer[100];
//volatile uint8_t ackPending = 0;
volatile uint8_t ackSending = 0;
//volatile uint8_t ackPendingHW = 0;        /* waiting for 0x07792010 TMCM ACK before publishing */
volatile uint32_t lastAckHWTick = 0;
volatile uint32_t mqttPayloadcount=0;
volatile uint32_t canUploadTimer = 0;
volatile float g_cmdValue = 0.0f;


