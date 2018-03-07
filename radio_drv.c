#include "radio_drv.h"
#include "radio.h"
#include "assert.h"

#include "FreeRTOS.h"
#include "semphr.h"
//~ #include "task.h"
#include "timers.h"

//~ #include <stdint.h>
//~ #include <stdio.h>
#include <string.h>
//~ #include <math.h>

//Lorawan on 868.10, 868.30  868.50 @125kHz(EU)
//defaults
#define RF_FREQUENCY                                868300000 // Hz
#define TX_OUTPUT_POWER                             0         // dBm
#define LORA_BANDWIDTH                              7         // [7: 125 kHz,
                                                              //  8: 250 kHz,
                                                              //  9: 500 kHz,
                                                              //  10: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             2         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         15         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

static SemaphoreHandle_t RadioSem;  //has radio
static QueueHandle_t rxEventQ, txEventQ;         //global
static RadioEvents_t RadioEvents;

typedef enum
{
    RXQ_DATA_RECEIVED = 0,
    RXQ_TIMEOUT,
    RXQ_ERROR,
    RXQ_INTERRUPTED,
    TXQ_DATA_SENT,
    TXQ_TIMEOUT,
} Event_t;

typedef struct
{
    Event_t eType;
    void *pvData;
    uint16_t Len;
    int16_t rssi;
    int8_t snr;
} EventQ_t;

void OnTxDone( void )    //ISR
{
    EventQ_t txnotify;
    
    txnotify.eType = TXQ_DATA_SENT;
    txnotify.pvData = NULL;
    txnotify.Len = 0;

    DEBUG_PRINT("TX DONE EVENT");
    Radio.Standby( );
    xQueueSendFromISR( txEventQ, &txnotify, NULL );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )    //ISR
{
    EventQ_t rxnotify;
    
    rxnotify.eType = RXQ_DATA_RECEIVED;
    rxnotify.pvData = payload;
    rxnotify.Len = size;
    rxnotify.rssi = rssi;
    rxnotify.snr = snr;

    Radio.Standby( );
    xQueueSendFromISR( rxEventQ, &rxnotify, NULL );

    //~ RssiValue = rssi;
    //~ SnrValue = snr;
}

void OnTxTimeout( void )    //ISR
{
    EventQ_t txnotify;
    
    txnotify.eType = TXQ_TIMEOUT;
    txnotify.pvData = NULL;
    txnotify.Len = 0;

    Radio.Standby( );
    xQueueSendFromISR( txEventQ, &txnotify, NULL );
}

void OnRxTimeout( void )    //ISR
{
    EventQ_t rxnotify;
    
    rxnotify.eType = RXQ_TIMEOUT;
    rxnotify.pvData = NULL;
    rxnotify.Len = 0;

    DEBUG_PRINT("on timeout");
    Radio.Standby( );
    xQueueSendFromISR( rxEventQ, &rxnotify, NULL );
}

void OnRxError( void )  //ISR
{    
    EventQ_t rxnotify;
    
    rxnotify.eType = RXQ_ERROR;
    rxnotify.pvData = NULL;
    rxnotify.Len = 0;

    Radio.Standby( );
    xQueueSendFromISR( rxEventQ, &rxnotify, NULL );
}

uint16_t radio_read(uint8_t *data, uint16_t maxlen, int16_t *rssi, int8_t *snr, TickType_t timeout) {
    EventQ_t  xReceivedStructure;
    BaseType_t xStatus;
    uint16_t ret = 0;

    if( xSemaphoreTake( RadioSem, timeout ) == pdTRUE ) {
        xQueueReset(rxEventQ);      //empty queue
        Radio.Rx( timeout );        //TODO: subtract already elapsed time?
        xSemaphoreGive( RadioSem );       //give back radio control, TX might interrupt
        xStatus = xQueueReceive( rxEventQ, &xReceivedStructure, portMAX_DELAY );    //no timeout
        if( xStatus == pdPASS ) {
            if( xReceivedStructure.eType == RXQ_DATA_RECEIVED ) {
                DEBUG_PRINT("RECV");
                //copy to caller
                if(xReceivedStructure.Len>maxlen) //TODO:packet length error
                    ret = maxlen;
                else
                    ret = xReceivedStructure.Len;
                *rssi = xReceivedStructure.rssi;
                *snr = xReceivedStructure.snr;

                memcpy(data, xReceivedStructure.pvData, ret);
            }
            else if( xReceivedStructure.eType == RXQ_TIMEOUT ) {
                DEBUG_PRINT("timeout");
            }
            else if( xReceivedStructure.eType == RXQ_INTERRUPTED ) {
                DEBUG_PRINT("interrupted");
            }
        }
        else {
            configASSERT(false);
        }
        //~ xSemaphoreGive( RadioSem );       //give back radio control, no TX interrupt
    }
    else {
    }

    return ret;
}

uint16_t radio_write(uint8_t *data, uint16_t len){
    EventQ_t rxnotify;

    EventQ_t xReceivedStructure;
    BaseType_t xStatus;
    uint16_t ret = 0;

    configASSERT(txEventQ);

    rxnotify.eType = RXQ_INTERRUPTED;
    rxnotify.pvData = NULL;
    rxnotify.Len = 0;
    Radio.Standby( );
    xQueueSend( rxEventQ, &rxnotify, 0 );   //CHECKME: overwrite?
    DEBUG_PRINT("READ INTERRUPTED");

    if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        DEBUG_PRINT("GOT CONTROL");
        Radio.Send( data, len );                //copies data to fifo, fixed timeout
        xStatus = xQueueReceive( txEventQ, &xReceivedStructure, portMAX_DELAY );    //no timeout
        DEBUG_PRINT("EVENT");
        
        if( xStatus == pdPASS ) {
            if( xReceivedStructure.eType == TXQ_DATA_SENT ) {
                ret = len;
                DEBUG_PRINT("SENT");
            }
            else if (xReceivedStructure.eType == TXQ_TIMEOUT ) {
                DEBUG_PRINT("SENT timeout");
            }
        
        }
        else {
            configASSERT(false);
        }
        xSemaphoreGive( RadioSem );       //give back radio control
    }
    else {
        configASSERT(false);
    }

    return ret;
}


#include "sx1276.h"
//~ //keep shadow copy of settings 
static RadioLoRaSettings_t radio_settings;

void radio_txconfig(RadioLoRaSettings_t *settings) {
    if((settings->Power    == radio_settings.Power)      &&
    (settings->Bandwidth   == radio_settings.Bandwidth)  &&
    (settings->Datarate    == radio_settings.Datarate)   &&
    (settings->Coderate    == radio_settings.Coderate)   &&
    (settings->PreambleLen == radio_settings.PreambleLen)&&
    (settings->FixLen      == radio_settings.FixLen)     &&
    (settings->CrcOn       == radio_settings.CrcOn)      &&
    (settings->FreqHopOn   == radio_settings.FreqHopOn)  &&
    (settings->TxTimeout   == radio_settings.TxTimeout))
        return;     //no change
    if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        //copy RX settings in shadow
        radio_settings.Power       = settings->Power;
        radio_settings.Bandwidth   = settings->Bandwidth;
        radio_settings.Datarate    = settings->Datarate;
        radio_settings.Coderate    = settings->Coderate;
        radio_settings.PreambleLen = settings->PreambleLen;
        radio_settings.FixLen      = settings->FixLen;
        radio_settings.CrcOn       = settings->CrcOn;
        radio_settings.FreqHopOn   = settings->FreqHopOn;
        radio_settings.TxTimeout   = settings->TxTimeout;    //TODO:TimeOnAir

        Radio.SetTxConfig( MODEM_LORA, settings->Power,
                                       0, 
                                       settings->Bandwidth,
                                       settings->Datarate, 
                                       settings->Coderate,
                                       settings->PreambleLen, 
                                       settings->FixLen,
                                       settings->CrcOn,
                                       settings->FreqHopOn,
                                       0, 
                                       LORA_IQ_INVERSION_ON, 
                                       settings->TxTimeout );
        xSemaphoreGive( RadioSem );       //give back radio control
    }
    else
        configASSERT(false);
}

void radio_rxconfig(RadioLoRaSettings_t *settings) {

    if((settings->Bandwidth== radio_settings.Bandwidth)  &&
    (settings->Datarate    == radio_settings.Datarate)   &&
    (settings->Coderate    == radio_settings.Coderate)   &&
    (settings->PreambleLen == radio_settings.PreambleLen)&&
    (settings->FixLen      == radio_settings.FixLen)     &&
    (settings->CrcOn       == radio_settings.CrcOn)      &&
    (settings->FreqHopOn   == radio_settings.FreqHopOn))
        return;     //no change

    if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        //copy RX settings
        radio_settings.Bandwidth   = settings->Bandwidth;
        radio_settings.Datarate    = settings->Datarate;
        radio_settings.Coderate    = settings->Coderate;
        radio_settings.PreambleLen = settings->PreambleLen;
        radio_settings.FixLen      = settings->FixLen;
        radio_settings.CrcOn       = settings->CrcOn;
        radio_settings.FreqHopOn   = settings->FreqHopOn;

        Radio.SetRxConfig( MODEM_LORA, 
                                        settings->Bandwidth, 
                                        settings->Datarate,
                                        settings->Coderate,
                                        0, 
                                        settings->PreambleLen,
                                        LORA_SYMBOL_TIMEOUT,
                                        settings->FixLen,
                                        0,      //payload len
                                        settings->CrcOn, 
                                        settings->FreqHopOn,
                                        0,      //HopPeriod
                                        LORA_IQ_INVERSION_ON, 
                                        true    //rxContinuous
                                        );
        xSemaphoreGive( RadioSem );       //give back radio control
    }
    else
        configASSERT(false);
                                    
}

void radio_status(char * buffer) {
    if( xSemaphoreTake( RadioSem, portMAX_DELAY ) == pdTRUE ) {
        Radio.GetStatus(buffer);
        xSemaphoreGive( RadioSem );       //give back radio control
    }
    else {
        configASSERT(false);
    }

}

void radio_init(void)
{
    DEBUG_PRINT("radio driver init");

    rxEventQ = xQueueCreate( 1, sizeof(EventQ_t) );
    txEventQ = xQueueCreate( 1, sizeof(EventQ_t) );

    RadioSem = xSemaphoreCreateBinary();    //take radio hardware

    configASSERT(RadioSem);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetModem(MODEM_LORA);
    Radio.SetChannel( RF_FREQUENCY );
    //~ bool     LowDatarateOptimize;
    //~ uint8_t  PayloadLen;
    //~ uint8_t  HopPeriod;
    //~ bool     RxContinuous;
    //~ radio_settings.Power        = TX_OUTPUT_POWER;
    //~ radio_settings.Bandwidth    = LORA_BANDWIDTH;
    //~ radio_settings.Datarate     = LORA_SPREADING_FACTOR;
    //~ radio_settings.Coderate     = LORA_CODINGRATE;
    //~ radio_settings.PreambleLen  = LORA_PREAMBLE_LENGTH;
    //~ radio_settings.FixLen       = LORA_FIX_LENGTH_PAYLOAD_ON;
    //~ radio_settings.CrcOn        = true;
    //~ radio_settings.FreqHopOn    = false;
    //~ radio_settings.TxTimeout    = 3000;
    //~ radio_settings.IqInverted   = true;


    //~ radio_txconfig(&radio_settings);    
    //~ radio_rxconfig(&radio_settings);    
    Radio.SetModem(MODEM_LORA);
    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    xSemaphoreGive(RadioSem);

    DEBUG_PRINT("radio driver initialized");
}

