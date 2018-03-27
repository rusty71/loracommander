/*
 * Code copied from usb_start.h
 */
#ifndef RADIO_DRV_H
#define RADIO_DRV_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    RF_STANDBY = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
}RadioState_t;

typedef struct
{
    uint32_t Channel;
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    uint8_t  Size;       //packet sdize
    int8_t   SnrValue;
    int16_t  RssiValue;
    RadioState_t State;
}RadioLoRaSettings_t;

void radio_init(void);
uint16_t radio_read(uint8_t *data, uint16_t maxlen, int16_t *rssi, int8_t *snr, uint32_t timeout);
uint16_t radio_write(uint8_t *data, uint16_t len);  //tx timeout in modem settings
void radio_get_config(RadioLoRaSettings_t *config);
void radio_set_config(RadioLoRaSettings_t *config);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif //RADIO_DRV
