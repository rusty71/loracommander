/*
 * Code copied from usb_start.h
 */
#ifndef RADIO_DRV_H
#define RADIO_DRV_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
#include <stdint.h>

void radio_init(void);
uint16_t radio_read(uint8_t *data, uint16_t maxlen, int16_t *rssi, int8_t *snr, uint32_t timeout);
uint16_t radio_write(uint8_t *data, uint16_t len);  //tx timeout in modem settings


#ifdef __cplusplus
}
#endif // __cplusplus

#endif //RADIO_DRV
