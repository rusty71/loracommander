#ifndef CDC_DRV_H
#define CDC_DRV_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>

/**
 * \berif Initialize USB
 */
void cdc_init(void);
void cdc_wait_DTR(void);
void cdc_wait_CDC(void);
uint16_t cdc_write(uint8_t *buf, uint16_t len);
uint16_t cdc_read(uint8_t *buf, uint16_t maxlen, uint32_t timeout);


#ifdef __cplusplus
}
#endif // __cplusplus

#endif //CDC_DRV
