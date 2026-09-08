#ifndef __CAN_PROBE_H__
#define __CAN_PROBE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void CanProbe_Run(void);
uint8_t CanProbe_GetLastCommand(uint32_t *rx_count,
                                uint32_t *rx_id,
                                uint8_t *dlc,
                                uint8_t data[4]);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROBE_H__ */
