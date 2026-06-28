#ifndef HOST_STUB_VIRTUAL_SERIAL_PORT_H
#define HOST_STUB_VIRTUAL_SERIAL_PORT_H

#include <stdbool.h>
#include <stdint.h>

uint8_t vcp_is_connected(void);
uint8_t vcp_transmit(const uint8_t *data, uint16_t len);
bool vcp_rx_read_byte(uint8_t *byte);

#endif
