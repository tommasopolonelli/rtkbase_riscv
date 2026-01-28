#ifndef MAIN_H
#define MAIN_H

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void tx_com(uint8_t *tx_buffer, uint16_t len);
void platform_delay(uint32_t ms);

#endif /* MAIN_H */
