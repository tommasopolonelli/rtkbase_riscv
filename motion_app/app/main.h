#ifndef MAIN_H
#define MAIN_H

#define SPI_PORT 0

typedef struct {
    bool status;            // 1 if correctly configured, 0 otherwise
    int x;                  // acceleration x-axis in g
    int y;                  // acceleration y-axis in g
    int z;                  // acceleration z-axis in g
    int temperature;        // temperature in degree Celsius
    bool movement;          // 1 if movement detected, 0 otherwise
} sensor_event_json_t;


/* TX MACROS */
#define TX_NO_ERROR           (0)
#define TX_ERROR_INIT         (-1)    
#define TX_ERROR_CALIB        (-2)

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void tx_com(uint8_t mov_flag, int err, float *acc_mg, float temp_degC);
void platform_delay(uint32_t ms);
int platform_INT_get(void);

#endif /* MAIN_H */
