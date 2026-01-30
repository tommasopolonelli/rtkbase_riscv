/**
 * @file    main.c
 * @author  Tommaso Polonelli
 * @date    
 * @version 1.0
 * 
 * @brief   Motion detection implementation using LIS2DH12 accelerometer
 * 
 * @details This module provides functionality for motion detection procedures
 *          utilizing the LIS2DH12 3-axis accelerometer. It includes self-test
 *          capabilities and motion detection algorithms.
 * 
 * @note    This implementation is part of the rtkbase RISC-V motion application
 * 
 * @copyright (C) 2024. All rights reserved.
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "lis2dh12_reg.h"
#include "main.h"
#include "Debug.h"


/* Private macro -------------------------------------------------------------*/

#define DURATION_LSB (100)   //in ms dependent on ODR
#define THRESHOLD_LSB (15)   //15 mg accuracy

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Main Example --------------------------------------------------------------*/
void motion_detection_task(uint32_t event_duration_ms, uint32_t event_threshold_mg)
{

  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = SPI_PORT;

  int16_t data_raw_acceleration[3];
  int16_t data_raw_temperature;
  float_t acceleration_mg[3];
  float_t temperature_degC;

  if ((event_duration_ms/DURATION_LSB) > 127){
    /* avoid register sturation */
    event_duration_ms = 127 * DURATION_LSB;
  }
  if ((event_threshold_mg/THRESHOLD_LSB) > 127){
    /* avoid register sturation */
    event_threshold_mg = 127 * THRESHOLD_LSB;
  }

  /* Enable Block Data Update. */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate to 1Hz. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_10Hz);
  /* Set full scale to 2g. */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
  /* Enable temperature sensor. */
  lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);
  /* Set device in low power mode with 8 bit resol. */
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);
  /* Set device HP filter to 0.02 Hz cut-off */
  lis2dh12_high_pass_bandwidth_set(&dev_ctx, LIS2DH12_AGGRESSIVE);
  lis2dh12_high_pass_mode_set(&dev_ctx, LIS2DH12_NORMAL);
  /* Connect HP to data path on interrupts */
  lis2dh12_high_pass_int_conf_set(&dev_ctx, LIS2DH12_ON_INT1_GEN);
  /* Route selected Interrupts to INT1 pin */
  lis2dh12_ctrl_reg3_t int_settings;
  int_settings.i1_ia1 = 1; // Interrupt activity 1 driven to INT1 pin
  lis2dh12_pin_int1_config_set(&dev_ctx, &int_settings);
  /* Interrupt 1 pin latched */
  lis2dh12_int1_pin_notification_mode_set(&dev_ctx, LIS2DH12_INT1_LATCHED);
  /* Event acceleration threshold in mg, with a precision of 15 mg per LSB */
  lis2dh12_int1_gen_threshold_set(&dev_ctx, (uint8_t)(event_threshold_mg/THRESHOLD_LSB));
  /* Event duration in multiples of ODR - our case is multiple of 100 ms */
  lis2dh12_int1_gen_duration_set(&dev_ctx, (uint8_t)(event_duration_ms/DURATION_LSB));

  /* Dummy read to force the HP filter to the current acceleration value */
  /* This read may be performed anytime it is required to set the orientation/tilt of the device as a reference state
     without waiting for the filter to settle. */
  uint8_t reference;
  lis2dh12_filter_reference_get(&dev_ctx, &reference);

  int platform_INT_get(void)

  /* Read samples in polling mode (no int) */
  while (1) {
    lis2dh12_reg_t reg;

    /* Poll for interrupt INT1 */
    if (platform_INT_get()) {
      Debug("----- INT1 ----- \r\n");
      tx_com(1, TX_NO_ERROR, acceleration_mg, temperature_degC);
    }

    /* Read output only if new value available */
    lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);

    if (reg.byte) {
      /* Read accelerometer data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = lis2dh12_from_fs2_lp_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] = lis2dh12_from_fs2_lp_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] = lis2dh12_from_fs2_lp_to_mg(data_raw_acceleration[2]);
      Debug("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(0, TX_NO_ERROR, acceleration_mg, temperature_degC);
    }

    lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);

    if (reg.byte) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lis2dh12_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lis2dh12_from_lsb_lp_to_celsius(data_raw_temperature);
      Debug("Temperature [degC]:%6.2f\r\n", temperature_degC);
      tx_com(0, TX_NO_ERROR, acceleration_mg, temperature_degC);
    }

  }
}

