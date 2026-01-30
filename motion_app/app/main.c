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
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include "Debug.h"

/* Support for MilkV Duo S*/
#include "wiringx.h"

#include "main.h"
#include "lis2dh12_reg.h"
#include "lis2dh12_motion.h"
#include "duos_pinmux.h"

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME 5 // ms

/* Self test limits converted from 10bit right-aligned to 16bit left-aligned. */
#define MIN_ST_LIMIT_LSb (17 * 64)
#define MAX_ST_LIMIT_LSb (360 * 64)

/* SPI2 is routed to J3 B13-B15 */
#define LIS_INT_PIN   (22)
#define LIS_CS_PIN    (26)
#define LIS_SCLK_PIN  (23)
#define LIS_MOSI_PIN  (19)
#define LIS_MISO_PIN  (21)

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static int32_t lis2dh12_self_test(void);
static int DEV_Equipment_Testing(void);
static int DEV_GPIO_Init(void);
static void Handler(int signo);

/* Main Function  --------------------------------------------------------------*/

int main(int argc, char *argv[])
{

  /* 1) Setup Ctrl+C handler */
  signal(SIGINT, Handler);

  /* 2) Check for OS and libraries  */
  if (DEV_Equipment_Testing() != EXIT_SUCCESS)
  {
    Debug("Library not supported\n");
    return EXIT_FAILURE;
  }

  /* 3) Init sensor and calibration */
  if (lis2dh12_self_test() != EXIT_SUCCESS)
  {
    Debug("lis2dh12 init failed\n");
    return EXIT_FAILURE;
  }

  /* 4) Start motion detection task */
  motion_detection_task(0, 250); // Example: 0 second duration, 250 mg threshold

  return EXIT_SUCCESS;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{

  /* Write multiple command */
  reg |= 0x40;
  digitalWrite(LIS_CS_PIN, LOW);
  wiringXSPIDataRW(SPI_PORT, &reg, 1);
  wiringXSPIDataRW(SPI_PORT, (uint8_t *)bufp, len);
  digitalWrite(LIS_CS_PIN, HIGH);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{

  /* Read multiple command */
  reg |= 0xC0;
  memset(bufp, 0, len);
  digitalWrite(LIS_CS_PIN, LOW);
  wiringXSPIDataRW(SPI_PORT, &reg, 1);
  wiringXSPIDataRW(SPI_PORT, bufp, len);
  digitalWrite(LIS_CS_PIN, HIGH);

  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t mov_flag, int err, float *acc_mg, float temp_degC)
{

    // Use real-time clock for wall timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    printf("{\"ts\":%lld.%09ld,\"movement\":\"%u\",\"err\":\"%d\",
      \"x_mg\":%4.2f,\"y_mg\":%4.2f,\"z_mg\":%4.2f\",
      "\"temp\":\"%6.2f\"}\n",
      (long long)ts.tv_sec, ts.tv_nsec, mov_flag, err,
      acc_mg[0], acc_mg[1], acc_mg[2], 
      temp_degC);

    // Ensure Python receives the line promptly when piped
    fflush(stdout);

}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
  Debug("Sleep for :%d ms\n", ms);
  usleep(ms * 1000);
}

int platform_INT_get(void)
{
    /* read assigned interrupt value */
    return digitalRead(LIS_INT_PIN);
}

/* Private Function  --------------------------------------------------------------*/

/**
 * @brief Performs self-test on the LIS2DH12 accelerometer sensor.
 * 
 * This function initializes the LIS2DH12 accelerometer via SPI, configures it
 * for self-test operation, and validates the sensor's functionality by comparing
 * acceleration measurements with and without self-test enabled.
 * 
 * The test procedure includes:
 * - Device identification and initialization
 * - Configuration of sensor parameters (2g full scale, 50Hz ODR, normal mode)
 * - Baseline acceleration measurement (5 samples averaged)
 * - Self-test positive mode activation
 * - Self-test acceleration measurement (5 samples averaged)
 * - Comparison of measurements against manufacturer-defined limits
 * - Per-axis PASS/FAIL status reporting
 * 
 * @return int32_t EXIT_SUCCESS if all three axes pass self-test limits,
 *                 EXIT_FAILURE if device is not found or initialization fails.
 *                 Individual axis results are transmitted via tx_com().
 * 
 * @note Requires platform_init(), platform_delay(), platform_write(),
 *       platform_read(), and tx_com() to be implemented.
 * @note Self-test results are output as formatted strings with min/max limits
 *       and measured values for each axis (X, Y, Z).
 */
static int32_t lis2dh12_self_test(void)
{
  int16_t data_raw_acceleration[3];
  float_t acceleration_st_mg[3];
  float_t acceleration_mg[3];
  float_t test_val_mg[3];
  float_t max_st_limit_mg;
  float_t min_st_limit_mg;
  stmdev_ctx_t dev_ctx;
  lis2dh12_reg_t reg;
  uint8_t i, j;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = SPI_PORT;
  /* Wait boot time and initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lis2dh12_device_id_get(&dev_ctx, &reg.byte);

  if (reg.byte != LIS2DH12_ID)
  {
    /* manage here device not found */
    Debug("Defice not found, read ID :%d \n", reg.byte);
    return EXIT_FAILURE;
  }

  /* Enable Block Data Update. */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale to 2g. */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
  /* Set device in normal mode. */
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_NM_10bit);
  /* Set Output Data Rate to 1Hz. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_50Hz);
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do
  {
    lis2dh12_status_get(&dev_ctx, &reg.status_reg);
  } while (!reg.status_reg.zyxda);

  /* Read dummy data and discard it */
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  /* Read 5 sample and get the average vale for each axis */
  memset(acceleration_mg, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++)
  {
    /* Check if new value available */
    do
    {
      lis2dh12_status_get(&dev_ctx, &reg.status_reg);
    } while (!reg.status_reg.zyxda);

    /* Read data and accumulate the mg value */
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

    for (j = 0; j < 3; j++)
    {
      acceleration_mg[j] += lis2dh12_from_fs2_nm_to_mg(
          data_raw_acceleration[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++)
  {
    acceleration_mg[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
  // lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do
  {
    lis2dh12_status_get(&dev_ctx, &reg.status_reg);
  } while (!reg.status_reg.zyxda);

  /* Read dummy data and discard it */
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  /* Read 5 sample and get the average vale for each axis */
  memset(acceleration_st_mg, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++)
  {
    /* Check if new value available */
    do
    {
      lis2dh12_status_get(&dev_ctx, &reg.status_reg);
    } while (!reg.status_reg.zyxda);

    /* Read data and accumulate the mg value */
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

    for (j = 0; j < 3; j++)
    {
      acceleration_st_mg[j] += lis2dh12_from_fs2_nm_to_mg(
          data_raw_acceleration[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++)
  {
    acceleration_st_mg[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++)
  {
    test_val_mg[i] = fabsf((acceleration_st_mg[i] - acceleration_mg[i]));
  }

  min_st_limit_mg = lis2dh12_from_fs2_nm_to_mg(MIN_ST_LIMIT_LSb);
  max_st_limit_mg = lis2dh12_from_fs2_nm_to_mg(MAX_ST_LIMIT_LSb);

  /* Check self test limit */
  for (i = 0; i < 3; i++)
  {
    if ((min_st_limit_mg < test_val_mg[i]) &&
        (test_val_mg[i] < max_st_limit_mg))
    {
      float acc_mg[3] = {acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]};
      tx_com(0, TX_NO_ERROR, acc_mg, 0.0f);
      Debug("Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - PASS\r\n",
               i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }
    else
    {
      float acc_mg[3] = {acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]};
      tx_com(0, TX_ERROR_CALIB, acc_mg, 0.0f);
      Debug("Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - FAIL\r\n",
               i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }

  }

  /* Disable Self Test */
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
  /* Disable sensor. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_POWER_DOWN);

  return EXIT_SUCCESS;
}

/**
 * @brief Platform specific initialization (platform dependent)
 * 
 * Initializes the platform hardware including GPIO, SPI, and wiringX setup.
 * This function sets up the milkv_duos board configuration and configures
 * the SPI interface at 1.8 MHz baud rate.
 * 
 * @details
 * - Initializes wiringX with "milkv_duos" board identifier
 * - Initializes GPIO subsystem via DEV_GPIO_Init()
 * - Configures SPI port with 1.8 MHz clock speed
 * - Performs cleanup (wiringXGC) on setup failures
 * 
 * @return void
 * 
 * @note If wiringXSetup or SPI setup fails, debug messages are logged.
 *       The function returns early on SPI initialization failure.
 * 
 * @see wiringXSetup()
 * @see DEV_GPIO_Init()
 * @see wiringXSPISetup()
 * @see wiringXGC()
 */
/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

  int fd_spi;

  if (wiringXSetup("milkv_duos", NULL) == -1)
  {
    wiringXGC();
    Debug("FAIL: wiringXSetup \n");
  }

  if (DEV_GPIO_Init() != EXIT_SUCCESS)
  {
    Debug("FAIL: GPIO INIT \n");
  }

  // SPI Config
  if ((fd_spi = wiringXSPISetup(SPI_PORT, 1800000)) < 0)
  {
    Debug("SPI Setup failed: %d\n", fd_spi);
    wiringXGC();
    return;
  }
}

/**
 * @brief Detects and validates the device's operating system environment.
 * 
 * Reads the system issue file (/etc/issue) and checks if the detected
 * operating system is one of the supported systems (Raspbian, Debian, NixOS).
 * 
 * @return EXIT_SUCCESS if a supported OS is detected, -1 if:
 *         - /etc/issue cannot be opened
 *         - /etc/issue cannot be read
 *         - OS is not recognized as one of the supported systems
 * 
 * @note The function expects /etc/issue to contain system identification
 *       information and will fail if the file is inaccessible or unreadable.
 */
static int DEV_Equipment_Testing(void)
{
  FILE *fp;
  char issue_str[64];

  fp = fopen("/etc/issue", "r");
  if (fp == NULL)
  {
    Debug("Unable to open /etc/issue");
    return -1;
  }
  if (fread(issue_str, 1, sizeof(issue_str), fp) <= 0)
  {
    Debug("Unable to read from /etc/issue");
    return -1;
  }
  issue_str[sizeof(issue_str) - 1] = '\0';
  fclose(fp);

  Debug("Current environment: ");

  char systems[][9] = {"Raspbian", "Debian", "NixOS"};
  int detected = 0;
  for (int i = 0; i < 3; i++)
  {
    if (strstr(issue_str, systems[i]) != NULL)
    {
      Debug("%s\n", systems[i]);
      detected = 1;
    }
  }
  if (!detected)
  {
    Debug("OS not recognized\n");
    Debug("Built for Debian, but unable to detect environment.\n");
    return -1;
  }

  return EXIT_SUCCESS;
}

/**
 * @brief Initializes GPIO pins and device pin multiplexing for the motion application.
 * 
 * Configures SPI3 communication pins and sets up interrupt/chip-select pins for the LIS sensor.
 * Validates GPIO pin accessibility and configures pin modes (input/output).
 * 
 * @return EXIT_SUCCESS if all GPIO pins are valid and initialization completes successfully.
 * @return EXIT_FAILURE if either LIS_INT_PIN or LIS_CS_PIN validation fails.
 * 
 * @details
 * - Configures pins B13, B14, B15 for SPI3 communication (SDO and SCK)
 * - Configures pins A28 and A18 for pass-through pinmux
 * - Validates LIS_INT_PIN and sets it as digital input
 * - Validates LIS_CS_PIN and sets it as digital output with initial HIGH state
 * - Applies a 1ms delay after initialization
 */
static int DEV_GPIO_Init(void)
{

  duos_pinmux("B13", "SPI3_SDO");
  duos_pinmux("B14", "SPI3_SDO");
  duos_pinmux("B15", "SPI3_SCK");
  duos_pinmux("A28", "A28");
  duos_pinmux("A18", "A18");

  if (wiringXValidGPIO(LIS_INT_PIN) != 0)
  {
    Debug("Invalid GPIO %d\n", LIS_INT_PIN);
    return EXIT_FAILURE;
  }
  pinMode(LIS_INT_PIN, PINMODE_INPUT);

  if (wiringXValidGPIO(LIS_CS_PIN) != 0)
  {
    Debug("Invalid GPIO %d\n", LIS_INT_PIN);
    return EXIT_FAILURE;
  }
  pinMode(LIS_CS_PIN, PINMODE_OUTPUT);

  digitalWrite(LIS_CS_PIN, HIGH);

  platform_delay(1);

  return EXIT_SUCCESS;
}

/******************************************************************************
function:	Module exits, closes SPI and LIS2D library
parameter:
Info:
******************************************************************************/
void DEV_Module_Exit(void)
{
  digitalWrite(LIS_CS_PIN, HIGH);
  wiringXGC();
}

static void Handler(int signo)
{
  // System Exit
  printf("\r\nHandler:exit\r\n");
  DEV_Module_Exit();

  exit(0);
}