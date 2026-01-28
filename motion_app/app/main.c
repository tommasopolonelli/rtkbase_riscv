/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Tommaso Polonelli
 * @brief   This file implement the motion detection procedure using the 
 * lis2dh12
 *
 ******************************************************************************
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
#include "Debug.h"

/* Support for MilkV Duo S*/
#include "wiringx.h"

#include "main.h"
#include "lis2dh12_reg.h"
#include "duos_pinmux.h"

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            5 //ms

/* Self test limits converted from 10bit right-aligned to 16bit left-aligned. */
#define    MIN_ST_LIMIT_LSb     17*64
#define    MAX_ST_LIMIT_LSb    360*64

/* SPI2 is routed to J3 B13-B15 */
#define SPI_PORT    0
#define LIS_INT_PIN (22)
#define LIS_CS_PIN (26)
#define LIS_SCLK_PIN (23)
#define LIS_MOSI_PIN (19)
#define LIS_MISO_PIN (21)

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float_t acceleration_st_mg[3];
static float_t acceleration_mg[3];
static uint8_t tx_buffer[1000];
static float_t test_val_mg[3];
static float_t max_st_limit_mg;
static float_t min_st_limit_mg;


/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void tx_com(uint8_t *tx_buffer, uint16_t len);
void platform_delay(uint32_t ms);
static void platform_init(void);
static int32_t lis2dh12_self_test(void);
static int DEV_Equipment_Testing(void);
static int DEV_GPIO_Init(void);
static void Handler(int signo);

/* Main Function  --------------------------------------------------------------*/

int main(int argc, char *argv[])
{

    /* 3) Setup Ctrl+C handler */
    signal(SIGINT, Handler);

    /* 4) Configure pinmux */

    if (DEV_Equipment_Testing() != EXIT_SUCCESS) {
        Debug("Library not supported\n");
        return EXIT_FAILURE;
    }

    if (lis2dh12_self_test() != EXIT_SUCCESS) {
        Debug("lis2dh12 init failed\n");
        return EXIT_FAILURE;
    }

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
    wiringXSPIDataRW(SPI_PORT, (uint8_t*) bufp, len);
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
void tx_com(uint8_t *tx_buffer, uint16_t len)
{

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
	usleep(ms*1000);
}

/* Private Function  --------------------------------------------------------------*/

static int32_t lis2dh12_self_test(void)
{
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

  if (reg.byte != LIS2DH12_ID) {
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
  do {
    lis2dh12_status_get(&dev_ctx, &reg.status_reg);
  } while (!reg.status_reg.zyxda);

  /* Read dummy data and discard it */
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  /* Read 5 sample and get the average vale for each axis */
  memset(acceleration_mg, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lis2dh12_status_get(&dev_ctx, &reg.status_reg);
    } while (!reg.status_reg.zyxda);

    /* Read data and accumulate the mg value */
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

    for (j = 0; j < 3; j++) {
      acceleration_mg[j] += lis2dh12_from_fs2_nm_to_mg(
                              data_raw_acceleration[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    acceleration_mg[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
  //lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do {
    lis2dh12_status_get(&dev_ctx, &reg.status_reg);
  } while (!reg.status_reg.zyxda);

  /* Read dummy data and discard it */
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  /* Read 5 sample and get the average vale for each axis */
  memset(acceleration_st_mg, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lis2dh12_status_get(&dev_ctx, &reg.status_reg);
    } while (!reg.status_reg.zyxda);

    /* Read data and accumulate the mg value */
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

    for (j = 0; j < 3; j++) {
      acceleration_st_mg[j] += lis2dh12_from_fs2_nm_to_mg(
                                 data_raw_acceleration[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    acceleration_st_mg[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val_mg[i] = fabsf((acceleration_st_mg[i] - acceleration_mg[i]));
  }

  min_st_limit_mg = lis2dh12_from_fs2_nm_to_mg(MIN_ST_LIMIT_LSb);
  max_st_limit_mg = lis2dh12_from_fs2_nm_to_mg(MAX_ST_LIMIT_LSb);

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( min_st_limit_mg < test_val_mg[i] ) &&
        ( test_val_mg[i] < max_st_limit_mg)) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - PASS\r\n",
              i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }

    else {
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Axis[%d]: lmt min %4.2f mg - lmt max %4.2f mg - val %4.2f mg - FAIL\r\n",
              i, min_st_limit_mg, max_st_limit_mg, test_val_mg[i]);
    }

    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }

  /* Disable Self Test */
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
  /* Disable sensor. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_POWER_DOWN);

  return EXIT_SUCCESS;

}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

    int fd_spi;

    if(wiringXSetup("milkv_duos", NULL) == -1) {
        wiringXGC();
        Debug("FAIL: wiringXSetup \n");
    }

    if(DEV_GPIO_Init()!=EXIT_SUCCESS){
        Debug("FAIL: GPIO INIT \n");
    }

	// SPI Config
	if ((fd_spi = wiringXSPISetup(SPI_PORT, 1800000)) <0) {
        Debug("SPI Setup failed: %d\n", fd_spi);
        wiringXGC();
        return;
    }

}

static int DEV_Equipment_Testing(void)
{
	FILE *fp;
	char issue_str[64];

	fp = fopen("/etc/issue", "r");
	if (fp == NULL) {
		Debug("Unable to open /etc/issue");
		return -1;
	}
	if (fread(issue_str, 1, sizeof(issue_str), fp) <= 0) {
		Debug("Unable to read from /etc/issue");
		return -1;
	}
	issue_str[sizeof(issue_str)-1] = '\0';
	fclose(fp);

	Debug("Current environment: ");

	char systems[][9] = {"Raspbian", "Debian", "NixOS"};
	int detected = 0;
	for(int i=0; i<3; i++) {
		if (strstr(issue_str, systems[i]) != NULL) {
			Debug("%s\n", systems[i]);
			detected = 1;
		}
	}
	if (!detected) {
		Debug("OS not recognized\n");
		Debug("Built for Debian, but unable to detect environment.\n");
		return -1;
	}

	return EXIT_SUCCESS;
}

static int DEV_GPIO_Init(void)
{

    duos_pinmux("B13", "SPI3_SDO");
    duos_pinmux("B14", "SPI3_SDO");
    duos_pinmux("B15", "SPI3_SCK");
    duos_pinmux("A28", "A28");
    duos_pinmux("A18", "A18");

	if(wiringXValidGPIO(LIS_INT_PIN) != 0) {
        Debug("Invalid GPIO %d\n", LIS_INT_PIN);
        return EXIT_FAILURE;
    }
	pinMode(LIS_INT_PIN, PINMODE_INPUT);

	if(wiringXValidGPIO(LIS_CS_PIN) != 0) {
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

}

static void Handler(int signo)
{
    //System Exit
    printf("\r\nHandler:exit\r\n");
    DEV_Module_Exit();

    exit(0);
}