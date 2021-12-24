/**
 * Copyright (c) 2019, Bitkey, Inc.
 * All rights reserved.
 */
/**@file
 *
 * @defgroup btk_twi Bitkey TWI Control 
 * @{
 * @ingroup  btk
 * @brief    Bitkey TWI Control .
 */
#ifndef DRV_TWI_H__
#define DRV_TWI_H__
#include "nrf_gpio.h"
#include "sdk_config.h"
#ifdef __cplusplus
extern "C" {
#endif

#define TWI0_SCL_PIN NRF_GPIO_PIN_MAP(0,10)//NRF_GPIO_PIN_MAP(0,22)
#define TWI0_SDA_PIN NRF_GPIO_PIN_MAP(0,9)//NRF_GPIO_PIN_MAP(0,20)

#define TWI1_SCL_PIN NRF_GPIO_PIN_MAP(1,9)//(0,22)
#define TWI1_SDA_PIN NRF_GPIO_PIN_MAP(0,8)//(0,20)

#define BTK_LED0_ADDR    (0x80>>1)
#define BTK_LED1_ADDR    (0x82>>1)
#define BTK_LED2_ADDR    (0x86>>1)
#define BTK_LED3_ADDR    (0x8E>>1)
#define BTK_RTC_ADDR      (0xa2>>1)

#define JES_SHT_ADDR      (0x45)
#define JES_MLX_ADDR      (0x5A)
#define JES_SC7_ADDR      (0x18) 

typedef enum{
BTK_TWI_RTC_ENABLED  = 0x01,
BTK_TWI_LED0_ENABLED = 0x03,
BTK_TWI_LED1_ENABLED = 0x04,
BTK_TWI_LED2_ENABLED = 0x05,
BTK_TWI_LED3_ENABLED = 0x06,
JES_TWI_SHT_ENABLED = 0x07,//sht31 enabled
JES_TWI_MLX_ENABLED = 0x08,//mlx90614 enabled
JES_TWI_SC7_ENABLED = 0x09,//SC7A20 enabled
}twi_t; 

/*brief initialization
*
* @details enabled twi0.
*
*/
void twi0_init(void);

/*brief initialization
*
* @details enabled twi1.
*
*/
void twi1_init (void);

/*brief uninit
*
* @details disable  twi0.
*
*/
void twi0_uninit(void);

/*brief twi write data
*
* @param[in] twi    reference twi_t.
* @param[in] p_str  Data to be write.
* @param[in] twi    Length of data to write.
*
*/
void twi_write_data(twi_t twi,uint8_t *p_str,uint8_t len);

/*brief twi write address.
*
* @param[in] twi      reference twi_t.
* @param[in] regAddr  register address to be write.
* @param[in] twi      Length of address to write.
*
*/
void twi_write_addr(twi_t twi,uint8_t regAddr,uint8_t len);

/*brief twi read data.
*
* @param[in]  twi      reference twi_t.
* @param[out] data     Data to be write.
* @param[in]  twi      Length of data to read.
*
*/
void twi_read_data(twi_t twi,uint8_t *p_str,uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // DRV_TWI_H__

/** @} */