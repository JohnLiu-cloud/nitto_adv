#ifndef GPIO_H_
#define GPIO_H_
#include "nrf_gpio.h"

#define   NI_PWR_PIN            NRF_GPIO_PIN_MAP(0,29)
#define   NI_BUT_PIN            NRF_GPIO_PIN_MAP(1,13)
//#define   NI_IR_PWR_PIN        NRF_GPIO_PIN_MAP(1,15)
#define   NI_LED_PWR_PIN        NRF_GPIO_PIN_MAP(0,26)
#define   NI_SHT_RES_PIN        NRF_GPIO_PIN_MAP(0,6)
#define   NI_SC7_PWR_PIN        NRF_GPIO_PIN_MAP(0,2)
#define   NI_SHT_PWR_PIN        NRF_GPIO_PIN_MAP(1,15)//NRF_GPIO_PIN_MAP(0,24)

#define NI_PWR_ON()         nrf_gpio_pin_set(NI_PWR_PIN)
#define NI_PWR_OFF()        nrf_gpio_pin_clear(NI_PWR_PIN)

//#define NI_IR_PWR_ON()         nrf_gpio_pin_clear(NI_IR_PWR_PIN) 
//#define NI_IR_PWR_OFF()        nrf_gpio_pin_set(NI_IR_PWR_PIN)

#define NI_LED_PWR_ON()         nrf_gpio_pin_clear(NI_LED_PWR_PIN) 
#define NI_LED_PWR_OFF()        nrf_gpio_pin_set(NI_LED_PWR_PIN)

#define NI_SC7_PWR_ON()         nrf_gpio_pin_clear(NI_SC7_PWR_PIN) 
#define NI_SC7_PWR_OFF()        nrf_gpio_pin_set(NI_SC7_PWR_PIN)

#define NI_SHT_PWR_ON()         nrf_gpio_pin_clear(NI_SHT_PWR_PIN) 
#define NI_SHT_PWR_OFF()        nrf_gpio_pin_set(NI_SHT_PWR_PIN)

#define NI_SHT_RES_H()         nrf_gpio_pin_set(NI_SHT_RES_PIN) 
#define NI_SHT_RES_L()        nrf_gpio_pin_clear(NI_SHT_RES_PIN)

/**
 * @brief gpio init.
 */
void gpio_init(void);

/**
 * @brief system power on.
 */
void system_power(void);

/**
 * @brief led power on.
 */
void led_power_on(uint16_t count);

/**
 * @brief led power off.
 */
void led_power_off(void);

/**
 * @brief led enter sleep mode.
 */
void led_enter_sleep(void);

/**
 * @brief ir power on.
 */
void ir_power_on(void);

/**
 * @brief ir power off.
 */
void ir_power_off(void);

/**
 * @brief sht power on.
 */
void sht_power_on(void);

/**
 * @brief sht power off.
 */
void sht_power_off(void);

/**
 * @brief ir wake up.
 */
void ir_wakeup(void);

void led_pwr_on_factory_test(void);

void led_pwr_off_factory_test(void);

#endif //GPIO_H_