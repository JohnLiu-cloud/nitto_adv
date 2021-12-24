#include "gpio.h"
#include "drv_twi.h"
#include "nrf_delay.h"
#include "custom_board.h"

static bool m_system_pwr = false;

static uint16_t led_count = 0;

/**
 * @brief gpio init.
 */
void gpio_init(void)
{
  nrf_gpio_cfg_output(NI_PWR_PIN);

  //nrf_gpio_cfg_output(NI_IR_PWR_PIN);

  nrf_gpio_cfg_output(NI_LED_PWR_PIN);

  nrf_gpio_cfg_output(NI_SHT_RES_PIN);

  nrf_gpio_cfg_output(NI_SC7_PWR_PIN);

  nrf_gpio_cfg_output(NI_SHT_PWR_PIN);

  nrf_gpio_cfg_output(TWI0_SCL_PIN);

  nrf_gpio_cfg_output(TWI0_SDA_PIN);
  
  NI_PWR_OFF();

  NI_LED_PWR_OFF();

  //NI_IR_PWR_OFF();

  NI_SC7_PWR_OFF();

  NI_SHT_PWR_OFF();

  NI_SHT_RES_H();
}

/**
 * @brief system power on.
 */
void system_power(void)
{
  if( m_system_pwr )
  {
    m_system_pwr = false;
    NI_PWR_OFF();
  }
  else 
  {
    m_system_pwr = true;
    NI_PWR_ON();
  }
}

bool is_system_power(void)
{
  return m_system_pwr;
}
/**
 * @brief led power on.
 */
void led_power_on(uint16_t count)
{
  led_count = count;
  NI_LED_PWR_ON();
}

void led_pwr_on_factory_test(void)
{
  NI_LED_PWR_ON();
}
/**
 * @brief led power off.
 */
void led_power_off(void)
{
  led_count = 0;
  NI_LED_PWR_OFF();
}

void led_pwr_off_factory_test(void)
{
  NI_LED_PWR_OFF();
}

/**
 * @brief led enter sleep mode.
 */
void led_enter_sleep(void)
{
  if( led_count ) 
  {
    led_count--;
    if( led_count == 0 )
    {
      led_power_off();
    }
  }
}

/**
 * @brief IR power on.
 */
void ir_power_on(void)
{
  //NI_IR_PWR_ON();
}

/**
 * @brief IR power off.
 */
void ir_power_off(void)
{
  //NI_IR_PWR_OFF();
}

/**
 * @brief sht power on.
 */
void sht_power_on(void)
{
  NI_SHT_PWR_ON();
}

/**
 * @brief sht power off.
 */
void sht_power_off(void)
{
  NI_SHT_PWR_OFF();
}

/**
 * @brief ir wake up.
 */
void ir_wakeup(void)
{
  nrf_gpio_pin_set(TWI0_SCL_PIN);

  nrf_gpio_pin_clear(TWI0_SDA_PIN); 

  nrf_delay_ms(50);

  nrf_gpio_pin_set(TWI0_SDA_PIN);
}