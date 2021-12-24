#include "nrf_drv_gpiote.h"
#include "sdk_common.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrfx_spim.h"
#include "nrf_delay.h"
#include "drv_led.h"
#include "gpio.h"
#include "drv_adc.h"
#include "nrf_log.h"

#define SPI_LEN 255
#define LED_DAT_PIN     NRF_GPIO_PIN_MAP(0,31)
#define LED_CLK_PIN     NRF_GPIO_PIN_MAP(0,24)

static volatile bool xfer_done;

uint8_t       m_reset_buf[40];

uint8_t       m_led_tx_buf[SPI_LEN];           /**< TX buffer. */

static uint8_t m_length =SPI_LEN;        /**< Transfer length. */

#define SPI_INSTANCE  2 /**< SPI instance index. */

const nrf_drv_spi_t led_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    xfer_done = true;
}

/**
 * @brief led init.
 */
void led_init(void)
{
    memset(m_reset_buf,0,40);
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = 255;
    spi_config.miso_pin = 255;
    spi_config.mosi_pin = LED_DAT_PIN;
    spi_config.sck_pin  = LED_CLK_PIN; 
		//spi_config.frequency = NRF_DRV_SPI_FREQ_125K;
    APP_ERROR_CHECK(nrf_drv_spi_init(&led_spi, &spi_config, spi_event_handler, NULL));  

}

/**@brief Function parse the data into the format required by the IC.0 = (High level 245~345ns , Low level 545~645ns). 1 = (High level 545~645ns , Low level 245~345ns).
 *
 * @param[in] original      original data.
 * @param[in] len           original data len.
 * @param[out] analysis     analysis.
 *
 * @retval analysis data length.
 */
uint8_t led_analysis_data(uint8_t *original, uint8_t len, uint8_t *analysis)
{
  uint8_t temp;
  uint8_t length;
  uint32_t out;
  uint16_t i,j;
  analysis[0]=0;
  for( i=0 ; i<len ; i++ )
  {
     temp=original[i];
     out = 0;
     for( j=0 ; j<8 ; j++ )
     {
        out <<= 3;
        if( temp & 0x80 ) //110
        {
          out |= 0x06; 
        }
        else //100
        {
          out |= 0x04;
        }
        temp <<= 1;
     }
     analysis[3*i+1]=(uint8_t)(out>>16);
     analysis[3*i+2]=(uint8_t)(out>>8);
     analysis[3*i+3]=(uint8_t)out;
  }
  length = len*3+1;
  return  length;
}

/**@brief led colour setting.
 *
 * @param[in] col rgb data.
 */
void led_colour_set(led_col_t *col)
{
    m_length = led_analysis_data( (uint8_t*)col , 3 , m_led_tx_buf );
    xfer_done=false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&led_spi, m_led_tx_buf, m_length, 0, 0));
    while(xfer_done == false);

    m_length=40;
    xfer_done=false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&led_spi, m_reset_buf, m_length, 0, 0));
    while(xfer_done == false);  
}

/**
 *@brief green led.
 */
void led_colour_green(void)
{
    led_col_t col;
    col.r = 0;
    col.g = 100;
    col.b = 0;
    led_colour_set(&col);
}

/**
 *@brief red led.
 */
void led_colour_red(void)
{
    led_col_t col;
    col.r = 100;
    col.g = 0;
    col.b = 0;
    led_colour_set(&col);
}

/**
 *@brief orange led.
 */
void led_colour_orange(void)
{
    led_col_t col;
    col.r = 100;
    col.g = 60;
    col.b = 0;
    led_colour_set(&col);
}

/**
 *@brief blue led.
 */
void led_colour_blue(void)
{
    led_col_t col;
    col.r = 0;
    col.g = 0;
    col.b = 100;
    led_colour_set(&col);
}


void led_factory_test(void)
{
  static uint8_t mode = 0;
  if( mode >= 3 )
  {
    mode = 0;
  }
  switch( mode )
  {
    case 0:
      led_colour_green();
    break;
    case 1:
      led_colour_red();
    break;
    case 2:
      led_colour_blue();
    break;
  }
  mode++;
}

void led_low_power(void)
{
  static uint8_t count = 0;
   if( g_low_bat )
  {
    if( count == 0 )
    {
      count = 5;
      led_power_on(1);
      nrf_delay_ms(10);
      led_colour_red();
    }
    else
    {
      count--;
    }
  }
}