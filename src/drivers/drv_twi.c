#include "drv_twi.h"
#include "nrf_delay.h"
#include "gpio.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"
#include "adv.h"
#include "nrf_log.h"


#define TWI_INSTANCE_ID1     1  
#define TWI_INSTANCE_ID0     0   
                                                 /* TWI instance ID. */
static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID1);            /* TWI instance. */
static const nrf_drv_twi_t m_twi0 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID0);

static volatile bool m_xfer0_done = false;                                            /* Indicates if operation on TWI has ended. */
static volatile bool m_xfer1_done = false;  

static void twi0_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
			//NRF_LOG_INFO("TWI events.");
            m_xfer0_done = true;
            break;
        default:
            break;
    }
}
/**
 * @brief TWI events handler.
 */
static void twi1_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
			//NRF_LOG_INFO("TWI events.");
            m_xfer1_done = true;
            break;
        default:
            break;
    }
}

/*brief initialization
*
* @details enabled twi0.
*
*/
void twi0_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi0_config = {
       .scl                = TWI0_SCL_PIN,
       .sda                = TWI0_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi0, &twi0_config, twi0_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi0);
}


int16_t is_twi0_done(void)
{
  int32_t count = 500;
  while (m_xfer0_done == false)
  {
    count--;
    if( count <=0 )
    {
      return 1;
    }
    nrf_delay_ms(1);
  }
  return 0;
}

int is_twi1_done(void)
{
  int32_t count = 500;
  while (m_xfer1_done == false)
  {
    count--;
    if( count <=0 )
    {
      return 1;
    }
    nrf_delay_ms(1);
  }
  return 0;
}
/*brief initialization
*
* @details enabled twi1.
*
*/
void twi1_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi1_config = {
       .scl                = TWI1_SCL_PIN,
       .sda                = TWI1_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi1, &twi1_config, twi1_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi1);
}

/*brief uninit
*
* @details disable twi0 and twi1.
*
*/
void twi0_uninit(void)
{
  nrf_drv_twi_uninit(&m_twi0);

  nrf_gpio_cfg_output(TWI0_SCL_PIN);

  nrf_gpio_cfg_output(TWI0_SDA_PIN); 
}

/**
function for write data to sensor.
*/
void twi_write_data(twi_t twi,uint8_t *p_str,uint8_t len)
{   
  ret_code_t err_code;
  int is_done;
    switch(twi)
    {
    case JES_TWI_SHT_ENABLED:
        m_xfer1_done = false;
        err_code = nrf_drv_twi_tx(&m_twi1,JES_SHT_ADDR,p_str,len,false);
        APP_ERROR_CHECK(err_code);
        //while (m_xfer1_done == false);
        is_done = is_twi1_done();
        g_normal = is_done<<STATE_SHT_NG;
    break;
    case JES_TWI_MLX_ENABLED:
        m_xfer0_done = false;
        err_code = nrf_drv_twi_tx(&m_twi0,JES_MLX_ADDR,p_str,len,false);
        APP_ERROR_CHECK(err_code);
        //while (m_xfer0_done == false);
        is_done = is_twi0_done();
        g_normal = is_done<<STATE_MLX_NG;
    break;
    case JES_TWI_SC7_ENABLED:
        m_xfer1_done = false;
        err_code = nrf_drv_twi_tx(&m_twi1,JES_SC7_ADDR,p_str,len,false);
        APP_ERROR_CHECK(err_code);
        //while (m_xfer1_done == false);
        is_done = is_twi1_done();
        g_normal = is_done<<STATE_SC7_NG;
    break;
    }
 
}

void twi_write_addr(twi_t twi,uint8_t regAddr,uint8_t len)
{  
    ret_code_t err_code;
    int is_done;
    switch(twi)
    {
    case JES_TWI_SHT_ENABLED:
        m_xfer1_done = false;
        err_code = nrf_drv_twi_tx(&m_twi1,JES_SHT_ADDR,&regAddr,len,false);
        APP_ERROR_CHECK(err_code);
        //while (m_xfer1_done == false);
        is_done = is_twi1_done();
        g_normal = is_done<<STATE_SHT_NG;
    break;
    case JES_TWI_MLX_ENABLED:
        m_xfer0_done = false;
        err_code = nrf_drv_twi_tx(&m_twi0,JES_MLX_ADDR,&regAddr,len,true);
        APP_ERROR_CHECK(err_code);
        //while (m_xfer0_done == false);
        is_done = is_twi0_done();
        g_normal = is_done<<STATE_MLX_NG;
    break;
    case JES_TWI_SC7_ENABLED:
        m_xfer1_done = false;
        err_code = nrf_drv_twi_tx(&m_twi1,JES_SC7_ADDR,&regAddr,len,false);
        APP_ERROR_CHECK(err_code);
        //while (m_xfer1_done == false);
        is_done = is_twi1_done();
        g_normal = is_done<<STATE_SC7_NG;
    break;
    } 
}

void twi_read_data(twi_t twi,uint8_t *p_str,uint8_t len)
{
    ret_code_t err_code;
    int is_done;
    switch(twi)
    {
    case JES_TWI_SHT_ENABLED:
        m_xfer1_done = false;
        err_code = nrf_drv_twi_rx(&m_twi1, JES_SHT_ADDR, p_str,len); 
        APP_ERROR_CHECK(err_code);
        //while (m_xfer1_done == false);
        is_done = is_twi1_done();
        g_normal = is_done<<STATE_SHT_NG;
    break;
    case JES_TWI_MLX_ENABLED:
        m_xfer0_done = false;
        err_code = nrf_drv_twi_rx(&m_twi0, JES_MLX_ADDR, p_str,len); 
        APP_ERROR_CHECK(err_code);
        //while (m_xfer0_done == false);
        is_done = is_twi0_done();
        g_normal = is_done<<STATE_MLX_NG;
    break;
    case JES_TWI_SC7_ENABLED:
        m_xfer1_done = false;
        err_code = nrf_drv_twi_rx(&m_twi1,JES_SC7_ADDR,p_str,len); 
        APP_ERROR_CHECK(err_code);
        //while (m_xfer1_done == false);
        is_done = is_twi1_done();
        g_normal = is_done<<STATE_SC7_NG;
    break;
    }    	
  
}