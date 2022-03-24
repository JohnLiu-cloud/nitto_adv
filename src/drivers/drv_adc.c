#include "nrf_drv_saadc.h"
#include "adv.h"
#include "drv_adc.h"
#include "nrf_log.h"

#define SAMPLES_IN_BUFFER 1

static bool drv_adc_start = false;

bool g_low_bat = false;

static nrf_saadc_value_t     m_buffer_pool[SAMPLES_IN_BUFFER];

static int m_batt_adc;

static void saadc_stop(void)
{
  nrf_drv_saadc_uninit();
  NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
  NVIC_ClearPendingIRQ(SAADC_IRQn);
}

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    static uint8_t c=0;
    uint8_t i;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        m_batt_adc = p_event->data.done.p_buffer[0];

        drv_adc_average();

        //NRF_LOG_INFO("adc [%d]",m_batt_adc);
        if(nrfx_saadc_is_busy()){
//           NRF_LOG_INFO("adc abort!");
//           nrf_drv_saadc_abort();
           saadc_stop();
           drv_adc_start = false;
        }
    }
}

static void saadc_init(void)
{
    ret_code_t err_code;
    if((!nrfx_saadc_is_busy())&&(drv_adc_start==false)){

      //NRF_LOG_INFO("ADC Init");

      drv_adc_start = true;

      nrf_saadc_channel_config_t channel_config0 =
      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

      err_code = nrf_drv_saadc_init(NULL, saadc_callback);
      // APP_ERROR_CHECK(err_code);

      err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
      //    APP_ERROR_CHECK(err_code);
      err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool, SAMPLES_IN_BUFFER);
    }
//    APP_ERROR_CHECK(err_code);
}


ret_code_t drv_adc_read(void)
{
  ret_code_t err_code;
  saadc_init();
  if(nrfx_saadc_is_busy()){
//      NRF_LOG_INFO("ADC Read");
      //Event handler is called immediately after conversion is finished.
      err_code = nrf_drv_saadc_sample(); // Check error
      APP_ERROR_CHECK(err_code);
  }
  return NRF_SUCCESS;
}

void drv_adc_average(void)
{
  static uint16_t accumulative_total = 0;
  static int sum = 0;
  int ave;
  if( accumulative_total >= 60 ) {
    accumulative_total = 0;
    ave = sum/60;
    sum = 0;
    NRF_LOG_INFO("drv_adc_average [%d]",ave);
    if( ave > BATT_LEVEL_80 ) {
      sou_data.bat = 3 << 6; 
    } else if( ave > BATT_LEVEL_50 ) {
      sou_data.bat = 2 << 6; 
    } else if( ave > BATT_LEVEL_20 ) {
      sou_data.bat = 1 << 6; 
    } else if( ave > BATT_LEVEL_10 ) {
      sou_data.bat = 0; 
    } else {
      g_low_bat = true;
      sou_data.bat = 0;
      NRF_LOG_INFO("drv_low_pwr");
    }

  } else {
    sum += m_batt_adc;
    accumulative_total++;
  }
}