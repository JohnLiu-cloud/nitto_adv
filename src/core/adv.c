#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_delay.h"
#include "adv.h"

BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

uint16_t g_company_id = 0xffff;

sou_data_t sou_data;

adv_data_t adv_data;

sen_data_t sen_data;

uint8_t g_adv_info[ADV_INFO_LEN];

uint16_t g_normal = 0;

void adv_struct_fun( ble_advertising_t *p_adv )
{
  p_adv = &m_advertising;
}

/**@brief Function to advertising data group.
 */
void adv_data_group(void)
{
  uint8_t adv[31];

  //temperature value
  adv_data.tem[0] = (uint8_t)(((uint16_t)(sou_data.tem*100))>>8);
  adv_data.tem[1] = (uint8_t)((uint16_t)(sou_data.tem*100));

  //humidity value
  adv_data.hum[0] = (uint8_t)(((uint16_t)(sou_data.hum*100))>>8);
  adv_data.hum[1] = (uint8_t)((uint16_t)(sou_data.hum*100));

  //IR temperature value
  adv_data.Item[0] = (uint8_t)(((uint16_t)(sou_data.Item*100))>>8);
  adv_data.Item[1] = (uint8_t)((uint16_t)(sou_data.Item*100));

  //battery level
  adv_data.other = sou_data.bat;
  //rssi level
  adv_data.other |= 3 << 4; 

  //WDT flag
  adv_data.other |= 1 << 3;

  //MCU Error flag
  adv_data.other |= 1 << 2;

  //Sensor Error flag
  adv_data.other |= 1 << 1;

  //Data Error flag
  adv_data.other |= 1 << 0;

  //firmware version
  adv_data.ver = sou_data.ver;

  //sequence number
  adv_data.sn = sou_data.sn;

  //MAC address
  memcpy( adv_data.mac , sou_data.mac , DEV_MAC_LEN );

  //Device mode
  adv_data.mode = sou_data.mode;

  NRF_LOG_INFO("tem[%d.%d]hum[%d.%d]Item[%d.%d]", \
                (uint8_t)(sou_data.tem),((uint16_t)(sou_data.tem*100))%100 , \
                (uint8_t)(sou_data.hum),((uint16_t)(sou_data.hum*100))%100 , \
                (uint8_t)(sou_data.Item),((uint16_t)(sou_data.Item*100))%100 );
}

/**@brief Function manufacturer specific data .
 */
void set_manuf_data(ble_advdata_manuf_data_t *manuf)
{
    memcpy( g_adv_info , (uint8_t*)&adv_data , sizeof(adv_data_t) );
    manuf->company_identifier = g_company_id;
    manuf->data.p_data = (uint8_t *) g_adv_info;
    manuf->data.size   = ADV_INFO_LEN;
}

/**@brief Function to update advertising .
 */
void advertising_update(void)
{
    ret_code_t err_code;
    ble_advdata_t new_data;
    ble_advdata_manuf_data_t manuf_specific_data = {};

    memset( &new_data , 0 , sizeof(ble_advdata_t) );

    set_manuf_data(&manuf_specific_data);

    new_data.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advertising_advdata_update(&m_advertising, &new_data, NULL );
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
    ble_advdata_manuf_data_t manuf_specific_data = {};

    sou_data.mac[0] = ((NRF_FICR->DEVICEADDR[0]>> 0) & 0xff);
    sou_data.mac[1] = ((NRF_FICR->DEVICEADDR[0]>> 8) & 0xff);
    sou_data.mac[2] = ((NRF_FICR->DEVICEADDR[0]>>16) & 0xff);
    sou_data.mac[3] = ((NRF_FICR->DEVICEADDR[0]>>24) & 0xff);
    sou_data.mac[4] = ((NRF_FICR->DEVICEADDR[1]>> 0) & 0xff);
    sou_data.mac[5] = ((NRF_FICR->DEVICEADDR[1]>> 8) & 0xff);

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_NO_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    set_manuf_data(&manuf_specific_data);
    init.advdata.p_manuf_specific_data = &manuf_specific_data;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = 0;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}