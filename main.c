
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
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

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_ppi.h"
#include "nrf_delay.h"
#include "drv_mlx.h"
#include "drv_sht.h"
#include "drv_twi.h"
#include "drv_led.h"
#include "drv_flash.h"
#include "drv_sc7.h"
#include "drv_adc.h"
#include "adv.h"
#include "gpio.h"
#include "dfu.h"

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ACC_ENABLED 0

#define STANDBY_ENABLED 1

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */



#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/

static bool m_fst_write = false;
static bool m_factory_test = false;

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

APP_TIMER_DEF(m_app_timer_id);
#define TIMER_INTERVAL  APP_TIMER_TICKS(1000)

#if ACC_ENABLED
APP_TIMER_DEF(m_acc_timer_id);
#define ACC_INTERVAL    APP_TIMER_TICKS(50)
#endif //ACC_ENABLED

static void exe_factory_test(void);

static void timer_timeout_handler(void * p_context)
{
  uint8_t i;
  float sumItem=0,sumTem=0,sumHum=0;
  float maxItem,maxTem,maxHum;
  float minItem,minTem,minHum;
  static uint16_t count = 0;
  static uint8_t times = 0;
  bool read = false;

  drv_adc_read();
  if( m_factory_test )
  {
    led_pwr_on_factory_test();
    led_factory_test();
  }

#if STANDBY_ENABLED
  count++;
  switch( sou_data.mode )
  {
    case 0:
    case 1:
    case 2:
      if( count >= 10 ) // 10 seconds
      {
        count = 0;
        read = true;
      }
    break;
    case 3:
    case 4:
    case 5:
      if( count >= 30 ) // 30 seconds
      {
        count = 0;
        read = true;
      }
    break;
    case 6:
    case 7:
    case 8:
      if( count >= 60 ) // 60 seconds
      {
        count = 0;
        read = true;
      }
    break;
    case 9:
    case 10:
    case 11:
      if( count >= 100 ) // 100 seconds
      {
        count = 0;
        read = true;
      }
    break;
    case 12:
    case 13:
    case 14:
      if( count >= 300 ) // 300 seconds
      {
        count = 0;
        read = true;
      }
    break;
    case 15:
    case 16:
    case 17:
      if( count >= 600 ) // 600 seconds
      {
        count = 0;
        read = true;
      }
    break;
  }
  
  if( read )
  {
    sht_power_on();

    //ir_power_on();

    ir_wakeup();

    twi0_init();

    nrf_delay_ms(200);

    drv_mlx_get_tem( &sen_data.Item[times] );

    nrf_delay_ms(10);

    drv_mlx_read_emissivity();

    nrf_delay_ms(20);

    drv_sht_get_TemHum( &sen_data.tem[times] , &sen_data.hum[times] );  

    drv_mlx_sleepMode();

    sht_power_off();

    twi0_uninit();

    times++;

    if( times >= TIMES ) 
    {
      if( sou_data.mode%3 == 0 )
      {
        for( i=0 ; i<TIMES ; i++ )
        {
          sumItem += sen_data.Item[i];
          sumTem  += sen_data.tem[i];
          sumHum  += sen_data.hum[i]; 
        }

        sou_data.Item = sumItem/TIMES;
        sou_data.tem = sumTem/TIMES;
        sou_data.hum = sumHum/TIMES;

      }
      else if( sou_data.mode%3 == 1 )
      {
          maxItem = sen_data.Item[0];
          maxTem  = sen_data.tem[0];
          maxHum  = sen_data.hum[0]; 
        for( i=1 ; i<TIMES ; i++ )
        {
           if( maxItem < sen_data.Item[i] )
           {
              maxItem = sen_data.Item[i];
           }
           if( maxTem < sen_data.tem[i] )
           {
              maxTem = sen_data.tem[i];
           }
           if( maxHum < sen_data.hum[i] )
           {
              maxHum = sen_data.hum[i];
           }
        } 
        sou_data.Item = maxItem;
        sou_data.tem = maxTem;
        sou_data.hum = maxHum;         
      }
      else if( sou_data.mode%3 == 2 )
      {
          minItem = sen_data.Item[0];
          minTem  = sen_data.tem[0];
          minHum  = sen_data.hum[0]; 
        for( i=1 ; i<TIMES ; i++ )
        {
           if( minItem > sen_data.Item[i] )
           {
              minItem = sen_data.Item[i];
           }
           if( minTem > sen_data.tem[i] )
           {
              minTem = sen_data.tem[i];
           }
           if( minHum > sen_data.hum[i] )
           {
              minHum = sen_data.hum[i];
           }
        } 
        sou_data.Item = minItem;
        sou_data.tem = minTem;
        sou_data.hum = minHum;         
      }

      times = 0;

      adv_data_group();

      if( m_conn_handle == BLE_CONN_HANDLE_INVALID )
      {         
        advertising_update();   
      } 
      else 
      {
        if( m_factory_test )
        {
          exe_factory_test();
        }
      }
    }
    read = false;

  }
#endif
  led_enter_sleep();
  
  led_low_power();
}

#if ACC_ENABLED
static void acc_timeout_handler(void * p_context)
{
    drv_sc7_read_acc();
}
#endif //ACC_ENABLED

static void application_timers_start(void) 
{
    ret_code_t err_code;
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#if ACC_ENABLED
    err_code = app_timer_start(m_acc_timer_id, ACC_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif //ACC_ENABLED
}

static void create_times(void)
{
    // Create timers.
    ret_code_t err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
#if ACC_ENABLED
    err_code = app_timer_create(&m_acc_timer_id, APP_TIMER_MODE_REPEATED, acc_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif //ACC_ENABLED
}
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);


}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void nus_data_response( ni_cmd_t cmd , uint8_t state , uint8_t *p_data , uint16_t len )
{
  uint32_t err_code;
  uint8_t data_array[247];
  uint16_t length;
  
  data_array[0] = 0xf0;
  data_array[1] = cmd;
  data_array[2] = state;
  data_array[3] = (uint8_t)(len>>8);
  data_array[4] = (uint8_t)(len);

  if( len != 0 && p_data != NULL )
  {
    memcpy( &data_array[5] , p_data , len );
  }
  length = 5+len;

  err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
  if ((err_code != NRF_ERROR_INVALID_STATE) &&
    (err_code != NRF_ERROR_RESOURCES) &&
    (err_code != NRF_ERROR_NOT_FOUND))
  {
    APP_ERROR_CHECK(err_code);
  }
}

static void exe_factory_test(void)
{
  uint8_t data_array[247];
  uint16_t length;
  length = 7;
  memcpy( data_array , (uint8_t*)&adv_data , length );
  nus_data_response( NI_CMD_FACTORY_TEST , 0 , data_array , length  );
}

static void exe_fstorage_write(void)
{
    if( m_fst_write )
    {
      m_fst_write = false;
      fstorage_write( sou_data.sn , sou_data.mode , g_company_id , g_adv_interval );
      nus_data_response( NI_CMD_FLASH , 0 , NULL , 0 );
    }
}
/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint8_t cmd;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        if( p_evt->params.rx_data.p_data[0] == 0xff ) //Command header
        {
          cmd = p_evt->params.rx_data.p_data[1];      //Command ID
          switch(cmd)
          {
            case NI_CMD_MODE:
              NRF_LOG_INFO("NI_CMD_MODE,data=%d",p_evt->params.rx_data.p_data[4]);
              sou_data.mode = p_evt->params.rx_data.p_data[4];
              nus_data_response( cmd , 0 , NULL , 0 );
            break;
            case NI_CMD_WSN:
              NRF_LOG_INFO("NI_CMD_WSN,data=%d",p_evt->params.rx_data.p_data[4]);
              sou_data.sn = p_evt->params.rx_data.p_data[4];
              nus_data_response( cmd , 0 , NULL , 0 );
            break;
            case NI_CMD_RSN:
              NRF_LOG_INFO("NI_CMD_RSN");
              nus_data_response( cmd , 0 , &sou_data.sn , 1 );
            break;
            case NI_CMD_FLASH:
              NRF_LOG_INFO("NI_CMD_FLASH");
              m_fst_write = true;
            break;
            case NI_CMD_FACTORY_TEST:
              NRF_LOG_INFO("NI_CMD_FACTORY_TEST");
              if( m_factory_test )
              {
                m_factory_test = false;
              }
              else
              {
                m_factory_test = true;
              }
              nus_data_response( cmd , 0 , NULL , 0 );
            break;
            case NI_CMD_MAC:
              NRF_LOG_INFO("NI_CMD_MAC");
              nus_data_response( cmd , 0 , sou_data.mac , DEV_MAC_LEN );
            break;
            case NI_CMD_ID:
              NRF_LOG_INFO("NI_CMD_ID");
              g_company_id = (uint16_t)(p_evt->params.rx_data.p_data[4]<<8) + (uint16_t)p_evt->params.rx_data.p_data[5];
              nus_data_response( cmd , 0 , NULL , 0 );
            break;
            case NI_CMD_ADV:
              NRF_LOG_INFO("NI_CMD_ADV");
              g_adv_interval = (uint16_t)(p_evt->params.rx_data.p_data[4]<<8) + (uint16_t)p_evt->params.rx_data.p_data[5];
              nus_data_response( cmd , 0 , NULL , 0 );
            break;
            default:
            break;
          }
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_factory_test = false;
            led_pwr_off_factory_test();
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    static bool push = false;
    NRF_LOG_INFO("button evt = %d",event);
    switch (event)
    {
        case BSP_EVENT_KEY_0:

            break;

        case BSP_EVENT_KEY_1:
            push = true;
            system_power();
            led_power_on(3);
            nrf_delay_ms(10);
            led_colour_blue();
            break;

        case BSP_EVENT_KEY_2:
              if( !push )
              {
                led_power_on(3);
                nrf_delay_ms(10);
                if( g_normal )
                {
                  led_colour_red();
                }
                else
                {
                  led_colour_green();
                }
              }
              push = false;
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_INFO("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_INFO(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */







/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init( BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}



#define NI_VER 1
/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;

    ret_code_t err_code;
    // Initialize.
    //uart_init();
    sou_data.ver = NI_VER;

    log_init();

    timers_init();

    create_times();

    buttons_leds_init(&erase_bonds);
   
    gpio_init();
    
    twi1_init();

    //ir_wakeup();
    //twi0_init();
    //nrf_delay_ms(200);
    //drv_mlx_sleepMode();

    led_init();
#if ACC_ENABLED
    drv_sc7_init();
#endif
    fstorage_init();

    fstorage_read( &sou_data.sn , &sou_data.mode , &g_company_id , &g_adv_interval );

    int8_t txpower;
    txpower = 0;
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, BLE_GAP_TX_POWER_ROLE_ADV, txpower);
    APP_ERROR_CHECK(err_code);

    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    dfu_services_init();
    advertising_init();
    conn_params_init();
    // Start execution.
    //printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("nitto ble started.");
    NRF_LOG_INFO("nitto id:0x%04x mode:%d sn:%d.",g_company_id,sou_data.mode,sou_data.sn);

    advertising_start();
    
    application_timers_start();

    // Enter main loop.
    for (;;)
    {
        exe_fstorage_write();

        idle_state_handle();
    }
}


/**
 * @}
 */
