#ifndef ADV_H_
#define ADV_H_

#include "ble_advdata.h"
#include "ble_advertising.h"

#define APP_ADV_INTERVAL                5280                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEV_MAC_LEN 6

#define ADV_INFO_LEN 16

#define  TIMES 6


#define STATE_SHT_NG 0
#define STATE_MLX_NG 1
#define STATE_SC7_NG 2

typedef enum{
  NI_CMD_NULL = 0,
  NI_CMD_MODE,
  NI_CMD_WSN,
  NI_CMD_RSN,
  NI_CMD_FLASH,
  NI_CMD_FACTORY_TEST,
  NI_CMD_MAC,
  NI_CMD_ID
}ni_cmd_t;

typedef struct{
  float tem;      //temperature
  float hum;      //humidity
  float Item;     //IR temperature
  uint8_t bat;      //battery level
  int8_t  rssi;     //rssi level 
  uint8_t wdt;      //WDT flag
  uint8_t mcu;      //MCU Error flag
  uint8_t sensor;   //sensor error flag
  uint8_t data;     //data error flag
  uint8_t ver;      //firmware version
  uint8_t sn;       //sequence number
  uint8_t mac[DEV_MAC_LEN];   //mac address
  uint8_t mode;     //device mode
}sou_data_t;

typedef struct{
  uint8_t tem[2];
  uint8_t hum[2];
  uint8_t Item[2];
  uint8_t other;
  uint8_t ver;
  uint8_t sn;
  uint8_t mac[DEV_MAC_LEN];
  uint8_t mode;
}adv_data_t;

typedef struct{
  float tem[TIMES];      //temperature
  float hum[TIMES];      //humidity
  float Item[TIMES];     //IR temperature
}sen_data_t;

extern uint16_t g_normal;

extern uint16_t g_company_id;

extern sou_data_t sou_data;

extern adv_data_t adv_data;

extern sen_data_t sen_data;

extern uint8_t g_adv_info[ADV_INFO_LEN];

void adv_struct_fun( ble_advertising_t *p_adv );

/**@brief Function to advertising data group.
 */
void adv_data_group(void);

/**@brief Function manufacturer specific data .
 */
void set_manuf_data(ble_advdata_manuf_data_t *manuf);

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void);

/**@brief Function for starting advertising.
 */
void advertising_start(void);

/**@brief Function to update advertising .
 */
void advertising_update(void);
#endif //ADV_H_