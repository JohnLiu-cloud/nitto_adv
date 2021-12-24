#include "drv_twi.h"
#include "drv_sht.h"
#include "adv.h"
#include "nrf_log.h"

#define SHT_CMD_STATE 0X2C06


void drv_sht_get_TemHum( float *p_tem , float *p_hum )
{
  uint8_t cmd[2];
  uint8_t buff[3];
  uint16_t tem,hum;
  float Temperature=0;
  float Humidity=0;

  cmd[0] = (SHT_CMD_STATE>>8);
  
  cmd[1] = (SHT_CMD_STATE&0xff);

  twi_write_data( JES_TWI_SHT_ENABLED , cmd , 2 );

  twi_read_data( JES_TWI_SHT_ENABLED , buff , 6 );

  //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "drv sht [%x %x %x]", buff[0] , buff[1] , buff[2]);

  //__LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,"drv sht",buff,6);

  tem = ((buff[0]<<8) | buff[1]);

  hum = ((buff[3]<<8) | buff[4]);

  Temperature= (175.0*(float)tem/65535.0-45.0);

  Humidity= (100.0*(float)hum/65535.0);

  *p_tem = Temperature;

  *p_hum = Humidity;

  NRF_LOG_INFO("drv sht tem %d.%d",(uint8_t)(Temperature),((uint16_t)(Temperature*100))%100);

  NRF_LOG_INFO("drv sht hum %d.%d",(uint8_t)(Humidity),((uint16_t)(Humidity*100))%100);
}
