#include "drv_twi.h"
#include "drv_sc7.h"
#include "gpio.h"
#include "nrf_delay.h"
#include "adv.h"
#include "nrf_log.h"

#define SC7_CTRL_REG1   0x20
#define SC7_CTRL_REG4   0x23
#define SC7_STATUS_REG  0x27
#define SC7_DATA_REG    0x28

void drv_sc7_init(void)
{
  uint8_t data[2];

  NI_SC7_PWR_ON();
  nrf_delay_ms(500);

  data[0] = SC7_CTRL_REG1;
  data[1] = 0x47;
  twi_write_data( JES_TWI_SC7_ENABLED , data , 2 );

  data[0] = SC7_CTRL_REG4;
  data[1] = 0x88;
  twi_write_data( JES_TWI_SC7_ENABLED , data , 2 );

}

void drv_sc7_read_acc(void)
{
  uint8_t status;
  uint8_t addr;
  uint8_t data[6];
  int16_t x,y,z;
  
  addr = SC7_STATUS_REG;
  twi_write_addr( JES_TWI_SC7_ENABLED , addr , 1 );
  twi_read_data( JES_TWI_SC7_ENABLED , &status , 1 );

    status = status & 0x0f;
    if (0x0f == status)
    {
      addr = SC7_DATA_REG+0x80;
      twi_write_addr( JES_TWI_SC7_ENABLED , addr , 1 );
      twi_read_data( JES_TWI_SC7_ENABLED , data , 6 );
      x = data[0] + (data[1]<<8);
      y = data[2] + (data[3]<<8);
      z = data[4] + (data[5]<<8);
      NRF_LOG_INFO("acc:x[%d] y[%d] z[%d]",x,y,z);
    }
}