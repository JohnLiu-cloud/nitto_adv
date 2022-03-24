#include "nrf_gpio.h"
#include "drv_twi.h"
#include "drv_mlx.h"
#include "adv.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "math.h"
#include "stdlib.h"

static uint8_t PEC_Calculation(uint8_t *dat , uint8_t len);


void drv_mlx_get_tem( float *tem)
{

    uint8_t cmd;
    uint8_t buff[4];
    uint8_t arr[6];
    uint8_t PecReg;
    float mlx_temp;
    cmd = 0X07;
    twi_write_addr( JES_TWI_MLX_ENABLED , cmd , 1 );
    twi_read_data( JES_TWI_MLX_ENABLED , buff , 3 );

    arr[0] = (JES_MLX_ADDR<<1);        
    arr[1] = cmd;
    arr[2] = (JES_MLX_ADDR<<1)+1;         //Load array arr
    arr[3] = buff[0];  //DataL               
    arr[4] = buff[1];  //DataH                               
    PecReg=PEC_Calculation(arr,5);     //Calculate CRC

    mlx_temp = ( (buff[1]<<8) | buff[0] )*0.02-273.15;

    *tem = mlx_temp;

    if( PecReg != buff[2] )
    {
      g_normal = 1<<STATE_MLX_NG;
    }

    //NRF_LOG_INFO("[%02x][%02x]",buff[0],buff[1]);
      NRF_LOG_INFO("IR tem %d",(int16_t)(mlx_temp*100));

   // NRF_LOG_INFO("PEC=[%02x][%02x]",buff[2],PecReg);
}


static uint8_t PEC_Calculation(uint8_t *dat , uint8_t len)
{
  uint8_t i;
  uint8_t crc=0;
  while( len-- )
  {
    crc ^= *dat++;
    for( i=0 ; i<8 ; i++ )
    {
      if( crc&0x80 )
      {
        crc = (crc<<1)^0x07;
      }
      else
      {
        crc = (crc<<1);
      }
    }
  }
    return crc;
}
/*brief mlx90614 enter sleep mode.
*
*/
void drv_mlx_sleepMode(void)
{
  uint8_t cmd;
  uint8_t buff[3];
  uint8_t arr[6];
  uint8_t PecReg;
  cmd = 0xff;
  //PecReg = 0xe8;

  arr[0] = JES_MLX_ADDR<<1;        
  arr[1] = cmd;  
  PecReg = PEC_Calculation(arr,2);
  NRF_LOG_INFO("PEC=0x%02x",PecReg);

  buff[0] = cmd;
  buff[1] = PecReg;

  twi_write_data(JES_TWI_MLX_ENABLED,buff,2);
}

void drv_mlx_read_emissivity(void)
{
    uint8_t cmd;
    uint8_t buff[4];
    uint8_t arr[6];
    uint8_t PecReg;
    float mlx_temp;
    cmd = 0X04|0X20;
    twi_write_addr( JES_TWI_MLX_ENABLED , cmd , 1 );
    twi_read_data( JES_TWI_MLX_ENABLED , buff , 3 );

    arr[0] = (JES_MLX_ADDR<<1);        
    arr[1] = cmd;
    arr[2] = (JES_MLX_ADDR<<1)+1;         //Load array arr
    arr[3] = buff[0];  //DataL       
    arr[4] = buff[1];                                     
    PecReg=PEC_Calculation(arr,5);     //Calculate CRC



    //NRF_LOG_INFO("emissivity=0x%02x%02x.",arr[3],arr[4]);
}