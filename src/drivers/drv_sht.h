#ifndef DRV_SHT_H
#define DRV_SHT_H
#include "stdint.h"

/*brief Get the tem&hum data of sht31.
*
*  @details drv_sht is tem&hum data.
*
*/
void drv_sht_get_TemHum( float *tem , float *hum );
#endif //DRV_SHT_H