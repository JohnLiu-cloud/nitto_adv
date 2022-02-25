#ifndef  DRV_MLX_H
#define  DRV_MLX_H

/*brief initialization
*
*/
void drv_mlx_init(void);

/*brief Get the temperature data of mlx90614.
*
*  @details drv_mlx is tem data.
*
*/
void drv_mlx_get_tem( float *tem);

/*brief mlx90614 enter sleep mode.
*
*/
void drv_mlx_sleepMode(void);

void drv_mlx_read_emissivity(void);
#endif //DRV_MLX_H