#ifndef DRV_ADC_H
#define DRV_ADC_H

extern bool g_low_bat;

#define BATT_LEVEL_80 398 // v=V/2 move 2.80v static 2.87v 1120ma/h 
#define BATT_LEVEL_50 385 // v=V/2 move 2.78v static 2.84v 700ma/h
#define BATT_LEVEL_20 369 // v=V/2 move 2.65v static 2.70v 280ma/h
#define BATT_LEVEL_10 361 // v=V/2 move 2.51v static 2.65v 140ma/h


ret_code_t drv_adc_read(void);

void drv_adc_average(void);

#endif //DRV_ADC_H