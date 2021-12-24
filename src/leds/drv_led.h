#ifndef DRV_LED_H_
#define DRV_LED_H_

typedef struct{
  uint8_t g;
  uint8_t r;
  uint8_t b;
}led_col_t;

/**
 * @brief led init.
 */
void led_init(void);

/**@brief led colour setting.
 *
 * @param[in] col rgb data.
 */
void led_colour_set(led_col_t *col);

/**
 *@brief green led.
 */
void led_colour_green(void);

/**
 *@brief red led.
 */
void led_colour_red(void);

/**
 *@brief orange led.
 */
void led_colour_orange(void);

/**
 *@brief blue led.
 */
void led_colour_blue(void);

/**
 *@brief led factory test.
 */
void led_factory_test(void);

void led_low_power(void);

#endif //DRV_LED_H_