
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"


#define BUTTONS_NUMBER 1

#define BUTTON_1       NRF_GPIO_PIN_MAP(1,13)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           true


#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
