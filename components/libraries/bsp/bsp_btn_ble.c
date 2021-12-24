
#include "bsp_btn_ble.h"
#include "nrf_sdh_ble.h"
#include "bsp.h"

#define BTN_ID_PWR             0  /**< ID of button used to power on and power off the application. */
#define BTN_ID_STA             0 /**< ID of button used to query status the application. */


#define BTN_ACTION_SLEEP          BSP_BUTTON_ACTION_RELEASE    
#define BTN_ACTION_DISCONNECT     BSP_BUTTON_ACTION_LONG_PUSH  /**< Button action used to gracefully terminate a connection on long press. */
#define BTN_ACTION_WHITELIST_OFF  BSP_BUTTON_ACTION_LONG_PUSH  /**< Button action used to turn off usage of the whitelist. */

/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_INVALID_PARAM.
 */
#define RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_INVALID_PARAM)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_NOT_SUPPORTED.
 */
#define RETURN_ON_ERROR_NOT_NOT_SUPPORTED(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_NOT_SUPPORTED)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/**@brief This macro will call the registered error handler if err_code
 *        is not NRF_SUCCESS and the error handler is not NULL.
 */
#define CALL_HANDLER_ON_ERROR(err_code)                           \
do                                                                \
{                                                                 \
    if (((err_code) != NRF_SUCCESS) && (m_error_handler != NULL)) \
    {                                                             \
        m_error_handler(err_code);                                \
    }                                                             \
}                                                                 \
while (0)


static bsp_btn_ble_error_handler_t m_error_handler = NULL; /**< Error handler registered by the user. */
static uint32_t                    m_num_connections = 0;  /**< Number of connections the device is currently in. */


/**@brief Function for configuring the buttons for connection.
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t connection_buttons_configure()
{
    uint32_t err_code;

    err_code = bsp_event_to_button_action_assign(BTN_ID_PWR,
                                                 BSP_BUTTON_ACTION_LONG_PUSH,
                                                 BSP_EVENT_KEY_1);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_STA,
                                                 BSP_BUTTON_ACTION_RELEASE,
                                                 BSP_EVENT_KEY_2);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);


    return NRF_SUCCESS;
}


/**@brief Function for configuring the buttons for advertisement.
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t advertising_buttons_configure()
{
    uint32_t err_code;

    err_code = bsp_event_to_button_action_assign(BTN_ID_PWR,
                                                 BSP_BUTTON_ACTION_LONG_PUSH,
                                                 BSP_EVENT_KEY_1);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_STA,
                                                 BSP_BUTTON_ACTION_RELEASE,
                                                 BSP_EVENT_KEY_2);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

    return NRF_SUCCESS;
}






/**
 * @brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 * @param[in]   p_context       Context.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            if (m_num_connections == 0)
            {
                err_code = connection_buttons_configure();
                CALL_HANDLER_ON_ERROR(err_code);
            }

            m_num_connections++;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_num_connections--;

            if (m_num_connections == 0)
            {
                err_code = advertising_buttons_configure();
                CALL_HANDLER_ON_ERROR(err_code);
            }
            break;

        default:
            break;
    }
}

NRF_SDH_BLE_OBSERVER(m_ble_observer, BSP_BTN_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);


uint32_t bsp_btn_ble_init(bsp_btn_ble_error_handler_t error_handler, bsp_event_t * p_startup_bsp_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    m_error_handler = error_handler;

    if (m_num_connections == 0)
    {
        err_code = advertising_buttons_configure();
    }

    return err_code;
}
