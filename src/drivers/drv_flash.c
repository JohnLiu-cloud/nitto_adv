#include "nrf.h"
#include "nrf_soc.h"
#include "nordic_common.h"
#include "app_util.h"
#include "nrf_fstorage.h"
#include "app_error.h"
#include "nrf_log.h"
#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"
#endif
#include "drv_flash.h"

#define FLASH_UNIT_SIZE 6

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);


NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x3e000,
    .end_addr   = 0x3ffff,
};

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}


void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}

void fstorage_init( void )
{
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;
#ifdef SOFTDEVICE_PRESENT
    NRF_LOG_INFO("SoftDevice is present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_sd implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
#else
    NRF_LOG_INFO("SoftDevice not present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_nvmc implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
     * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the
     * SoftDevice is disabled or not present.
     *
     * Using this implementation when the SoftDevice is enabled results in a hardfault. */
    p_fs_api = &nrf_fstorage_nvmc;
#endif
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    print_flash_info(&fstorage);

    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    (void) nrf5_flash_end_addr_get();
}

void fstorage_write( uint8_t sn , uint8_t mode , uint16_t id , uint16_t interval )
{
    ret_code_t rc;
    uint8_t w_data[FLASH_UNIT_SIZE];
    w_data[0] = sn;
    w_data[1] = mode;
    w_data[2] = (uint8_t)(id>>8);
    w_data[3] = (uint8_t)id;
    w_data[4] = (uint8_t)(interval>>8);
    w_data[5] = (uint8_t)interval;

    rc = nrf_fstorage_erase(&fstorage, 0x3e000,1, NULL);
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    /* Let's write to flash. */
    NRF_LOG_INFO("Writing \"%d,%d\" to flash.", w_data[0],w_data[1] );
    rc = nrf_fstorage_write(&fstorage, 0x3e000, &w_data, FLASH_UNIT_SIZE, NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("Done.");
}

void fstorage_read( uint8_t *sn , uint8_t *mode , uint16_t *id , uint16_t *interval )
{
  ret_code_t rc;
  uint8_t r_data[4];
    /* Read data. */
    rc = nrf_fstorage_read(&fstorage, 0x3e000, r_data, FLASH_UNIT_SIZE );
    if (rc != NRF_SUCCESS)
    {
      NRF_LOG_ERROR("fstorage read err=%s.",nrf_strerror_get(rc));
      return;
    }
    NRF_LOG_INFO("read \"0x%02x,0x%02x\" in flash.", r_data[0],r_data[1] );
    if(  r_data[0] != 0xff &&  r_data[1] != 0xff )
    {
      *sn = r_data[0];
      *mode = r_data[1];
    } 
    else
    {
      *sn = 0;
      *mode = 0;   
    }
    *id = (uint16_t)(r_data[2]<<8) + (uint16_t)r_data[3];

    *interval = (uint16_t)(r_data[4]<<8) + (uint16_t)r_data[5]; 

}