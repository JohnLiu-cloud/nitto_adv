#ifndef DRV_FLASH_H
#define DRV_FLASH_H

void fstorage_init( void );

void fstorage_write( uint8_t sn , uint8_t mode , uint16_t id , uint16_t interval );

void fstorage_read( uint8_t *sn , uint8_t *mode , uint16_t *id , uint16_t *interval );

#endif //DRV_FLASH_H