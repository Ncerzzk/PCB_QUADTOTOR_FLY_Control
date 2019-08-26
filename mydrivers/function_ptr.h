#ifndef __FPTR_H
#define __FPTR_H

typedef uint8_t (*i2c_write_buffer_fptr)(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
typedef uint8_t (*i2c_read_buffer_fptr) (uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
typedef void (*i2c_err_reset_fptr) (void);
typedef void (*delay_ms_fptr)(uint32_t t);
typedef void (*delay_us_fptr)(uint32_t t);


#endif