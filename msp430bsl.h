//The MIT License (MIT)
//
//Copyright (c) 2018 Peter Andersson (pelleplutt1976<at>gmail.com)
//
//Permission is hereby granted, free of charge, to any person obtaining a copy of
//this software and associated documentation files (the "Software"), to deal in
//the Software without restriction, including without limitation the rights to
//use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
//the Software, and to permit persons to whom the Software is furnished to do so,
//subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
//FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
//COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
//IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _MSP430BSL_H
#define _MSP430BSL_H

#include <stdint.h>

#define MSP430BSL_VERSION_API     1
#define MSP430BSL_VERSION_MINOR   0

#define BSL_ERR                     (-4)
#define BSL_ERR_NOT_SUPPORTED       (-5)
#define BSL_ERR_HDR_BAD             (-6)
#define BSL_ERR_PKT_TOO_LONG        (-7)
#define BSL_ERR_CHK_BAD             (-8)
#define BSL_ERR_LEN_BAD             (-9)
#define BSL124_ERR_BAD_ACK          (-11)
#define BSL124_ERR_NACK             (-12)
#define BSL124_ERR_PKT_TOO_SMALL    (-13)
#define BSL124_ERR_PKT_NOT_EVEN     (-14)
#define BSL56_ERR_RX_CMD_BAD        (-51)
#define BSL56_ERR_WRITE_CHECK_FAIL  (-52)
#define BSL56_ERR_FLASH_FAIL_BIT    (-53)
#define BSL56_ERR_FLASH_VCHANGE     (-54)
#define BSL56_ERR_LOCKED            (-55)
#define BSL56_ERR_ACCESS_DENIED     (-56)
#define BSL56_ERR_FLASH_BYTE_FAIL   (-57)
#define BSL56_ERR_UNKNOWN_CMD       (-58)

/** BSL pin */
typedef enum {
  BSL_PIN_RESET = 0,
  BSL_PIN_TEST
} bsl_pin_t;

/** BSL pin state */
typedef enum {
  BSL_PINSTATE_LO = 0,
  BSL_PINSTATE_HI,
  BSL_PINSTATE_TRI
} bsl_pinstate_t;

/** Sleeps at least given amount of of nanoseconds.
    @param ns     number of nanoseconds to sleep */
typedef void (*bsl_hal_sleep_ns_fn_t)(uint32_t ns);

/** Sets given pin to given state.
    @param pin    the pin
    @param state  the state of the pin
    @return       0 on success, nonzero otherwize */
typedef int (*bsl_hal_gpio_set_fn_t)(bsl_pin_t pin, bsl_pinstate_t state);

/** Configures the serial port.
    In case of UART, it must be configured with 8 databits, 1 stopbit,
    and even parity.
    @param rate   the bit rate of the serial port
    @return       0 on success, nonzero otherwize */
typedef int (*bsl_hal_ser_config_t)(uint32_t rate);


/** Reads from the serial port, blocking.
    @param buf    where to place read data
    @param len    how many bytes to read
    @return       0 on success, nonzero otherwize */
typedef int (*bsl_hal_ser_read_t)(uint8_t *buf, uint16_t len);

/** Writes to the serial port, blocking.
    @param buf    what to write
    @param len    how many bytes to write
    @return       0 on success, nonzero otherwize */
typedef int (*bsl_hal_ser_write_t)(const uint8_t *buf, uint16_t len);

/** HAL config struct
    All HAL functions are supposed to be blocking.
    Regarding serial communication, there is no full duplex communication.
 */
typedef struct {
  /** nanosecond sleep HAL function */
  bsl_hal_sleep_ns_fn_t sleep_fn;
  /** GPIO set hi/lo/tri HAL function */
  bsl_hal_gpio_set_fn_t gpio_set_fn;
  /** serial config HAL function */
  bsl_hal_ser_config_t ser_config_fn;
  /** serial read HAL function */
  bsl_hal_ser_read_t ser_read_fn;
  /** serial write HAL function */
  bsl_hal_ser_write_t ser_write_fn;
} bsl_hal_t;

/** BSL version struct */
typedef struct {
  union {
    struct {
      uint16_t device_family;
      uint16_t bsl_version;
    } bsl124;
    struct {
      /* 0x00 = TI */
      uint8_t vendor;
      uint8_t interpreter_version;
      uint8_t api_version;
      uint8_t periph_version;
    } bsl56;
  };
} bsl_version_t;

enum bsl_type {
  BSL_TYPE_124_FAMILIES = 0,
  BSL_TYPE_56_FAMILIES
};

/** Initiates the library and sets the HAL functions.
    @param type   if the chip is of 1xx,2xx, or 4xx families;
                  or of 5xx, or 6xx families
    @param hal    the hal functions.
    @return       0 on success, nonzero otherwize */
int bsl_init(enum bsl_type type, bsl_hal_t *hal);

/** Resets the MSP430.
    @return       0 on success, nonzero otherwize */
int bsl_reset(void);

/** Enters BSL mode for MSP430.
    @param dedicated_jtag_pins  whether the msp430 in question have dedicated
                                JTAG pins or shared JTAG pins. The MSP430FR2311
                                for instance have shared JTAG pins - in this
                                case 0 must be passed.
    @return                     0 on success, nonzero otherwize */
int bsl_enter(int dedicated_jtag_pins);

/** Unlocks the BSL commands.
    If wrong password is given, the BSL will mass erase and reset password
    to 0xffff..ff.
    @param pass   the 32 byte password. If NULL is passed the default password
                  is assumed (0xffff..ff).
    @return       0 on success, nonzero otherwize */
int bsl_unlock(uint8_t pass[32]);

/** Query the BSL version.
    @param v      a struct to be populated with BSL version information.
    @return       0 on success, nonzero otherwize */
int bsl_get_version(bsl_version_t *v);

/** Read memory.
    @param addr   the address to read from
    @param buf    where to put read data
    @param len    how many bytes to read
    @return       0 on success, nonzero otherwize */
int bsl_read_mem(uint32_t addr, uint8_t *buf, uint16_t len);

/** Write memory.
    @param addr   the address to write to
    @param buf    data to write
    @param len    how many bytes to write
    @return       0 on success, nonzero otherwize */
int bsl_write_mem(uint32_t addr, const uint8_t *buf, uint16_t len);

/** Erase segment.
    @param addr   the address to the segment
    @return       0 on success, nonzero otherwize */
int bsl_erase_segment(uint32_t addr);

/** Performs a mass erase. Does not erase information memory.
    @return       0 on success, nonzero otherwize */
int bsl_mass_erase(void);

/** Jump directly to given address. No core response is given.
    @param addr   the address to jump to
    @return       0 on success, nonzero otherwize */
int bsl_load_pc(uint32_t addr);

/** The MCU performs a check for only 0xFFs within given range.
    MSP430 BSL 1xx 2xx 4xx families only.
    @param addr   the address to check
    @param len    how many bytes to check
    @param crc    populated with the crc value
    @return       0 on success (only 0xFFs),
                  BSL124_ERR_NACK if not,
                  other nonzero on other error*/
int bsl_erase_check(uint32_t addr, uint16_t len);

/** The MCU performs a 16-bit CRC check using the CCITT standard.
    MSP430 BSL 5xx 6xx families only.
    @param addr   the address to check
    @param len    how many bytes to check
    @param crc    populated with the crc value
    @return       0 on success, nonzero otherwize */
int bsl_crc_check(uint32_t addr, uint16_t len, uint16_t *crc);

/** Toggle INFO_A lock to either protect or lock the INFO_A segment.
    MSP430 BSL 5xx 6xx families only.
    @return       0 on success, nonzero otherwize */
int bsl_unlock_and_lock_info(void);


#endif // _MSP430BSL_H
