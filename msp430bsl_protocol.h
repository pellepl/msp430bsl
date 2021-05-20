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

/*
 * msp430bsl_protocol.h
 *
 * Constants gathered from spec
 *
 *  Created on: Mar 1, 2018
 *      Author: petera
 */

#ifndef _MSP430BSL_PROTOCOL_H_
#define _MSP430BSL_PROTOCOL_H_

/* should be 250UL but wont work - current value derived from using ttyapp.c */
#define DELAY_PIN_STATE_NS          (120UL*100000UL)

#define DELAY_TX_AFTER_RX_NS        (1200UL*1000UL)

#define BSL_DEFAULT_BAUDRATE        (9600)

#define BSL_HDR                     (0x80)

#define BSL124_SYNC                 (0x80)
#define BSL124_ACK                  (0x90)
#define BSL124_NACK                 (0xa0)

#define BSL124_CMD_RXDATABLOCK      (0x12)
#define BSL124_CMD_RXPASSWORD       (0x10)
#define BSL124_CMD_ERASE_SEGMENT    (0x16)
#define _BSL124_CMD_ERASE_MAIN_INFO (0x96)
#define BSL124_CMD_MASS_ERASE       (0x18)
#define BSL124_CMD_ERASE_CHECK      (0x1c)
#define BSL124_CMD_SET_BAUDRATE     (0x20)
#define BSL124_CMD_SET_MEMOFFSET    (0x21)
#define BSL124_CMD_LOAD_PC          (0x1a)
#define BSL124_CMD_TXDATABLOCK      (0x14)
#define BSL124_CMD_TXBSLVERSION     (0x1e)

#define BSL56_CMD_RXDATABLOCK       (0x10)
#define BSL56_CMD_RXDATABLOCKFAST   (0x1b)
#define BSL56_CMD_RXPASSWORD        (0x11)
#define BSL56_CMD_ERASE_SEGMENT     (0x12)
#define BSL56_CMD_UNLOCK_LOCK_INFO  (0x13)
#define BSL56_CMD_MASS_ERASE        (0x15)
#define BSL56_CMD_CRCCHECK          (0x16)
#define BSL56_CMD_LOADPC            (0x17)
#define BSL56_CMD_TXDATABLOCK       (0x18)
#define BSL56_CMD_TXBSLVERSION      (0x19)

/** ACK */
#define BSL_OK                      (0x00)
/** Header incorrect. The packet did not begin with the required 0x80 value. */
#define BSL56_ERR_HDR               (0x51)
/** Checksum incorrect. The packet did not have the correct checksum value. */
#define BSL56_ERR_CHK               (0x52)
/** Packet size zero. The size for the BSL core command was given as 0. */
#define BSL56_ERR_SZE               (0x53)
/** Packet size exceeds buffer. The packet size given is too big for the RX buffer. */
#define BSL56_ERR_SZF               (0x54)
/** Unknown error */
#define BSL56_ERR                   (0x55)
/** Unknown baud rate. The supplied data for baud rate change is not a known value. */
#define BSL56_ERR_BR                (0x56)

#define BSL56_RES_CMD               (0x3a)
#define BSL56_RES_MSG               (0x3b)

/** Operation Successful */
#define BSL56_MSG_OK                (0x00)
/** Flash Write Check Failed. After programming, a CRC is run on the programmed data. If the CRC does not match the expected result, this error is returned. */
#define BSL56_MSG_FLASHWRCHKFAIL    (0x01)
/** Flash Fail Bit Set. An operation set the FAIL bit in the flash controller (see the MSP430x5xx and MSP430x6xxFamily User's Guide for more details on the flash fail bit). */
#define BSL56_MSG_FLASHFAILBIT      (0x02)
/** Voltage Change During Program. The VPE was set during the requested write operation (see the MSP430x5xxand MSP430x6xx Family User's Guide for more details on the VPE bit). */
#define BSL56_MSG_FLASHVCHANGE      (0x03)
/** BSL Locked. The correct password has not yet been supplied to unlock the BSL. */
#define BSL56_MSG_LOCKED            (0x04)
/** BSL Password Error. An incorrect password was supplied to the BSL when attempting an unlock. */
#define BSL56_MSG_ACCESSDENIED      (0x05)
/** Byte Write Forbidden. This error is returned when a byte write is attempted in a flash area. */
#define BSL56_MSG_FLASHBYTEDENIED   (0x06)
/** Unknown Command. The command given to the BSL was not recognized. */
#define BSL56_MSG_UNKNOWNCMD        (0x07)
/** Packet Length Exceeds Buffer Size. The supplied packet length value is too large to be held in the BSL receive buffer */
#define BSL56_MSG_PKTTOOLONG        (0x08)


#endif /* _MSP430BSL_PROTOCOL_H_ */
