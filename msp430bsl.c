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
 * msp430bsl.c
 *
 *  Created on: Mar 1, 2018
 *      Author: petera
 */


#include "msp430bsl.h"
#include "msp430bsl_protocol.h"


#ifdef _LINUX_DEBUG
#include <stdio.h>
#ifndef DEFAULT_DBG_LEVEL
#define DEFAULT_DBG_LEVEL 0
#endif
int _msp_bsl_dbg = DEFAULT_DBG_LEVEL;
#define err(f, ...) do { if (_msp_bsl_dbg > 0) printf("[BSL E] %s:%d\t" f "\n", __FILE__, __LINE__, ## __VA_ARGS__); } while (0)
#define dbg(f, ...) do { if (_msp_bsl_dbg > 1) printf("[BSL D] %s:%d\t" f "\n", __FILE__, __LINE__, ## __VA_ARGS__); } while (0)
#else
#ifdef _GW_DEBUG
#include <debug.h>
#define err(f, ...) do { dbg_printf("[BSL E] %s:%d   " f "\n", __FILE__, __LINE__, ## __VA_ARGS__); } while (0)
#define dbg(f, ...) do { dbg_printf("[BSL D] %s:%d   " f "\n", __FILE__, __LINE__, ## __VA_ARGS__); } while (0)
#else
#ifndef err
#define err(f, ...) do {} while (0)
#endif
#ifndef dbg
#define dbg(f, ...) do {} while (0)
#endif
#endif
#endif

static struct {
  enum bsl_type type;
  bsl_hal_t *hal;
} bsl;

///////////////////////////////////////////////////////////////////////////////
//                              BSL124 COMMANDS                              //
///////////////////////////////////////////////////////////////////////////////

static uint16_t bsl124_chk(const uint8_t *hdr, uint8_t hdr_len, const uint8_t *b, uint8_t len)
{
  uint16_t c = 0;
  for (uint8_t i = 0; i < hdr_len; i+=2) {
    c ^= (hdr[i] | (hdr[i+1] << 8));
  }
  for (uint8_t i = 0; i < len; i+=2) {
    c ^= (b[i] | (b[i+1] << 8));
  }
  return ~c;
}

static int bsl124_sync(void)
{
  int res;
  uint8_t c = BSL124_SYNC;
  res = bsl.hal->ser_write_fn(&c, 1);
  if (res) return res;
  c = (uint8_t)~BSL124_ACK;
  res = bsl.hal->ser_read_fn(&c, 1);
  if (res) return res;
  if (c != BSL124_ACK) return BSL124_ERR_BAD_ACK;
  bsl.hal->sleep_fn(DELAY_TX_AFTER_RX_NS);
  return res;
}

static int bsl124_recv_ack(void)
{
  uint8_t c;
  int res = bsl.hal->ser_read_fn(&c, 1);
  if (res) return res;
  dbg("got ack %02x", c);
  bsl.hal->sleep_fn(DELAY_TX_AFTER_RX_NS);
  if (c == BSL124_ACK)
    return BSL_OK;
  else if (c == BSL124_NACK)
    return BSL124_ERR_NACK;

  return BSL124_ERR_BAD_ACK;
}

static int bsl124_recv_pkt(uint8_t *buf, uint8_t len)
{
  int res;
  uint8_t hdr[4];

  res = bsl.hal->ser_read_fn(hdr, 1);
  if (res) return res;
  dbg("got hdr %02x", hdr[0]);
  if (hdr[0] != BSL_HDR) return BSL_ERR_HDR_BAD;

  res = bsl.hal->ser_read_fn(&hdr[1], 3);
  if (res) return res;
  if (hdr[2] != hdr[3] || (hdr[2] & 1))
    return BSL_ERR_LEN_BAD;

  uint8_t rxlen = hdr[2];
  if (rxlen > len) return BSL_ERR_PKT_TOO_LONG;
  res = bsl.hal->ser_read_fn(buf, rxlen);
  if (res) return res;

  uint8_t chkbuf[2];
  res = bsl.hal->ser_read_fn(chkbuf, 2);
  if (res) return res;
  uint16_t lchk = bsl124_chk(hdr, 4, buf, rxlen);
  if (lchk != (chkbuf[0] | (chkbuf[1] << 8)))
    res = BSL_ERR_CHK_BAD;

  bsl.hal->sleep_fn(DELAY_TX_AFTER_RX_NS);
  return res;
}

static int bsl124_command(uint8_t cmd, uint16_t a, uint16_t l, const uint8_t *bufp, uint8_t lenp)
{
  static uint8_t hdr124[8];
  int res;
  uint8_t pkt_len = 4 + lenp;


  if (pkt_len & 1) return BSL124_ERR_PKT_NOT_EVEN;
  if (pkt_len > 254) return BSL_ERR_PKT_TOO_LONG;

  switch (cmd) {
  case BSL124_CMD_RXDATABLOCK:
    if (lenp < 4) return BSL124_ERR_PKT_TOO_SMALL;
    l = lenp;
    break;
  case BSL124_CMD_ERASE_SEGMENT:
    l = 0xa502;// magic nbr from BSL spec http://www.ti.com/lit/ug/slau319r/slau319r.pdf
    break;
  case _BSL124_CMD_ERASE_MAIN_INFO:
    cmd = BSL124_CMD_ERASE_SEGMENT;
    l = 0xa504;// magic nbr from BSL spec http://www.ti.com/lit/ug/slau319r/slau319r.pdf
    break;
  case BSL124_CMD_MASS_ERASE:
    l = 0xa506;// magic nbr from BSL spec http://www.ti.com/lit/ug/slau319r/slau319r.pdf
    break;
  case BSL124_CMD_SET_MEMOFFSET:
    l = a;
    break;
  }
  hdr124[0] = BSL_HDR;
  hdr124[1] = cmd;
  hdr124[2] = pkt_len;
  hdr124[3] = pkt_len;
  hdr124[4] = a;
  hdr124[5] = a >> 8;
  hdr124[6] = l;
  hdr124[7] = l >> 8;

  if (bufp == (void*)0) lenp = 0;
  uint16_t chk = bsl124_chk(hdr124, 8, bufp, lenp);

  dbg("tx cmd %02x, len %d, chk %04x", hdr124[1], 8+lenp, chk);

  res = bsl124_sync();
  if (res) return res;

  res = bsl.hal->ser_write_fn(hdr124, sizeof(hdr124));
  if (res) return res;
  if (lenp) {
    res = bsl.hal->ser_write_fn(bufp, lenp);
    if (res) return res;
  }
  res = bsl.hal->ser_write_fn((uint8_t[2]){chk, chk>>8}, 2);
  return res;
}

static int _bsl124_addr_hi_check(uint32_t addr)
{
  int res = BSL_OK;
  addr >>= 16;
  if (addr) {
    res = bsl124_command(BSL124_CMD_SET_MEMOFFSET, addr, 0, (void*)0, 0);
    if (res) return res;
    res = bsl124_recv_ack();
  }
  return res;
}

static int bsl124_unlock(uint8_t p[32])
{
  uint8_t *passwrd = (uint8_t[32]){
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  };
  if (p) passwrd = p;
  int res;
  res = bsl124_command(BSL124_CMD_RXPASSWORD, 0, 0, passwrd, 32);
  if (res) return res;
  res = bsl124_recv_ack();
  return res;
}

static int bsl124_get_version(bsl_version_t *v)
{
  int res;
  res = bsl124_command(BSL124_CMD_TXBSLVERSION, 0, 0, (void*)0, 0);
  if (res) return res;
  uint8_t buf[0x10];
  res = bsl124_recv_pkt(buf, sizeof(buf));
  if (res) return res;

  v->bsl124.device_family = (buf[0] << 8) | buf[1];
  v->bsl124.bsl_version = (buf[10] << 8) | buf[11];
  return res;
}

static int bsl124_read_mem(uint32_t addr, uint8_t *buf, uint16_t len)
{
  int res;
  res = _bsl124_addr_hi_check(addr);
  if (res) return res;
  res = bsl124_command(BSL124_CMD_TXDATABLOCK, addr, len, (void*)0, 0);
  if (res) return res;
  res = bsl124_recv_pkt(buf, len);
  return res;
}

static int bsl124_write_mem(uint32_t addr, const uint8_t *buf, uint16_t len)
{
  int res;
  res = _bsl124_addr_hi_check(addr);
  if (res) return res;
  res = bsl124_command(BSL124_CMD_RXDATABLOCK, addr, len, buf, len);
  if (res) return res;
  res = bsl124_recv_ack();
  return res;
}

static int bsl124_erase_segment(uint32_t addr)
{
  int res;
  res = _bsl124_addr_hi_check(addr);
  if (res) return res;
  res = bsl124_command(BSL124_CMD_ERASE_SEGMENT, addr, 0, (void*)0, 0);
  if (res) return res;
  res = bsl124_recv_ack();
  return res;
}

static int bsl124_mass_erase(void)
{
  int res;
  res = bsl124_command(BSL124_CMD_MASS_ERASE, 0, 0, (void*)0, 0);
  if (res) return res;
  res = bsl124_recv_ack();
  return res;
}

static int bsl124_erase_check(uint32_t addr, uint16_t len)
{
  int res;
  res = _bsl124_addr_hi_check(addr);
  if (res) return res;
  res = bsl124_command(BSL124_CMD_ERASE_CHECK, addr, len, (void*)0, 0);
  if (res) return res;
  res = bsl124_recv_ack();
  return res;
}

static int bsl124_load_pc(uint32_t addr)
{
  int res;
  res = _bsl124_addr_hi_check(addr);
  if (res) return res;
  res = bsl124_command(BSL124_CMD_LOAD_PC, addr, 0, (void*)0, 0);
  return res;
  // no ack
}

///////////////////////////////////////////////////////////////////////////////
//                              BSL56 COMMANDS                               //
///////////////////////////////////////////////////////////////////////////////

static uint16_t bsl56_crc(uint16_t crc, uint8_t data)
{
  uint8_t x = ((crc >> 8) ^ data) & 0xff;
  x ^= x >> 4;
  return ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xffff;
}

static uint16_t bsl56_pktchk_w_payload(uint8_t first_byte, const uint8_t *buf, uint16_t len, const uint8_t *bufp, uint16_t lenp)
{
  uint16_t i;
  uint16_t crc = 0xffff;
  crc = bsl56_crc(crc, first_byte);
  for (i = 0; i < len; i++) crc = bsl56_crc(crc, buf[i]);
  if (lenp) {
    for (i = 0; i < lenp; i++) crc = bsl56_crc(crc, bufp[i]);
  }
  return crc;
}

static uint16_t bsl56_pktchk(uint8_t first_byte, const uint8_t *buf, uint16_t len)
{
  return bsl56_pktchk_w_payload(first_byte, buf, len, (void *)0, 0);
}

static int bsl56_command_w_payload(const uint8_t *buf, uint16_t len, const uint8_t *bufp, uint16_t lenp)
{
  uint8_t hdr_buf[3] = {BSL_HDR, (len+lenp) & 0xff, (len+lenp) >> 8};
  uint16_t chk = bsl56_pktchk_w_payload(buf[0], &buf[1], len-1, bufp, lenp);
  uint8_t chk_buf[2] = {chk & 0xff, chk >> 8};
  uint8_t ack;
  int res;

  dbg("tx cmd %02x, len %d, chk %04x", buf[0], len+lenp, chk);
  res = bsl.hal->ser_write_fn(hdr_buf, sizeof(hdr_buf));
  if (res) return res;
  res = bsl.hal->ser_write_fn(buf, len);
  if (res) return res;
  if (lenp) {
    res = bsl.hal->ser_write_fn(bufp, lenp);
    if (res) return res;
  }
  res = bsl.hal->ser_write_fn(chk_buf, sizeof(chk_buf));
  if (res) return res;
  res = bsl.hal->ser_read_fn(&ack, 1);
  if (res) return res;
  if (ack) err("got ack %02x", ack);
  switch (ack) {
  case BSL_OK: res = BSL_OK; break;
  case BSL56_ERR_HDR: res = BSL_ERR_HDR_BAD; break;
  case BSL56_ERR_CHK: res = BSL_ERR_CHK_BAD; break;
  case BSL56_ERR_SZE: res = BSL_ERR_LEN_BAD; break;
  case BSL56_ERR_SZF: res = BSL_ERR_LEN_BAD; break;
  default: res = BSL_ERR; break;
  }

  return res;
}

static int bsl56_command(const uint8_t *buf, uint16_t len)
{
  return bsl56_command_w_payload(buf, len, (void *)0, 0);
}

static int bsl56_receive(uint8_t *buf, uint16_t *max_len)
{
  int res;
  uint8_t hdr_buf[3];
  uint8_t chk_buf[2];
  res = bsl.hal->ser_read_fn(hdr_buf, sizeof(hdr_buf));
  if (res) return res;
  if (hdr_buf[0] != BSL_HDR) {
    err("unknown response header");
    return BSL_ERR_HDR_BAD;
  }

  uint16_t len = hdr_buf[1] | (hdr_buf[2]<<8);
  len--;
  if (len > *max_len) {
    err("rx overflow, %d of %d", len, *max_len);
    return BSL_ERR_PKT_TOO_LONG;
  }

  if (len == 0) {
    err("invalid length");
    return BSL_ERR;
  }

  uint8_t cmd;
  res = bsl.hal->ser_read_fn(&cmd, 1);
  if (res) return res;

  res = bsl.hal->ser_read_fn(buf, len);
  if (res) return res;

  if (cmd == BSL56_RES_MSG) {
    if (buf[0] != BSL56_MSG_OK) {
      err("message %02x", buf[0]);
      switch (buf[0]) {
      case BSL56_MSG_FLASHWRCHKFAIL: return BSL56_ERR_WRITE_CHECK_FAIL; break;
      case BSL56_MSG_FLASHFAILBIT: return BSL56_ERR_FLASH_FAIL_BIT; break;
      case BSL56_MSG_FLASHVCHANGE: return BSL56_ERR_FLASH_VCHANGE; break;
      case BSL56_MSG_LOCKED: return BSL56_ERR_LOCKED; break;
      case BSL56_MSG_ACCESSDENIED: return BSL56_ERR_ACCESS_DENIED; break;
      case BSL56_MSG_FLASHBYTEDENIED: return BSL56_ERR_FLASH_BYTE_FAIL; break;
      case BSL56_MSG_UNKNOWNCMD: return BSL56_ERR_UNKNOWN_CMD; break;
      case BSL56_MSG_PKTTOOLONG: return BSL_ERR_PKT_TOO_LONG; break;
      default: return BSL_ERR; break;
      }
      return buf[0];
    }
  } else if (cmd != BSL56_RES_CMD) {
    err("unknown response command");
    return BSL56_ERR_RX_CMD_BAD;
  }

  res = bsl.hal->ser_read_fn(chk_buf, sizeof(chk_buf));
  if (res) return res;

  uint16_t chk = chk_buf[0] | (chk_buf[1]<<8);
  dbg("rx len %d, chk %04x", len, chk);
  uint16_t localchk = bsl56_pktchk(cmd, buf, len);
  if (chk != localchk) {
    err("chksums differ, local %04x, remote %04x", localchk, chk);
    return BSL_ERR_CHK_BAD;
  }
  *max_len = len;

  bsl.hal->sleep_fn(DELAY_TX_AFTER_RX_NS);

  return 0;
}

static int bsl56_unlock(uint8_t p[32])
{
  uint8_t *passwrd = (uint8_t[32]){
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
      0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  };
  if (p) passwrd = p;
  int res;
  uint8_t cmd[33];
  int i;
  cmd[0] = BSL56_CMD_RXPASSWORD;
  for (i = 0; i < 32; i++) cmd[1+i] = passwrd[i];
  res = bsl56_command(cmd, sizeof(cmd));
  if (res) return res;
  uint8_t r_buf[4];
  uint16_t l = sizeof(r_buf);
  res = bsl56_receive(r_buf, &l);
  return res;
}

static int bsl56_get_version(bsl_version_t *v)
{
  int res;
  res = bsl56_command((uint8_t[1]){BSL56_CMD_TXBSLVERSION}, 1);
  if (res) return res;
  uint8_t v_buf[4];
  uint16_t l = sizeof(v_buf);
  res = bsl56_receive(v_buf, &l);
  if (res) return res;
  v->bsl56.vendor = v_buf[0];
  v->bsl56.interpreter_version = v_buf[1];
  v->bsl56.api_version = v_buf[2];
  v->bsl56.periph_version = v_buf[3];
  return res;
}

static int bsl56_read_mem(uint32_t addr, uint8_t *buf, uint16_t len)
{
  int res;
  res = bsl56_command((uint8_t[1+3+2]){
    BSL56_CMD_TXDATABLOCK,
    addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff,
    len  & 0xff, (len >> 8) & 0xff},
    1+3+2);
  if (res) return res;
  res = bsl56_receive(buf, &len);
  return res;
}

static int bsl56_write_mem(uint32_t addr, const uint8_t *buf, uint16_t len)
{
  int res;
  res = bsl56_command_w_payload((uint8_t[1+3]){
    BSL56_CMD_RXDATABLOCK,
    addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff},
    1+3, buf, len);
  if (res) return res;
  uint8_t rsp[4];
  uint16_t l = sizeof(rsp);
  res = bsl56_receive(rsp, &l);
  return res;
}

static int bsl56_erase_segment(uint32_t addr)
{
  int res;
  res = bsl56_command((uint8_t[1+3]){
    BSL56_CMD_ERASE_SEGMENT,
    addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff},
    1+3);
  if (res) return res;
  uint8_t rsp[4];
  uint16_t l = sizeof(rsp);
  res = bsl56_receive(rsp, &l);
  return res;
}

static int bsl56_mass_erase(void)
{
  int res;
  res = bsl56_command((uint8_t[1]){BSL56_CMD_MASS_ERASE}, 1);
  if (res) return res;
  uint8_t rsp[4];
  uint16_t l = sizeof(rsp);
  res = bsl56_receive(rsp, &l);
  return res;
}

static int bsl56_unlock_and_lock_info(void)
{
  int res;
  res = bsl56_command((uint8_t[1]){BSL56_CMD_UNLOCK_LOCK_INFO}, 1);
  if (res) return res;
  uint8_t rsp[4];
  uint16_t l = sizeof(rsp);
  res = bsl56_receive(rsp, &l);
  return res;
}

static int bsl56_crc_check(uint32_t addr, uint16_t len, uint16_t *crc)
{
  int res;
  res = bsl56_command((uint8_t[1+3+2]){
    BSL56_CMD_CRCCHECK,
    addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff,
    len  & 0xff, (len >> 8) & 0xff},
    1+3+2);
  if (res) return res;
  uint8_t rsp[4];
  uint16_t l = sizeof(rsp);
  res = bsl56_receive(rsp, &l);
  *crc = (rsp[0] & 0xff) | ((rsp[1] & 0xff) << 8);
  return res;
}

static int bsl56_load_pc(uint32_t addr)
{
  int res;
  res = bsl56_command((uint8_t[1+3]){
    BSL56_CMD_LOADPC,
    addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff},
    1+3);
  // no response
  return res;
}

///////////////////////////////////////////////////////////////////////////////
//                                    API                                    //
///////////////////////////////////////////////////////////////////////////////

int bsl_init(enum bsl_type type, bsl_hal_t *hal)
{
  bsl.type = type;
  bsl.hal = hal;
  return 0;
}

int bsl_enter(int dedicated_jtag_pins)
{
  int res;
  bsl_pinstate_t test_assert = dedicated_jtag_pins ? BSL_PINSTATE_LO : BSL_PINSTATE_HI;
  bsl_pinstate_t test_noasse = dedicated_jtag_pins ? BSL_PINSTATE_HI : BSL_PINSTATE_LO;
  dbg("config uart to %d bps", BSL_DEFAULT_BAUDRATE);
  res = bsl.hal->ser_config_fn(BSL_DEFAULT_BAUDRATE);
  if (res) goto end;
  dbg("sending bsl enter sequence on RST & TEST pins");
  res = bsl.hal->gpio_set_fn(BSL_PIN_RESET, BSL_PINSTATE_LO);
  if (res) goto end;
  res = bsl.hal->gpio_set_fn(BSL_PIN_TEST, test_noasse);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS*2);

  res = bsl.hal->gpio_set_fn(BSL_PIN_TEST, test_assert);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS);

  res = bsl.hal->gpio_set_fn(BSL_PIN_TEST, test_noasse);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS);

  res = bsl.hal->gpio_set_fn(BSL_PIN_TEST, test_assert);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS);

  res = bsl.hal->gpio_set_fn(BSL_PIN_RESET, BSL_PINSTATE_HI);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS);

  res = bsl.hal->gpio_set_fn(BSL_PIN_TEST, test_noasse);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS);

end:
  (void)bsl.hal->gpio_set_fn(BSL_PIN_RESET, BSL_PINSTATE_TRI);
  (void)bsl.hal->gpio_set_fn(BSL_PIN_TEST, BSL_PINSTATE_TRI);
  return res;
}

int bsl_reset(void)
{
  int res;
  res = bsl.hal->gpio_set_fn(BSL_PIN_RESET, BSL_PINSTATE_LO);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS*2);
  res = bsl.hal->gpio_set_fn(BSL_PIN_RESET, BSL_PINSTATE_HI);
  if (res) goto end;
  bsl.hal->sleep_fn(DELAY_PIN_STATE_NS);
end:
  (void)bsl.hal->gpio_set_fn(BSL_PIN_RESET, BSL_PINSTATE_TRI);
  return res;
}

int bsl_unlock(uint8_t p[32])
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_unlock(p);
  else
    return bsl56_unlock(p);
}

int bsl_get_version(bsl_version_t *v)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_get_version(v);
  else
    return bsl56_get_version(v);
}

int bsl_read_mem(uint32_t addr, uint8_t *buf, uint16_t len)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_read_mem(addr, buf, len);
  else
    return bsl56_read_mem(addr, buf, len);
}

int bsl_write_mem(uint32_t addr, const uint8_t *buf, uint16_t len)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_write_mem(addr, buf, len);
  else
    return bsl56_write_mem(addr, buf, len);
}

int bsl_erase_segment(uint32_t addr)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_erase_segment(addr);
  else
    return bsl56_erase_segment(addr);
}

int bsl_mass_erase(void)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_mass_erase();
  else
    return bsl56_mass_erase();
}

int bsl_unlock_and_lock_info(void)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return BSL_ERR_NOT_SUPPORTED;
  else
    return bsl56_unlock_and_lock_info();
}

int bsl_crc_check(uint32_t addr, uint16_t len, uint16_t *crc)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return BSL_ERR_NOT_SUPPORTED;
  else
    return bsl56_crc_check(addr, len, crc);
}

int bsl_erase_check(uint32_t addr, uint16_t len)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_erase_check(addr, len);
  else
    return BSL_ERR_NOT_SUPPORTED;
}

int bsl_load_pc(uint32_t addr)
{
  if (bsl.type == BSL_TYPE_124_FAMILIES)
    return bsl124_load_pc(addr);
  else
    return bsl56_load_pc(addr);
}
