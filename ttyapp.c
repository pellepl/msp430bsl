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

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>

#include <time.h>

#include <fcntl.h>

#include <termios.h>
#include <sys/ioctl.h>

#include <errno.h>

#include "msp430bsl.h"

#define out(f, ...) do { if (__dbg > 0) printf(f, ## __VA_ARGS__); } while (0)
#define err(f, ...) do { if (__dbg > 1) printf("%s:%d\t" f, __FILE__, __LINE__, ## __VA_ARGS__); } while (0)
#define dbg(f, ...) do { if (__dbg > 2) printf("%s:%d\t" f, __FILE__, __LINE__, ## __VA_ARGS__); } while (0)
#define xstr(s) str(s)
#define str(s) #s

#define ARGISYES(_s) ( strcasecmp("yes", _s) == 0 || strcasecmp("y", _s) == 0 || strcmp("1", _s) == 0 || strcasecmp("on", _s) == 0  || strcasecmp("true", _s) == 0  )
#define ARGISNO(_s)  ( strcasecmp("no", _s)  == 0 || strcasecmp("n", _s) == 0 || strcmp("0", _s) == 0 || strcasecmp("off", _s) == 0 || strcasecmp("false", _s) == 0 )

#define DEFAULT_SECTOR_SIZE       512
#define DEFAULT_READ_BUF          32
#define DEFAULT_WRITE_BUF         32

#define ERR_VERIFICATION_FAILED   -111

typedef int (* cli_input_fn_t)(const char *n, const char *val);

typedef enum {
  NOARG = 0, OPTARG, ARG
} cli_input_arg_t;

#define PIN_DTR                   -1
#define PIN_RTS                   -2

typedef struct  {
  const char *name;
  const char *shortname;
  const char *arghelp;
  const char *help;
  cli_input_arg_t arg;
  cli_input_fn_t fn;
} cli_input_def_t;

static volatile int run = 1;
static uint32_t erase_map_sz;
static uint32_t *erase_map = NULL;
static int __dbg = 1;
static int locked = 1;

static char arg_uart_device[256];
static int arg_dedicated_jtag_pins = 0;
static enum bsl_type arg_bsl_family = BSL_TYPE_124_FAMILIES;
static uint32_t arg_sector_size = DEFAULT_SECTOR_SIZE;
static int arg_reset_pin = PIN_DTR;
static int arg_test_pin = PIN_RTS;
static int arg_reset_pin_inv = 0;
static int arg_test_pin_inv = 0;
static int arg_auto_erase = 0;
static int arg_auto_verify = 0;
static uint32_t arg_read_buf = DEFAULT_READ_BUF;
static uint32_t arg_write_buf = DEFAULT_WRITE_BUF;
static uint8_t arg_key[32] = {
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

static struct {
  int ttyfd;
  struct termios term_settings;
  int term_status;
} app;

static int recalc_erase_map(void) {
  if (erase_map) free(erase_map);
  erase_map_sz = 0x400000 / 8 / arg_sector_size;
  erase_map = malloc(erase_map_sz);
  memset(erase_map, 0, erase_map_sz);
  return erase_map == NULL ? ENOMEM : 0;
}

static speed_t baud2speed(int baud)
{
  switch (baud) {
  case 50: return B50;
  case 75: return B75;
  case 110: return B110;
  case 134: return B134;
  case 150: return B150;
  case 200: return B200;
  case 300: return B300;
  case 600: return B600;
  case 1200: return B1200;
  case 1800: return B1800;
  case 2400: return B2400;
  case 4800: return B4800;
  case 9600: return B9600;
  case 19200: return B19200;
  case 38400: return B38400;
#ifdef B57600
  case 57600: return B57600;
#endif
#ifdef B115200
  case 115200: return B115200;
#endif
#ifdef B230400
  case 230400: return B230400;
#endif
#ifdef B460800
  case 460800: return B460800;
#endif
#ifdef B500000
  case 500000: return B500000;
#endif
#ifdef B576000
  case 576000: return B576000;
#endif
#ifdef B921600
  case 921600: return B921600;
#endif
#ifdef B1000000
  case 1000000: return B1000000;
#endif
#ifdef B1152000
  case 1152000: return B1152000;
#endif
#ifdef B1500000
  case 1500000: return B1500000;
#endif
#ifdef B2000000
  case 2000000: return B2000000;
#endif
#ifdef B2500000
  case 2500000: return B2500000;
#endif
#ifdef B3000000
  case 3000000: return B3000000;
#endif
#ifdef B3500000
  case 3500000: return B3500000;
#endif
#ifdef B4000000
  case 4000000: return B4000000;
#endif
  default:      return 0;
  }
}

static int open_uart(const char *dev)
{
  int ttyfd = open(dev, O_RDWR/* | O_DIRECT *//* | O_NONBLOCK */);
  if (ttyfd < 0) {
    err("could not open \"%s\": %s\n", dev, strerror(errno));
    return ttyfd;
  }
  /* get current UART settings */
  if (tcgetattr(ttyfd, &app.term_settings) < 0) {
    err("could get configuration for \"%s\": %s\n", dev, strerror(errno));
    return errno;
  }
  /* get current UART line status */
  if (ioctl(ttyfd, TIOCMGET, &app.term_status) < 0) {
    err("could not get status for \"%s\": %s\n", dev, strerror(errno));
    return errno;
  }
  app.ttyfd = ttyfd;
  return 0;
}

static int config_uart(int baudrate)
{
  /* disable input processing */
  app.term_settings.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF);
  /* disable line processing */
  app.term_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  /* disable character processing, no hang-up */
  app.term_settings.c_cflag &= ~(HUPCL);
  app.term_settings.c_cflag |= CREAD | CLOCAL;
  /* do not insert 0d before 0a */
  app.term_settings.c_oflag &= ~OPOST;

  speed_t rate = baud2speed(baudrate);
  if (rate == 0 ||
      cfsetispeed(&app.term_settings, rate) < 0 ||
      cfsetospeed(&app.term_settings, rate) < 0) {
    err("baud rate not supported\n");
    return EINVAL;
  }
  /* 8 databits */
  app.term_settings.c_cflag &= ~CSIZE;
  app.term_settings.c_cflag |= CS8;
  /* 1 stop bit */
  app.term_settings.c_cflag &= ~CSTOPB;
  /* even parity */
  app.term_settings.c_cflag |= PARENB;
  app.term_settings.c_cflag &= ~PARODD;
  /* see http://unixwiz.net/techtips/termios-vmin-vtime.html */
  /* timeout */
  app.term_settings.c_cc[VTIME] = 10; /* tenths of a sec */
  /* vmin */
  app.term_settings.c_cc[VMIN] = 0;

  /* apply config */
  if (tcsetattr(app.ttyfd, TCSADRAIN, &app.term_settings) < 0) {
    err("could not configure device %s\n", strerror(errno));
    return errno;
  }
  return 0;
}

static void hal_sleep_ns(uint32_t ns)
{
  struct timespec sleep = {
      .tv_sec = 0,
      .tv_nsec = ns
  };
  nanosleep(&sleep, NULL);
}

static int _set_pin(int pindef, bsl_pinstate_t state) {
  if (state == BSL_PINSTATE_TRI) return 0;

  if (pindef == PIN_DTR || pindef == PIN_RTS) {
    int tiocm = pindef == PIN_DTR ? TIOCM_DTR : TIOCM_RTS;
    if (state == BSL_PINSTATE_HI)
      app.term_status &= ~tiocm;
    else if (state == BSL_PINSTATE_LO)
      app.term_status |= tiocm;
    if (ioctl(app.ttyfd, TIOCMSET, &app.term_status) < 0) {
      err("could not configure lines %s\n", strerror(errno));
      return errno;
    }
    return 0;
  }

  char path[256];
  sprintf(path, "/sys/class/gpio/gpio%d/value", pindef);
  int gpiofd = open(path, O_WRONLY);
  if (gpiofd < 0) {
    err("failed to open gpio %s, %s\n", path, strerror(errno));
    return errno;
  }
  char val = state == BSL_PINSTATE_HI ? '1':'0';
  int res = write(gpiofd, &val, 1);
  close(gpiofd);
  if (res < 0) {
    err("failed to set gpio %s, %s\n", path, strerror(errno));
    return errno;
  } else {
    res = 0;
  }
  return res;
}

static int hal_set_gpio(bsl_pin_t pin, bsl_pinstate_t state)
{
  if ((pin == BSL_PIN_RESET && arg_reset_pin_inv) ||
      (pin == BSL_PIN_TEST && arg_test_pin_inv)) {
    if (state == BSL_PINSTATE_HI) {
      state = BSL_PINSTATE_LO;
    } else if (state == BSL_PINSTATE_LO) {
      state = BSL_PINSTATE_HI;
    }
  }
  if (pin == BSL_PIN_RESET) {
    return _set_pin(arg_reset_pin, state);
  } else if (pin == BSL_PIN_TEST) {
    return _set_pin(arg_test_pin, state);
  }
  return -1;
}

static int hal_config(uint32_t rate)
{
  return config_uart(rate);
}

static int hal_rd(uint8_t *buf, uint16_t len)
{
  while (len && run) {
    int res = read(app.ttyfd, buf, len);
    if (res < 0) {
      err("read error %s\n", strerror(res));
      return res;
    }
    if (res == 0) {
      err("read timeout\n");
      return ETIMEDOUT;
    }
    len -= res;
    buf += res;
  }
  if (!run) {
    err("read aborted\n");
    return ECANCELED;
  }
  return 0;
}

static int hal_wr(const uint8_t *buf, uint16_t len)
{
  while (len && run) {
    int res = write(app.ttyfd, buf, len);
    if (res < 0) {
      err("write error %s\n", strerror(res));
      return res;
    }
    if (res == 0) {
      err("write timeout\n");
      return ETIMEDOUT;
    }
    len -= res;
    buf += res;
  }
  if (!run) {
    err("write aborted\n");
    return ECANCELED;
  }
  return 0;
}

static void app_shutdown(int ignore)
{
  (void)ignore;
  run = 0;
}

static const cli_input_def_t switch_defs[];
static const cli_input_def_t command_defs[];

static int parse_args(int argc, char *argv[], int switches)
{
  int res = 0;
  int argix = 1;

  while (res == 0 && argix < argc) {
    char *arg = argv[argix];

    if (arg[0] == '-') {
      if (switches) dbg("switch parsing %s\n", arg);
      const cli_input_def_t *d = switch_defs;
      int found = 0;
      while (d->fn) {
        int match_sname = (d->shortname && strcmp(&arg[1], d->shortname) == 0);
        int match_name = (arg[1] == '-' && d->name && strncmp(&arg[2], d->name, strlen(d->name)) == 0);
        if (match_sname || match_name) {
          found = 1;
          char *argarg = NULL;
          if (match_sname) {
            if (d->arg == ARG) {
              if (argix >= argc-1) {
                out("error: %s is expecting argument\n", arg);
                return EINVAL;
              }
              argarg = argv[++argix];
            }
          }
          else if (match_name) {
            char *eq = index(arg, '=');
            if (eq == NULL && d->arg == ARG) {
              out("error: %s is expecting argument\n", arg);
              return EINVAL;
            }
            argarg = eq ? ++eq : NULL;
          }
          if (switches) res = d->fn(arg, argarg);
          break;
        }
        d++;

      } // per argdef
      if (!found) {
        out("error: unknown switch %s\n", arg);
        return EINVAL;
      }
    } // if arg[0] == -
    else
    {
      if (!switches) dbg("command parsing %s\n", arg);
      const cli_input_def_t *d = command_defs;
      int found = 0;
      while (d->fn) {
        int match_sname = (d->shortname && strcmp(arg, d->shortname) == 0);
        int match_name = (d->name && strncmp(arg, d->name, strlen(d->name)) == 0);
        if (match_sname || match_name) {
          found = 1;
          char *argarg = NULL;
          if (d->arg == ARG) {
            if (argix >= argc-1) {
              out("error: %s is expecting argument\n", arg);
              return EINVAL;
            }
            argarg = argv[++argix];
          }
          if (!switches) res = d->fn(arg, argarg);
          break;
        }
        d++;
      } // per argdef
      if (!found) {
        out("error: unknown command %s\n", arg);
        return EINVAL;
      }
    }
    argix++;
  } // per input arg
  return res;
}

static int find_arg_num(const char *args, uint32_t arg, const char **start, uint32_t *len)
{
  const char *a = args;
  if (args == NULL) return -1;
  while (arg) {
    a = index(a, ',');
    if (a == NULL)
      return -1;
    else
      a++;
    arg--;
  }
  if (*a == 0) return -1;
  *start = a;
  a = index(a, ',');
  if (a == NULL) {
    *len = strlen(args) - (intptr_t)(*start-args);
  } else {
    *len = (intptr_t)a - (intptr_t)*start;
  }
  return 0;
}

static int parse_int(const char *args, uint32_t arg, uint32_t *argint)
{
  const char *start;
  uint32_t len;
  int res = find_arg_num(args, arg, &start, &len);
  if (res) return res;
  char b[256];
  strncpy(b, start, sizeof(b));
  *argint = strtol(b, NULL, 0);
  return 0;
}

static int parse_str(const char *args, uint32_t arg, const char **argstr)
{
  const char *start;
  uint32_t len;
  int res = find_arg_num(args, arg, &start, &len);
  if (res) return res;
  *argstr = start;
  return len;
}

static const char *strerrorapp(int res)
{
  if (res > 0) {
    return strerror(res);
  } else if (res < 0) {
    switch (res) {
    case BSL_ERR:                     return "General error";
    case BSL_ERR_NOT_SUPPORTED:       return "Not supported";
    case BSL_ERR_HDR_BAD:             return "Header bad";
    case BSL_ERR_PKT_TOO_LONG:        return "Packet too long";
    case BSL_ERR_LEN_BAD:             return "Length bad";
    case BSL_ERR_CHK_BAD:             return "Checksum bad";
    case BSL124_ERR_BAD_ACK:          return "Unknown acknowledgment from BSL";
    case BSL124_ERR_NACK:             return "NACK from BSL";
    case BSL124_ERR_PKT_TOO_SMALL:    return "Packet too small";
    case BSL124_ERR_PKT_NOT_EVEN:     return "Length not even";
    case BSL56_ERR_RX_CMD_BAD:        return "Unknown command";
    case BSL56_ERR_WRITE_CHECK_FAIL:  return "Write check fail";
    case BSL56_ERR_FLASH_FAIL_BIT:    return "Flash fail bit set";
    case BSL56_ERR_FLASH_VCHANGE:     return "Voltage change during flash";
    case BSL56_ERR_LOCKED:            return "Locked";
    case BSL56_ERR_ACCESS_DENIED:     return "Wrong password";
    case BSL56_ERR_FLASH_BYTE_FAIL:   return "Byte flashing prohibited";
    case BSL56_ERR_UNKNOWN_CMD:       return "Unknown command";

    case ERR_VERIFICATION_FAILED:     return "Verification failed";

    default:                          return "Unknown error";
    }
  } else {
    return "OK";
  }
}

int main(int argc, char *argv[])
{
  int res = -1;
  struct sigaction act;
  act.sa_handler = app_shutdown;
  sigaction(SIGINT, &act, NULL);

  memset(&app, 0, sizeof(app));

  strcpy(arg_uart_device, "/dev/ttyUSB0");
  res = parse_args(argc, argv, 1);
  if (res) goto out;

  recalc_erase_map();

  if (open_uart(arg_uart_device)) goto out;

  bsl_hal_t hal = {
      .sleep_fn = hal_sleep_ns,
      .gpio_set_fn = hal_set_gpio,
      .ser_config_fn = hal_config,
      .ser_read_fn = hal_rd,
      .ser_write_fn = hal_wr,
  };

  out("%s v%d.%d\n", argv[0], MSP430BSL_VERSION_API, MSP430BSL_VERSION_MINOR);
  out("%s, %s jtag pins, %s family\n",
      arg_uart_device,
      arg_dedicated_jtag_pins ? "dedicated" : "shared",
      arg_bsl_family == BSL_TYPE_124_FAMILIES ?
          "1xx/2xx/4xx" : "5xx/6xx");
  (void)hal;
  (void)arg_key;

  out("Connecting\n");
  if ((res = bsl_init(arg_bsl_family, &hal))) goto out;
  if ((res = bsl_enter(arg_dedicated_jtag_pins))) goto out;
  res = parse_args(argc, argv, 0);

out:
  if (app.ttyfd) close(app.ttyfd);
  if (erase_map) free(erase_map);
  if (res) {
    err("return code %d\n", res);
    out("%s\n", strerrorapp(res));
  }
  return res;
}

static int _unlock(void)
{
  int res = 0;
  if (locked) {
    out("Unlocking\n");
    if (__dbg > 1) {
      out("key:");
      for (uint32_t i = 0; i < sizeof(arg_key); i++) {
        out("%02x", arg_key[i]);
      }
      out("\n");
    }
    res = bsl_unlock(arg_key);
    if (res == 0) locked = 0;
    else err("unlock failed\n");
  }
  return res;
}

static int cli_arg_device(const char *n, const char *arg)
{
  strncpy(arg_uart_device, arg, sizeof(arg_uart_device));
  dbg("uart device %s\n", arg);
  return 0;
}

static int cli_arg_set_dedicated_jtag_pins(const char *n, const char *arg)
{
  if (ARGISYES(arg)) {
    dbg("dedicated jtag pins\n");
    arg_dedicated_jtag_pins = 1;
  } else if (ARGISNO(arg)) {
    dbg("shared jtag pins\n");
    arg_dedicated_jtag_pins = 0;
  } else {
    out("error: %s expecting 'yes' or 'no', not '%s'\n", n, arg);
    return EINVAL;
  }
  return 0;
}

static int cli_arg_family(const char *n, const char *arg)
{
  if (strcmp("124", arg) == 0) {
    dbg("1xx 2xx 4xx families\n");
    arg_bsl_family = BSL_TYPE_124_FAMILIES;
  } else if (strcmp("56", arg) == 0) {
    dbg("5xx 6xx families\n");
    arg_bsl_family = BSL_TYPE_56_FAMILIES;
  } else {
    out("error: %s expecting '124' or '56', not '%s'\n", n, arg);
    return EINVAL;
  }
  return 0;
}

static int _parse_pin(const char *n, const char *arg, int *set)
{
  if (strcmp("DTR", arg) == 0) {
    *set = PIN_DTR;
  } else if (strcmp("RTS", arg) == 0) {
    *set = PIN_RTS;
  } else if (sscanf(arg, "gpio%d", set) == 1) {
    // pass
  } else {
    out("error: %s expecting 'RTS', 'DTR', or 'gpio<num>', not '%s'\n", n, arg);
    return EINVAL;
  }
  return 0;
}

static int cli_arg_reset_pin(const char *n, const char *arg)
{
  return _parse_pin(n, arg, &arg_reset_pin);
}

static int cli_arg_test_pin(const char *n, const char *arg)
{
  return _parse_pin(n, arg, &arg_test_pin);
}

static int cli_arg_invert_reset_pin(const char *n, const char *arg)
{
  dbg("reset inverted\n");
  arg_reset_pin_inv = 1;
  return 0;
}

static int cli_arg_invert_test_pin(const char *n, const char *arg)
{
  dbg("test inverted\n");
  arg_test_pin_inv = 1;
  return 0;
}

static int cli_arg_key_string(const char *n, const char *arg)
{
  if (strlen(arg) != sizeof(arg_key)*2) {
    out("error: %s expects exactly %d hexadecimal characters as argument\n", n, (int)sizeof(arg_key)*2);
    return EINVAL;
  }

  dbg("key string %s\n", arg);
  for (uint32_t i = 0; i < sizeof(arg_key); i++) {
    uint32_t x;
    sscanf(&arg[i*2], "%02x", &x);
    arg_key[i] = x;
  }

  return 0;
}

static int set_key_image(const char *file)
{
  FILE *fd = fopen(file, "rb");
  if (fd == NULL) {
    out("error: could not open %s (%s)\n", file, strerror(errno));
    return errno;
  }

  fseek(fd, 0, SEEK_END);
  uint32_t fsize = (uint32_t)ftell(fd);
  if (fsize < sizeof(arg_key)) {
    out("error: file %s is too small to contain a key\n", file);
    fclose(fd);
    return EINVAL;
  }
  fseek(fd, -sizeof(arg_key), SEEK_END);
  int res = fread(arg_key, 1, sizeof(arg_key), fd);
  dbg("using %s (len %d) as image key file (read res:%d)\n", file, fsize, res);
  if (res < 0) {
    out("error: could not read %s (%s)\n", file, strerror(errno));
    fclose(fd);
    return errno;
  }
  dbg("key image %s\n", file);

  fclose(fd);

  return 0;
}

static int cli_arg_key_image(const char *n, const char *arg)
{
  const char *file = arg;
  if (strlen(file) == 0) {
    out("error: %s expecting a file as argument", n);
    return EINVAL;
  }
  return set_key_image(file);
}

static int cli_arg_sector_size(const char *n, const char *arg)
{
  arg_sector_size = strtol(arg, NULL, 0);
  // check sector size, and of 2 log
  if (arg_sector_size >= 64 && arg_sector_size <= 0x10000
      && (arg_sector_size & (arg_sector_size - 1)) == 0) {
    dbg("sector size %d\n", arg_sector_size);
    recalc_erase_map();
    return 0;
  }
  out("error: %s invalid sector size %s\n", n, arg);
  return EINVAL;
}

static int cli_arg_auto_erase(const char *n, const char *arg)
{
  dbg("auto erase on\n");
  arg_auto_erase = 1;
  return 0;
}

static int cli_arg_auto_verify(const char *n, const char *arg)
{
  dbg("auto verify on\n");
  arg_auto_verify = 1;
  return 0;
}

static int cli_arg_read_buf_size(const char *n, const char *arg)
{
  arg_read_buf = strtol(arg, NULL, 0);
  if (arg_read_buf >= 1) {
    dbg("read buf size %d\n", arg_read_buf);
    return 0;
  }
  out("error: %s invalid read buf size %s\n", n, arg);
  return EINVAL;
}

static int cli_arg_write_buf_size(const char *n, const char *arg)
{
  arg_write_buf = strtol(arg, NULL, 0);
  if (arg_write_buf >= 1) {
    dbg("write buf size %d\n", arg_write_buf);
    return 0;
  }
  out("error: %s invalid write buf size %s\n", n, arg);
  return EINVAL;
}

static int cli_arg_verbose(const char *n, const char *arg)
{
  __dbg = 2; return 0;
}
static int cli_arg_vverbose(const char *n, const char *arg)
{
  __dbg = 3; return 0;
}
extern int _msp_bsl_dbg;
static int cli_arg_vvverbose(const char *n, const char *arg)
{
  __dbg = 3; _msp_bsl_dbg = 2; return 0;
}
static int cli_arg_quiet(const char *n, const char *arg)
{
  __dbg = 0; return 0;
}

static int cli_arg_help(const char *n, const char *arg);

static int _erase(uint32_t addr, uint32_t size)
{
  int res = 0;
  while (size && res == 0) {
    uint32_t em_ix = (addr / arg_sector_size) / 32;
    uint32_t em_bix = (addr / arg_sector_size) % 32;
    if (erase_map[em_ix] & (1<<em_bix)) {
      dbg("  0x%08x (already erased)\n", addr);
    } else {
      out("  erase 0x%08x -- 0x%08x\n", addr, addr + arg_sector_size);
      res = bsl_erase_segment(addr);
      erase_map[em_ix] |= (1<<em_bix);
    }
    addr += arg_sector_size;
    size -= size < arg_sector_size ? size : arg_sector_size;
  }
  return res;
}

static int _write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  int res;
  if (arg_auto_erase) {
    res = _erase(addr, len);
    if (res) return res;
  }
  out("  write 0x%08x -- 0x%08x\n", addr, addr + len);
  res = bsl_write_mem(addr, buf, len);
  if (res) return res;
  if (arg_auto_verify) {
    uint8_t rbuf[len];
    res = bsl_read_mem(addr, rbuf, len);
    if (res) return res;
    for (uint32_t i = 0; i < len; i++) {
      if (rbuf[i] != buf[i]) {
        out("error: verification failed @ address %08x, expected 0x%02x but was 0x%02x\n", addr + i, buf[i], rbuf[i]);
        return ERR_VERIFICATION_FAILED;
      }
    }
  }
  return res;
}

static int cli_cmd_enter(const char *n, const char *arg)
{
  out("Forcing enter sequence\n");
  locked = 1;
  int res = bsl_enter(arg_dedicated_jtag_pins);
  return res;
}

static void _flush_rx(void)
{
  int res;
  uint8_t lost;
  uint32_t spoon_guard = 1000;
  do {
    res = read(app.ttyfd, &lost, 1);
  } while (res > 0 && --spoon_guard);
}

static int cli_cmd_unlock(const char *n, const char *arg)
{
  out("Forcing unlock\n");
  locked = 1;
  int res = _unlock();
  int tries = 2;
  while (res && arg_bsl_family == BSL_TYPE_56_FAMILIES && tries--) {
    dbg("failed, try enter bsl again\n");
    res = bsl_enter(arg_dedicated_jtag_pins);
    if (res) {
      err("could not reenter bsl\n");
      return res;
    }
    hal_sleep_ns(50*1000000UL);
    _flush_rx(); // get rid of garbage messing with the protocol
    res = _unlock();
  }
  return res;
}

static int cli_cmd_pause(const char *n, const char *arg)
{
  int res;
  uint32_t ms;
  res = parse_int(arg, 0, &ms);
  if (res) {
    out("error: %s needs number of milliseconds as first argument\n", n);
    return EINVAL;
  }

  out("Chill %d ms\n", ms);

  struct timespec sleep = {
      .tv_sec = ms/1000UL,
      .tv_nsec = (ms%1000UL) * 1000000UL
  };
  nanosleep(&sleep, NULL);

  return 0;
}

static int cli_cmd_read(const char *n, const char *arg)
{
  uint32_t addr, size;
  const char *argfile;
  char file[256];
  memset(file,0,sizeof(file));

  int res;
  res = parse_int(arg, 0, &addr);
  if (res) {
    out("error: %s needs an address as first argument\n", n);
    return EINVAL;
  }
  res = parse_int(arg, 1, &size);
  if (res) {
    out("error: %s needs a length as second argument\n", n);
    return EINVAL;
  }
  res = parse_str(arg, 2, &argfile);
  if (res < 0) {
    argfile = NULL;
  } else {
    strncpy(file, argfile, res);
  }
  res = 0;

  res = _unlock();
  if (res) return res;

  out("Reading memory 0x%08x, %d bytes", addr, size);
  if (argfile) {
    out(" to file %s", file);
  }
  out("\n");

  FILE *fd;

  if (!argfile) {
    fd = stdout;
  } else {
    fd = fopen(file, "a");
    if (fd == NULL) {
      out("error: could not create %s (%s)\n", file, strerror(errno));
      return errno;
    }
  }

  uint8_t buf[arg_read_buf];
  while (res == 0 && size) {
    uint32_t rlen = size < sizeof(buf) ? size : sizeof(buf);
    res = bsl_read_mem(addr, buf, rlen);
    if (res) break;
    fprintf(fd, "%08x: ", addr);
    for (uint32_t j = 0; j < rlen; j++) {
      fprintf(fd, "%02x ", buf[j]);
    }
    fprintf(fd, "\n");
    size -= rlen;
    addr += rlen;
  }

  if (fd && argfile) {
    fflush(fd);
    fclose(fd);
  }

  return res;
}

static int cli_cmd_dump(const char *n, const char *arg)
{
  uint32_t addr, size;
  const char *argfile;
  char file[256];
  memset(file,0,sizeof(file));

  int res;
  res = parse_int(arg, 0, &addr);
  if (res) {
    out("error: %s needs an address as first argument\n", n);
    return EINVAL;
  }
  res = parse_int(arg, 1, &size);
  if (res) {
    out("error: %s needs a length as second argument\n", n);
    return EINVAL;
  }
  res = parse_str(arg, 2, &argfile);
  if (res < 0) {
    argfile = NULL;
  } else {
    strncpy(file, argfile, res);
  }
  res = 0;

  res = _unlock();
  if (res) return res;

  out("Dumping memory 0x%08x, %d bytes to file %s\n", addr, size, file);

  FILE *fd;

  if (!argfile) {
    fd = stdout;
  } else {
    fd = fopen(file, "wb");
    if (fd == NULL) {
      out("error: could not create %s (%s)\n", file, strerror(errno));
      return errno;
    }
  }

  uint8_t buf[arg_read_buf];
  while (res == 0 && size) {
    uint32_t rlen = size < sizeof(buf) ? size : sizeof(buf);
    res = bsl_read_mem(addr, buf, rlen);
    if (res) break;
    res = fwrite(buf, 1, rlen, fd);
    if (res < 0) {
      res = errno;
    } else {
      res = 0;
    }
    fflush(fd);
    size -= rlen;
    addr += rlen;
  }

  if (fd && argfile) {
    fclose(fd);
  }

  return res;
}

static int cli_cmd_verify(const char *n, const char *arg)
{
  uint32_t addr;
  const char *argfile;
  char file[256];
  memset(file,0,sizeof(file));

  int res;

  res = parse_str(arg, 0, &argfile);
  if (res < 0) {
    out("error: %s needs a file as first argument\n", n);
    return EINVAL;
  }
  strncpy(file, argfile, res);
  res = 0;

  res = parse_int(arg, 1, &addr);
  if (res) {
    out("error: %s needs an address as second argument\n", n);
    return EINVAL;
  }

  res = set_key_image(file);
  if (res) return res;

  res = _unlock();
  if (res) return res;

  FILE *fd = fopen(file, "rb");
  if (fd == NULL) {
   out("error: could not open %s (%s)\n", file, strerror(errno));
   return errno;
  }

  fseek(fd, 0, SEEK_END);
  uint32_t size = (uint32_t)ftell(fd);
  fseek(fd, 0, SEEK_SET);

  out("Verifying %s (size:%d) 0x%08x--0x%08x\n", file, size, addr, addr+size);

  uint8_t buf[arg_read_buf];
  uint8_t fbuf[arg_read_buf];
  uint32_t offset = 0;
  while (res == 0 && size) {
    uint32_t rlen = size < sizeof(buf) ? size : sizeof(buf);
    res = bsl_read_mem(addr, buf, rlen);
    if (res) break;

    res = fread(fbuf, 1, rlen, fd);
    if (res < 0) {
      res = errno;
      goto end;
    }
    res = 0;

    out("  verify 0x%08x -- 0x%08x\n", addr, addr + rlen);
    for (uint32_t i = 0; i < rlen; i++) {
      if (buf[i] != fbuf[i]) {
        out("Data mismatch on address 0x%08x, file offset %d.\n", addr + i, offset + i);
        res = ERR_VERIFICATION_FAILED;
        goto end;
      }
    }

    addr += rlen;
    size -= rlen;
    offset += rlen;
  }

  if (res == 0) {
    out("Verification succeeded.\n");
  }

  end:
  fclose(fd);
  return res;
}


static int cli_cmd_write_bin(const char *n, const char *arg)
{
  uint32_t addr, offs, size;
  const char *argfile;
  char file[256];
  memset(file,0,sizeof(file));

  int res;

  res = parse_str(arg, 0, &argfile);
  if (res < 0) {
    out("error: %s needs a file as first argument\n", n);
    return EINVAL;
  }
  strncpy(file, argfile, res);
  res = 0;

  res = parse_int(arg, 1, &addr);
  if (res) {
    out("error: %s needs an address as second argument\n", n);
    return EINVAL;
  }
  res = parse_int(arg, 2, &offs);
  if (res) {
    offs = 0;
  }
  res = parse_int(arg, 3, &size);
  if (res) {
    size = 0xffffffff;
    res = 0;
  }

  FILE *fd = fopen(file, "rb");
  if (fd == NULL) {
    out("error: could not open %s (%s)\n", file, strerror(errno));
    return errno;
  }

  fseek(fd, 0, SEEK_END);
  uint32_t fsize = (uint32_t)ftell(fd);
  fseek(fd, offs, SEEK_SET);
  if (size == 0xffffffff) {
    size = fsize - offs;
  }
  if (offs > fsize) {
    size = 0;
  } else if (offs + size > fsize) {
    size = fsize - offs;
  }

  res = _unlock();
  if (res) return res;

  out("Writing file %s (offset:%d size:%d), to 0x%08x--0x%08x\n", file, offs, size, addr, addr+size);

  uint8_t buf[arg_write_buf];
  while (res == 0 && size) {
    uint32_t rlen = size < sizeof(buf) ? size : sizeof(buf);
    res = fread(buf, 1, rlen, fd);
    if (res < 0) {
      res = errno;
      goto end;
    }
    res = _write(addr, buf, rlen);
    addr += rlen;
    size -= rlen;
  }

  end:
  fclose(fd);
  return res;
}

static int cli_cmd_erase(const char *n, const char *arg)
{
  uint32_t addr, size;

  int res;
  res = parse_int(arg, 0, &addr);
  if (res) {
    out("error: %s needs an address as first argument\n", n);
    return EINVAL;
  }
  res = parse_int(arg, 1, &size);
  if (res) {
    out("error: %s needs a length as second argument\n", n);
    return EINVAL;
  }

  res = _unlock();
  if (res) return res;

  out("Erase memory 0x%08x -- 0x%08x (%d bytes), %d byte sized sectors\n", addr, addr+size, size, arg_sector_size);
  res = _erase(addr, size);
  return res;
}

static int cli_cmd_mass_erase(const char *n, const char *arg) {
  out("Mass erase\n");
  memset(erase_map, 0xff, erase_map_sz);
  return bsl_mass_erase();
}

static int cli_cmd_load_pc(const char *n, const char *arg)
{
  uint32_t addr;
  int res;
  res = parse_int(arg, 0, &addr);
  if (res) {
    out("error: %s needs an address as first argument\n", n);
    return EINVAL;
  }
  res = _unlock();
  if (res) return res;
  out("Load PC 0x%08x\n", addr);
  res = bsl_load_pc(addr);
  if (res == 0) locked = 1;
  return res;
}

static int cli_cmd_reset(const char *n, const char *arg)
{
  out("Reset\n");
  int res = bsl_reset();
  if (res == 0) locked = 1;
  return res;
}

static const cli_input_def_t switch_defs[] = {
    // connex
    {.name="device", .shortname="d",
        .fn=cli_arg_device,
        .arg=ARG, .arghelp="FILE",
        .help="Set the uart, e.g. /dev/ttyUSB0."},
    {.name="dedicated-jtag-pins", .shortname="p",
        .fn=cli_arg_set_dedicated_jtag_pins,
        .arg=ARG, .arghelp="<yes|no>",
        .help="Set whether the MSP430 has dedicated JTAG pins or not. Default is no."},
    {.name="bsl-family", .shortname="f",
        .fn=cli_arg_family,
        .arg=ARG, .arghelp="<124|56>",
        .help="Set the BSL family. Default is 124 (1xx,2xx,4xx chipsets)."},

    {.name="reset-pin", .shortname="rst",
        .fn=cli_arg_reset_pin,
        .arg=ARG, .arghelp="<DTR|RTS|gpioxx>",
        .help="Define the reset pin. Can either be uart DTR, RTS, or a gpio pin. In gpio case, /sys/class/gpio/gpioXX/value will be written. Default is DTR."},
    {.name="test-pin", .shortname="tst",
        .fn=cli_arg_test_pin,
        .arg=ARG, .arghelp="<DTR|RTS|gpioxx>",
        .help="Define the test pin. Can either be uart DTR, RTS, or a gpio pin. In gpio case, /sys/class/gpio/gpioXX/value will be written. Default is RTS."},

    {.name="inv-reset-pin", .shortname="irst",
        .fn=cli_arg_invert_reset_pin,
        .arg=NOARG,
        .help="Invert the reset pin signal."},
    {.name="inv-test-pin", .shortname="itst",
        .fn=cli_arg_invert_test_pin,
        .arg=NOARG,
        .help="Invert the test pin signal."},

    // key ops
    {.name="key", .shortname="k",
        .fn=cli_arg_key_string,
        .arg=ARG, .arghelp="<32 HEX VALUES>",
        .help="Enter unlock key, e.g. aa11cc0d..ff. Default key is ffff..ff."},
    {.name="key-image", .shortname="ki",
        .fn=cli_arg_key_image,
        .arg=ARG, .arghelp="<FILE>",
        .help="Unlock key is read from end of given file. I.e. if the file is the binary image currently written to the chip, the key will match."},

    // mem ops
    {.name="sector-size", .shortname="s",
        .fn=cli_arg_sector_size,
        .arg=ARG, .arghelp="<BYTES>",
        .help="Set sector size during erase. Default is " xstr(DEFAULT_SECTOR_SIZE) " bytes."},
    {.name="auto-erase", .shortname="ae",
        .fn=cli_arg_auto_erase,
        .arg=NOARG,
        .help="Enable auto erase before any writes."},
    {.name="auto-verify", .shortname="av",
        .fn=cli_arg_auto_verify,
        .arg=NOARG,
        .help="Enable auto verify after any writes."},

    {.name="read-buf", .shortname="rb",
        .fn=cli_arg_read_buf_size,
        .arg=ARG, .arghelp="<BYTES>",
        .help="Set max read buffer size. Default is " xstr(DEFAULT_READ_BUF) " bytes."},
    {.name="write-buf", .shortname="wb",
        .fn=cli_arg_write_buf_size,
        .arg=ARG, .arghelp="<BYTES>",
        .help="Set max write buffer size. Default is " xstr(DEFAULT_WRITE_BUF) " bytes."},

    // dbg
    {.name="quiet", .shortname="q",
        .fn=cli_arg_quiet,
        .arg=NOARG,
        .help="Disable all output."},
    {.name="verbose", .shortname="v",
        .fn=cli_arg_verbose,
        .arg=NOARG,
        .help="Set verbose output."},
    {.name="very-verbose", .shortname="vv",
        .fn=cli_arg_vverbose,
        .arg=NOARG,
        .help="Enable debug output."},
    {.name="violently-very-verbose", .shortname="vvv",
        .fn=cli_arg_vvverbose,
        .arg=NOARG,
        .help="Enable debug output, also for the BSL communication."},
    {.name="help",
        .fn=cli_arg_help,
        .arg=NOARG,
        .help="Prints help."},
    {.fn=NULL}
};

static const cli_input_def_t command_defs[] = {
    // access ops
    {.name="enter",
        .fn=cli_cmd_enter,
        .arg=NOARG,
        .help="Performs the initiation dance on RESET and TEST pins again. This is done automatically at startup."},
    {.name="unlock",
        .fn=cli_cmd_unlock,
        .arg=NOARG,
        .help="Unlocks the processor. If no key is defined in switches, default ffff..ff is used."},
    {.name="pause",
        .fn=cli_cmd_pause,
        .arg=ARG, .arghelp="MS",
        .help="Pauses the command parsing for given milliseconds."},

    // read mem ops
    {.name="read",
        .fn=cli_cmd_read,
        .arg=ARG, .arghelp="ADDR,SIZE(,FILE)",
        .help="Read memory. If FILE is given, output is dumped to file."},
    {.name="dump",
        .fn=cli_cmd_dump,
        .arg=ARG, .arghelp="ADDR,SIZE(,FILE)",
        .help="Read memory binary. If FILE is given, output is dumped to file."},
    {.name="verify",
        .fn=cli_cmd_verify,
        .arg=ARG, .arghelp="FILE,ADDR",
        .help="Verifies image against chip contents. This is supposed to be a full image, the last 32 bytes are used as key for unlocking the chip."},

    // write mem ops
    {.name="write-bin",
        .fn=cli_cmd_write_bin,
        .arg=ARG, .arghelp="FILE,ADDR(,OFFS(,SIZE))",
        .help="Write binary file to memory at given address, with optional file offset and size. If optional arguments are omitted, all file is written."},
#if 0
    {.name="write-hex", //TODO
        .fn=cli_arg_help,
        .arg=ARG, .arghelp="FILE",
        .help="Write intel hex file to memory."},
#endif
    {.name="erase",
        .fn=cli_cmd_erase,
        .arg=ARG, .arghelp="ADDR,SIZE",
        .help="Erase memory at given range."},
    {.name="mass-erase",
        .fn=cli_cmd_mass_erase,
        .arg=NOARG,
        .help="Mass erase."},

    // other ops
    {.name="load-pc",
        .fn=cli_cmd_load_pc,
        .arg=ARG, .arghelp="ADDR",
        .help="Set pc to given address."},
    {.name="reset",
        .fn=cli_cmd_reset,
        .arg=NOARG,
        .help="Reset processor."},

    {.fn=NULL}
};

static void print_wrapped(const char *str, uint32_t max_len, uint32_t initial_tab, uint32_t following_tabs)
{
  uint32_t i = 0;
  uint32_t last_break = 0;
  uint32_t last_wrap = 0;
  int first_line = 1;
  while (1) {
    if (i - last_wrap >= max_len || str[i] == 0) {
      if (first_line) for (uint32_t j=0; j<initial_tab; j++)printf(" ");
      else            for (uint32_t j=0; j<following_tabs; j++)printf(" ");
      first_line = 0;
      if (last_break && str[i] != 0) {
        printf("%.*s\n", last_break - last_wrap, &str[last_wrap]);
        last_wrap = last_break;
        last_break = 0;
      } else {
        printf("%.*s\n", i - last_wrap, &str[last_wrap]);
        last_wrap = i;
        last_break = 0;
      }
    }
    if (str[i] == ' ' || str[i] == ',' || str[i] == '.' || str[i] == ';') {
      last_break = i+1;
    } else if (str[i] == 0) {
      break;
    }
    i++;
  }
}

static void dump_input_def(int sw, const cli_input_def_t *d, uint32_t defs)
{
  while (defs--) {
    char txt[256];
    char *a = txt;
    a += sprintf(a, "  ");

    if (d->shortname) a += sprintf(a, "%s%s", sw ? "-" : "",  d->shortname);
    if (d->shortname && d->name) a += sprintf(a, ", ");
    if (d->name) a += sprintf(a, "%s%s", sw ? "--" : "", d->name);
    if (d->arg != NOARG && d->arghelp) {
      if (d->name)
        a += sprintf(a, sw ? "=" : " ");
      else
        a += sprintf(a, " ");
      if (d->arg == OPTARG) a += sprintf(a, "(");
      a += sprintf(a, "%s", d->arghelp);
      if (d->arg == OPTARG) a += sprintf(a, ")");
    }
    printf("%s", txt);
    int l = strlen(txt);
    int r = 30;
    if (l < 30) {
      r = 30 - l;
    } else {
      printf("\n");
    }
    if (d->help) print_wrapped(d->help, 50, r, 30);
    d++;
  }
}

static int cli_arg_help(const char *n, const char *arg)
{
  printf("\nSwitches:\n");
  dump_input_def(1, switch_defs, sizeof(switch_defs)/sizeof(switch_defs[0]));
  printf("\nCommands:\n");
  dump_input_def(0, command_defs, sizeof(command_defs)/sizeof(command_defs[0]));
  printf("\n");
  return 0;
}
