/*
 * camera.c  —  NuttX camera task
 *
 * Sequence:
 *   1. Open UART + SPI
 *   2. Handshake with OpenMV over UART
 *   3. Send CMD_CAPTURE
 *   4. Receive ACK + JPEG size (4B LE) + meta_len (2B LE) + meta string
 *   5. Wait SPI_CS_DELAY_US for OpenMV to enter spi.send_recv()
 *   6. Assert SPI CS, discard SPI_SYNC_BYTES, read JPEG
 *   7. Push metadata chunk (is_meta=1) then JPEG chunks into ring buffer
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <nuttx/spi/spi.h>

struct spi_dev_s;
extern struct spi_dev_s *stm32_spibus_initialize(int bus);

#include "ring_buffer.h"

/* ================================================================== */
/*  Hardware                                                           */
/* ================================================================== */
#define UART_DEV   "/dev/ttyS1"
#define SPI_BUS    3
#define SPI_FREQ   4000000
#define SPI_MODE   SPIDEV_MODE0

/* ================================================================== */
/*  Protocol                                                           */
/* ================================================================== */
#define CMD_HANDSHAKE  0x01
#define CMD_CAPTURE    0x02
#define ACK_BYTE       0xAA

/* ================================================================== */
/*  Sizes  — MUST match openmv_camera.py                              */
/* ================================================================== */
#define IMG_BUF_SIZE   (40000u)
#define SPI_XFER_SIZE  CHUNK_SIZE          /* 256 — one SPI block     */
#define SPI_SYNC_BYTES (1024u)             /* zero bytes before JPEG  */
#define META_BUF_SIZE  (128u)

/* ================================================================== */
/*  Timeouts                                                           */
/* ================================================================== */
#define HANDSHAKE_TRIES   5
#define HANDSHAKE_POLLS  50     /* × 100 ms = 5 s total               */
#define ACK_POLLS       300     /* × 10  ms = 3 s total               */
#define ACK_POLL_US   10000
#define SIZE_POLLS       50     /* × 10  ms = 500 ms total            */
#define SIZE_POLL_US  10000

/*
 * SPI_CS_DELAY_US  — gap between finishing UART reads and asserting
 * SPI CS.  OpenMV needs ~20-30 ms to finish writing UART bytes and
 * enter spi.send_recv().  Set to 25 ms (must be < OpenMV pyb.delay).
 */
#define SPI_CS_DELAY_US  25000u

/* ================================================================== */
/*  Static buffers                                                     */
/* ================================================================== */
static uint8_t image_buf[IMG_BUF_SIZE];
static uint8_t spi_rx[SPI_XFER_SIZE];
static uint8_t spi_tx[SPI_XFER_SIZE];   /* all-zero dummy TX          */
static char    meta_buf[META_BUF_SIZE];

/* ================================================================== */
/*  UART helpers                                                       */
/* ================================================================== */

static void uart_write_byte(int fd, uint8_t v)
{
  write(fd, &v, 1);
}

static int uart_read_byte(int fd, uint8_t *v, int retries, int delay_us)
{
  for (int t = 0; t < retries; t++)
    {
      if (read(fd, v, 1) == 1) return 0;
      if (delay_us > 0) usleep(delay_us);
    }
  return -1;
}

/* Read exactly `len` bytes; returns 0 on success, -1 on timeout */
static int uart_read_exact(int fd, uint8_t *buf, int len,
                            int retries, int delay_us)
{
  for (int i = 0; i < len; i++)
    {
      if (uart_read_byte(fd, &buf[i], retries, delay_us) < 0)
        {
          printf("[CAM] uart_read_exact timeout at byte %d/%d\n", i, len);
          return -1;
        }
    }
  return 0;
}

static void uart_flush(int fd)
{
  uint8_t d;
  while (read(fd, &d, 1) == 1);
  tcflush(fd, TCIFLUSH);
}

/* ================================================================== */
/*  UART configuration  — 115200 8N1 non-blocking                     */
/* ================================================================== */

static void uart_configure(int fd)
{
  struct termios t;
  tcgetattr(fd, &t);
  cfsetispeed(&t, B115200);
  cfsetospeed(&t, B115200);
  t.c_cflag  = CS8 | CREAD | CLOCAL;
  t.c_iflag  = 0;
  t.c_oflag  = 0;
  t.c_lflag  = 0;
  t.c_cc[VMIN]  = 0;
  t.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &t);
  printf("[CAM][UART] 115200 8N1 non-blocking\n");
}

/* ================================================================== */
/*  Handshake                                                          */
/* ================================================================== */

static int uart_handshake(int fd)
{
  uint8_t rx;

  printf("[CAM][UART] handshake start\n");

  for (int attempt = 0; attempt < HANDSHAKE_TRIES; attempt++)
    {
      uart_flush(fd);
      uart_write_byte(fd, CMD_HANDSHAKE);
      printf("[CAM] TX handshake attempt %d\n", attempt + 1);

      for (int p = 0; p < HANDSHAKE_POLLS; p++)
        {
          if (uart_read_byte(fd, &rx, 1, 0) == 0 && rx == ACK_BYTE)
            {
              uart_flush(fd);
              printf("[CAM][UART] HANDSHAKE OK\n");
              return 0;
            }
          usleep(100000);  /* 100 ms poll interval */
        }
    }

  printf("[CAM][UART] HANDSHAKE FAILED\n");
  return -1;
}

/* ================================================================== */
/*  SPI image transfer                                                 */
/*                                                                     */
/*  OpenMV sends: [SPI_SYNC_BYTES × 0x00] [JPEG padded to CHUNK]     */
/*  We:  discard sync, then copy JPEG into image_buf                  */
/* ================================================================== */

static int spi_read_image(struct spi_dev_s *spi_dev,
                           uint8_t *buf, uint32_t img_size)
{
  uint32_t remaining;
  uint32_t offset;

  memset(spi_tx, 0x00, SPI_XFER_SIZE);  /* dummy TX is always zero */

  SPI_LOCK(spi_dev, true);
  SPI_SETMODE(spi_dev, SPI_MODE);
  SPI_SETBITS(spi_dev, 8);
  SPI_SETFREQUENCY(spi_dev, SPI_FREQ);
  SPI_SELECT(spi_dev, 0, true);

  /* -- Discard sync bytes (1024) -- */
  remaining = SPI_SYNC_BYTES;
  while (remaining > 0)
    {
      uint32_t xfer = (remaining < SPI_XFER_SIZE)
                       ? remaining : SPI_XFER_SIZE;
      memset(spi_rx, 0, SPI_XFER_SIZE);
      SPI_EXCHANGE(spi_dev, spi_tx, spi_rx, xfer);
      remaining -= xfer;
    }
  printf("[CAM][SPI] %u sync bytes discarded\n", SPI_SYNC_BYTES);

  /* -- Read JPEG data -- */
  remaining = img_size;
  offset    = 0;
  while (remaining > 0)
    {
      uint32_t xfer = (remaining < SPI_XFER_SIZE)
                       ? remaining : SPI_XFER_SIZE;
      memset(spi_rx, 0, SPI_XFER_SIZE);
      SPI_EXCHANGE(spi_dev, spi_tx, spi_rx, xfer);
      memcpy(&buf[offset], spi_rx, xfer);
      offset    += xfer;
      remaining -= xfer;
    }

  SPI_SELECT(spi_dev, 0, false);
  SPI_LOCK(spi_dev, false);

  return 0;
}

/* ================================================================== */
/*  Push one chunk into ring buffer — blocks until slot available     */
/* ================================================================== */

static void rb_push(const struct image_chunk_s *c)
{
  while (rb_write(c) != 0)
    usleep(500);
}

/* ================================================================== */
/*  Entry point                                                        */
/* ================================================================== */

int camera_MSN2_main(int argc, char *argv[])
{
  printf("\n================ CAMERA START ================\n");

  rb_init();

  /* ---- Open UART ---- */
  int uart_fd = open(UART_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (uart_fd < 0)
    {
      printf("[CAM] UART open failed: %s\n", strerror(errno));
      return -1;
    }
  uart_configure(uart_fd);
  uart_flush(uart_fd);

  /* ---- Init SPI ---- */
  struct spi_dev_s *spi = stm32_spibus_initialize(SPI_BUS);
  if (!spi)
    {
      printf("[CAM] SPI init failed\n");
      close(uart_fd);
      return -1;
    }
  printf("[CAM] SPI%d ready\n", SPI_BUS);

  /* ---- Handshake ---- */
  if (uart_handshake(uart_fd) < 0)
    {
      close(uart_fd);
      return -1;
    }

  /* ==============================================================
   *  SINGLE CAPTURE
   * ============================================================== */
  printf("\n========== CAPTURE START ==========\n");

  /* 1. Send CMD_CAPTURE */
  uart_flush(uart_fd);
  uart_write_byte(uart_fd, CMD_CAPTURE);
  printf("[CAM] CMD_CAPTURE sent — waiting ACK...\n");

  /* 2. Wait for ACK */
  uint8_t ack = 0;
  if (uart_read_byte(uart_fd, &ack, ACK_POLLS, ACK_POLL_US) < 0
      || ack != ACK_BYTE)
    {
      printf("[CAM] ACK failed (got 0x%02X)\n", ack);
      close(uart_fd);
      return -1;
    }
  printf("[CAM] ACK OK\n");

  /* 3. JPEG size — 4 bytes little-endian */
  uint8_t sz[4] = {0};
  if (uart_read_exact(uart_fd, sz, 4, SIZE_POLLS, SIZE_POLL_US) < 0)
    {
      printf("[CAM] JPEG size read failed\n");
      close(uart_fd);
      return -1;
    }

  uint32_t img_size = (uint32_t)sz[0]
                    | ((uint32_t)sz[1] <<  8)
                    | ((uint32_t)sz[2] << 16)
                    | ((uint32_t)sz[3] << 24);

  printf("[CAM] JPEG size = %lu B\n", (unsigned long)img_size);

  if (img_size == 0 || img_size > IMG_BUF_SIZE)
    {
      printf("[CAM] Bad JPEG size — abort\n");
      close(uart_fd);
      return -1;
    }

  /* 4. Metadata length — 2 bytes little-endian */
  uint8_t ml[2] = {0};
  if (uart_read_exact(uart_fd, ml, 2, SIZE_POLLS, SIZE_POLL_US) < 0)
    {
      printf("[CAM] meta length read failed\n");
      close(uart_fd);
      return -1;
    }

  uint16_t meta_len = (uint16_t)ml[0] | ((uint16_t)ml[1] << 8);
  printf("[CAM] meta len  = %u B\n", meta_len);

  if (meta_len == 0 || meta_len >= META_BUF_SIZE)
    {
      printf("[CAM] meta length invalid (%u) — abort\n", meta_len);
      close(uart_fd);
      return -1;
    }

  /* 5. Metadata string */
  memset(meta_buf, 0, META_BUF_SIZE);
  if (uart_read_exact(uart_fd, (uint8_t *)meta_buf,
                      (int)meta_len, SIZE_POLLS, SIZE_POLL_US) < 0)
    {
      printf("[CAM] meta read failed\n");
      close(uart_fd);
      return -1;
    }
  printf("[CAM] meta      = \"%s\"\n", meta_buf);

  /* 6. Delay so OpenMV finishes writing UART and enters send_recv().
   *    OpenMV does pyb.delay(30) before its first SPI block.
   *    We wait 25 ms — less than OpenMV's delay so CS arrives while
   *    OpenMV is already sitting in spi.send_recv().               */
  printf("[CAM] waiting %lu ms before SPI CS...\n",
         (unsigned long)(SPI_CS_DELAY_US / 1000));
  usleep(SPI_CS_DELAY_US);

  /* 7. SPI: read sync + JPEG */
  printf("[CAM][SPI] reading %lu JPEG bytes "
         "(+ %u sync discarded)...\n",
         (unsigned long)img_size, SPI_SYNC_BYTES);

  memset(image_buf, 0, img_size);
  spi_read_image(spi, image_buf, img_size);

  /* Validate JPEG markers */
  int hdr_ok = (image_buf[0] == 0xFF &&
                image_buf[1] == 0xD8 &&
                image_buf[2] == 0xFF);
  int trl_ok = (img_size >= 2 &&
                image_buf[img_size - 2] == 0xFF &&
                image_buf[img_size - 1] == 0xD9);

  if (hdr_ok)
    printf("[CAM] JPEG header OK  : FF D8 FF\n");
  else
    printf("[CAM] WARN bad header : %02X %02X %02X\n",
           image_buf[0], image_buf[1], image_buf[2]);

  if (trl_ok)
    printf("[CAM] JPEG trailer OK : FF D9\n");
  else
    printf("[CAM] WARN bad trailer: %02X %02X\n",
           image_buf[img_size - 2], image_buf[img_size - 1]);

  /* 8. Push metadata chunk (is_meta = 1) */
  {
    struct image_chunk_s mc;
    memset(&mc, 0, sizeof(mc));
    memcpy(mc.data, meta_buf, meta_len);
    mc.len      = meta_len;
    mc.frame_id = 1;
    mc.is_last  = 0;
    mc.is_meta  = 1;   /* dedicated flag — no payload corruption */
    rb_push(&mc);
    printf("[CAM] meta chunk pushed\n");
  }

  /* 9. Push JPEG data chunks */
  uint32_t total = (img_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
  printf("[CAM] pushing %lu JPEG chunks...\n", (unsigned long)total);

  for (uint32_t ci = 0; ci < total; ci++)
    {
      uint32_t off = ci * CHUNK_SIZE;
      uint32_t len = img_size - off;
      if (len > CHUNK_SIZE) len = CHUNK_SIZE;

      struct image_chunk_s dc;
      memset(&dc, 0, sizeof(dc));
      memcpy(dc.data, &image_buf[off], len);
      dc.len      = (uint16_t)len;
      dc.frame_id = 1;
      dc.is_last  = (ci == total - 1) ? 1 : 0;
      dc.is_meta  = 0;
      rb_push(&dc);
    }

  printf("[CAM] all %lu chunks pushed — DONE\n", (unsigned long)total);

  close(uart_fd);
  return 0;
}