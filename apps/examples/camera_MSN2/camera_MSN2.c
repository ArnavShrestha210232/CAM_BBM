#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>

#include <nuttx/spi/spi.h>

struct spi_dev_s;
extern struct spi_dev_s *stm32_spibus_initialize(int bus);

#include "ring_buffer.h"
#include <semaphore.h>

static sem_t g_subscriber_done_sem;

#define SPI_FREQ        4000000
#define SPI_MODE        SPIDEV_MODE0

#define CMD_HANDSHAKE   0x01
#define CMD_CAPTURE     0x02
#define ACK_BYTE        0xAA

#define IMG_BUF_SIZE    (16000u)
#define SPI_XFER_SIZE   CHUNK_SIZE
#define SPI_SYNC_BYTES  (1024u)
#define META_BUF_SIZE   (128u)

#define HANDSHAKE_TRIES 5
#define HANDSHAKE_POLLS 50
#define ACK_POLLS       300
#define ACK_POLL_US     10000
#define SIZE_POLLS      50
#define SIZE_POLL_US    10000

#define SPI_CS_DELAY_US 25000u

static pthread_mutex_t g_rb_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    int camera_id;
    const char *uart_dev;
    int spi_bus;

    uint8_t *image_buf;
    uint8_t spi_rx[SPI_XFER_SIZE];
    uint8_t spi_tx[SPI_XFER_SIZE];
    char meta_buf[META_BUF_SIZE];
} camera_ctx_t;

static void rb_push(int cam_index, const struct image_chunk_s *c)
{
    while (1)
    {
        pthread_mutex_lock(&g_rb_mutex);
        int res = rb_write(cam_index,c);
        pthread_mutex_unlock(&g_rb_mutex);

        if (res == 0)
        {
            break;
        }
        usleep(500);
    }
}

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

static int uart_read_exact(int id, int fd, uint8_t *buf, int len,
                           int retries, int delay_us)
{
    for (int i = 0; i < len; i++)
    {
        if (uart_read_byte(fd, &buf[i], retries, delay_us) < 0)
        {
            fprintf(stderr, "[CAM %d] uart_read_exact timeout at byte %d/%d\n", id, i, len);
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

static void uart_configure(int id, int fd)
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
    fprintf(stderr, "[CAM %d][UART] 115200 8N1 non-blocking\n", id);
}

static int uart_handshake(int id, int fd)
{
    uint8_t rx;

    fprintf(stderr, "[CAM %d][UART] handshake start\n", id);

    for (int attempt = 0; attempt < HANDSHAKE_TRIES; attempt++)
    {
        uart_flush(fd);
        uart_write_byte(fd, CMD_HANDSHAKE);
        fprintf(stderr, "[CAM %d] TX handshake attempt %d\n", id, attempt + 1);

        for (int p = 0; p < HANDSHAKE_POLLS; p++)
        {
            if (uart_read_byte(fd, &rx, 1, 0) == 0 && rx == ACK_BYTE)
            {
                uart_flush(fd);
                fprintf(stderr, "[CAM %d][UART] HANDSHAKE OK\n", id);
                return 0;
            }
            usleep(100000);
        }
    }

    fprintf(stderr, "[CAM %d][UART] HANDSHAKE FAILED\n", id);
    return -1;
}

static int spi_read_image(camera_ctx_t *ctx, struct spi_dev_s *spi_dev, uint32_t img_size)
{
    uint32_t remaining;
    uint32_t offset;

    memset(ctx->spi_tx, 0x00, SPI_XFER_SIZE);

    SPI_LOCK(spi_dev, true);
    SPI_SETMODE(spi_dev, SPI_MODE);
    SPI_SETBITS(spi_dev, 8);
    SPI_SETFREQUENCY(spi_dev, SPI_FREQ);
    SPI_SELECT(spi_dev, 0, true);

    remaining = SPI_SYNC_BYTES;
    while (remaining > 0)
    {
        uint32_t xfer = (remaining < SPI_XFER_SIZE) ? remaining : SPI_XFER_SIZE;
        memset(ctx->spi_rx, 0, SPI_XFER_SIZE);
        SPI_EXCHANGE(spi_dev, ctx->spi_tx, ctx->spi_rx, xfer);
        remaining -= xfer;
    }
    fprintf(stderr, "[CAM %d][SPI] %u sync bytes discarded\n", ctx->camera_id, SPI_SYNC_BYTES);

    remaining = img_size;
    offset    = 0;
    while (remaining > 0)
    {
        uint32_t xfer = (remaining < SPI_XFER_SIZE) ? remaining : SPI_XFER_SIZE;
        memset(ctx->spi_rx, 0, SPI_XFER_SIZE);
        SPI_EXCHANGE(spi_dev, ctx->spi_tx, ctx->spi_rx, xfer);
        memcpy(&ctx->image_buf[offset], ctx->spi_rx, xfer);
        offset    += xfer;
        remaining -= xfer;
    }

    SPI_SELECT(spi_dev, 0, false);
    SPI_LOCK(spi_dev, false);

    return 0;
}
static uint8_t g_cam1_img_buf[IMG_BUF_SIZE];
static uint8_t g_cam2_img_buf[IMG_BUF_SIZE];
static void *camera_worker_thread(void *arg)
{
    camera_ctx_t *ctx = (camera_ctx_t *)arg;
    int id = ctx->camera_id;
    int cam_idx= id-1; /* C0 based array index for ring buffer */

    fprintf(stderr, "\n[CAM %d] Worker thread started\n", id);

    // ctx->image_buf = (uint8_t *)malloc(IMG_BUF_SIZE);
    // if (!ctx->image_buf)
    // {
    //     fprintf(stderr, "[CAM %d] Memory allocation failed\n", id);
    //     return NULL;
    // }

    int uart_fd = open(ctx->uart_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0)
    {
        fprintf(stderr, "[CAM %d] UART open failed (%s): %s\n", id, ctx->uart_dev, strerror(errno));
        free(ctx->image_buf);
        return NULL;
    }
    uart_configure(id, uart_fd);
    uart_flush(uart_fd);

    struct spi_dev_s *spi = stm32_spibus_initialize(ctx->spi_bus);
    if (!spi)
    {
        fprintf(stderr, "[CAM %d] SPI init failed (bus %d)\n", id, ctx->spi_bus);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }
    fprintf(stderr, "[CAM %d] SPI%d ready\n", id, ctx->spi_bus);

    if (uart_handshake(id, uart_fd) < 0)
    {
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }

    fprintf(stderr, "\n[CAM %d] ========== CAPTURE START ==========\n", id);

    uart_flush(uart_fd);
    uart_write_byte(uart_fd, CMD_CAPTURE);
    fprintf(stderr, "[CAM %d] CMD_CAPTURE sent — waiting ACK...\n", id);

    uint8_t ack = 0;
    if (uart_read_byte(uart_fd, &ack, ACK_POLLS, ACK_POLL_US) < 0 || ack != ACK_BYTE)
    {
        fprintf(stderr, "[CAM %d] ACK failed (got 0x%02X)\n", id, ack);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }
    fprintf(stderr, "[CAM %d] ACK OK\n", id);

    uint8_t sz[4] = {0};
    if (uart_read_exact(id, uart_fd, sz, 4, SIZE_POLLS, SIZE_POLL_US) < 0)
    {
        fprintf(stderr, "[CAM %d] JPEG size read failed\n", id);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }

    uint32_t img_size = (uint32_t)sz[0]
                      | ((uint32_t)sz[1] <<  8)
                      | ((uint32_t)sz[2] << 16)
                      | ((uint32_t)sz[3] << 24);

    fprintf(stderr, "[CAM %d] JPEG size = %lu B\n", id, (unsigned long)img_size);

    if (img_size == 0 || img_size > IMG_BUF_SIZE)
    {
        fprintf(stderr, "[CAM %d] Bad JPEG size — abort\n", id);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }

    uint8_t ml[2] = {0};
    if (uart_read_exact(id, uart_fd, ml, 2, SIZE_POLLS, SIZE_POLL_US) < 0)
    {
        fprintf(stderr, "[CAM %d] meta length read failed\n", id);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }

    uint16_t meta_len = (uint16_t)ml[0] | ((uint16_t)ml[1] << 8);
    fprintf(stderr, "[CAM %d] meta len  = %u B\n", id, meta_len);

    if (meta_len == 0 || meta_len >= META_BUF_SIZE)
    {
        fprintf(stderr, "[CAM %d] meta length invalid (%u) — abort\n", id, meta_len);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }

    memset(ctx->meta_buf, 0, META_BUF_SIZE);
    if (uart_read_exact(id, uart_fd, (uint8_t *)ctx->meta_buf,
                        (int)meta_len, SIZE_POLLS, SIZE_POLL_US) < 0)
    {
        fprintf(stderr, "[CAM %d] meta read failed\n", id);
        close(uart_fd);
        free(ctx->image_buf);
        return NULL;
    }
    fprintf(stderr, "[CAM %d] meta      = \"%s\"\n", id, ctx->meta_buf);

    fprintf(stderr, "[CAM %d] waiting %lu ms before SPI CS...\n",
            id, (unsigned long)(SPI_CS_DELAY_US / 1000));
    usleep(SPI_CS_DELAY_US);

    fprintf(stderr, "[CAM %d][SPI] reading %lu JPEG bytes (+ %u sync discarded)...\n",
            id, (unsigned long)img_size, SPI_SYNC_BYTES);

    memset(ctx->image_buf, 0, img_size);
    spi_read_image(ctx, spi, img_size);

    int hdr_ok = (ctx->image_buf[0] == 0xFF &&
                  ctx->image_buf[1] == 0xD8 &&
                  ctx->image_buf[2] == 0xFF);
    int trl_ok = (img_size >= 2 &&
                  ctx->image_buf[img_size - 2] == 0xFF &&
                  ctx->image_buf[img_size - 1] == 0xD9);

    if (hdr_ok)
        fprintf(stderr, "[CAM %d] JPEG header OK  : FF D8 FF\n", id);
    else
        fprintf(stderr, "[CAM %d] WARN bad header : %02X %02X %02X\n", id,
                ctx->image_buf[0], ctx->image_buf[1], ctx->image_buf[2]);

    if (trl_ok)
        fprintf(stderr, "[CAM %d] JPEG trailer OK : FF D9\n", id);
    else
        fprintf(stderr, "[CAM %d] WARN bad trailer: %02X %02X\n", id,
                ctx->image_buf[img_size - 2], ctx->image_buf[img_size - 1]);

    {
        struct image_chunk_s mc;
        memset(&mc, 0, sizeof(mc));
        memcpy(mc.data, ctx->meta_buf, meta_len);
        mc.len      = meta_len;
        mc.frame_id = (uint16_t)id;
        mc.is_last  = 0;
        mc.is_meta  = 1;
        rb_push(cam_idx, &mc);
        fprintf(stderr, "[CAM %d] meta chunk pushed\n", id);
    }

    uint32_t total = (img_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
    fprintf(stderr, "[CAM %d] pushing %lu JPEG chunks...\n", id, (unsigned long)total);

    for (uint32_t ci = 0; ci < total; ci++)
    {
        uint32_t off = ci * CHUNK_SIZE;
        uint32_t len = img_size - off;
        if (len > CHUNK_SIZE) len = CHUNK_SIZE;

        struct image_chunk_s dc;
        memset(&dc, 0, sizeof(dc));
        memcpy(dc.data, &ctx->image_buf[off], len);
        dc.len      = (uint16_t)len;
        dc.frame_id = (uint16_t)id;
        dc.is_last  = (ci == total - 1) ? 1 : 0;
        dc.is_meta  = 0;
        rb_push(cam_idx, &dc);
    }

    fprintf(stderr, "[CAM %d] all %lu chunks pushed — DONE\n", id, (unsigned long)total);

    close(uart_fd);
    // free(ctx->image_buf);
    return NULL;
}
void *ring_buffer_subscriber_task(void *arg)
{
    struct image_chunk_s chunk;

    for (int cam = 0; cam < 2; cam++)
    {
        int byte_count = 0;
        while (1)
        {
            pthread_mutex_lock(&g_rb_mutex);
            int res = rb_read(cam, &chunk);
            pthread_mutex_unlock(&g_rb_mutex);

            if (res == 0)
            {
                if (chunk.is_meta)
                {
                    fprintf(stderr, "\n[CAM %d META] %.*s\n\n", cam + 1, chunk.len, chunk.data);
                    fprintf(stderr, "[CAM %d FULL JPEG HEX]:\n", cam + 1);
                }
                else
                {
                    for (uint16_t i = 0; i < chunk.len; i++)
                    {
                        fprintf(stderr, "%02X ", chunk.data[i]);
                        byte_count++;
                        if (byte_count % 16 == 0)
                        {
                            fprintf(stderr, "\n");
                        }
                    }

                    if (chunk.is_last)
                    {
                        fprintf(stderr, "\n\n");
                        fflush(stderr);
                        sem_post(&g_subscriber_done_sem);
                        break;
                    }
                }
            }
            else
            {
                usleep(1000);
            }
        }
    }

    return NULL;
}
int camera_MSN2_main(int argc, char *argv[])
{
    fprintf(stderr, "\n================ DUAL CAMERA MSN2 START ================\n");
    sem_init(&g_subscriber_done_sem, 0, 0);
    rb_init();
    pthread_t sub_thread;
    pthread_create(&sub_thread, NULL, ring_buffer_subscriber_task, NULL);
    camera_ctx_t cam1_ctx = {
        .camera_id = 1,
        .uart_dev  = "/dev/ttyS2",
        .spi_bus   = 3,
        .image_buf = g_cam1_img_buf
    };

    camera_ctx_t cam2_ctx = {
        .camera_id = 2,
        .uart_dev  = "/dev/ttyS3",
        .spi_bus   = 4,
        .image_buf = g_cam2_img_buf
    };

    pthread_t thread1, thread2;

    int ret1 = pthread_create(&thread1, NULL, camera_worker_thread, &cam1_ctx);
    if (ret1 != 0)
    {
        fprintf(stderr, "[MAIN] Failed to spawn Camera 1 thread: %d\n", ret1);
    }

    int ret2 = pthread_create(&thread2, NULL, camera_worker_thread, &cam2_ctx);
    if (ret2 != 0)
    {
        fprintf(stderr, "[MAIN] Failed to spawn Camera 2 thread: %d\n", ret2);
    }

    if (ret1 == 0) pthread_join(thread1, NULL);
    if (ret2 == 0) pthread_join(thread2, NULL);

    // fprintf(stderr, "\n================ DUAL CAMERA MSN2 DONE ================\n");
// 2. Wait for the subscriber task to finish dumping all hex data from ring buffer
// Wait for both camera dumps to complete:
    sem_wait(&g_subscriber_done_sem); // Camera 1 finished dumping
    sem_wait(&g_subscriber_done_sem); // Camera 2 finished dumping
    // 3. Print DONE banner strictly after subscriber output finishes
    fprintf(stderr, "\n================ DUAL CAMERA MSN2 DONE ================\n");

    sem_destroy(&g_subscriber_done_sem);
    return 0;
}