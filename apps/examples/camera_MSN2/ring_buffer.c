#include "ring_buffer.h"
#include <string.h>
#include <nuttx/mutex.h>

/*
 * ring_buffer.c
 *
 * Logic mirrors the Verilog CircularBuffer exactly:
 *
 *   Verilog                     C equivalent
 *   ──────────────────────────  ─────────────────────────────────
 *   wr_en && !full  → write     rb_write()  checks count < RB_DEPTH
 *   rd_en && !empty → read      rb_read()   checks count > 0
 *   wr_ptr <= wr_ptr + 1        g_rb.wr_ptr++ then % RB_DEPTH for slot
 *   rd_ptr <= rd_ptr + 1        g_rb.rd_ptr++ then % RB_DEPTH for slot
 *   count +1 / -1 / hold        count updated after every write/read
 *   full  = (count == DEPTH)    rb_full()
 *   empty = (count == 0)        rb_empty()
 */

static struct ring_buffer_s g_rb;
static mutex_t              g_rb_lock;

/* ------------------------------------------------------------------ */
void rb_init(void)
{
  memset(&g_rb, 0, sizeof(g_rb));
  g_rb.wr_ptr = 0;
  g_rb.rd_ptr = 0;
  g_rb.count  = 0;
  nxmutex_init(&g_rb_lock);
}

/* ------------------------------------------------------------------ */
/*  Write one chunk                                                    */
/*  Verilog: if (wr_en && !full) { mem[wr_ptr] <= wr_data;           */
/*                                  wr_ptr <= wr_ptr + 1; count++ }  */
/* ------------------------------------------------------------------ */
int rb_write(const struct image_chunk_s *c)
{
  if (!c) return -1;

  nxmutex_lock(&g_rb_lock);

  /* full check — mirrors: assign full = (count == DEPTH) */
  if (g_rb.count >= RB_DEPTH)
    {
      nxmutex_unlock(&g_rb_lock);
      return -1;   /* full — caller retries */
    }

  uint32_t slot = g_rb.wr_ptr % RB_DEPTH;           /* slot index      */
  memcpy(&g_rb.slots[slot], c, sizeof(*c));
  g_rb.wr_ptr++;                                     /* advance pointer */
  g_rb.count++;                                      /* update count    */

  nxmutex_unlock(&g_rb_lock);
  return 0;
}

/* ------------------------------------------------------------------ */
/*  Read one chunk                                                     */
/*  Verilog: if (rd_en && !empty) { rd_data = mem[rd_ptr];           */
/*                                   rd_ptr <= rd_ptr + 1; count-- } */
/* ------------------------------------------------------------------ */
int rb_read(struct image_chunk_s *c)
{
  if (!c) return -1;

  nxmutex_lock(&g_rb_lock);

  /* empty check — mirrors: assign empty = (count == 0) */
  if (g_rb.count == 0)
    {
      nxmutex_unlock(&g_rb_lock);
      return -1;   /* empty — caller retries */
    }

  uint32_t slot = g_rb.rd_ptr % RB_DEPTH;           /* slot index      */
  memcpy(c, &g_rb.slots[slot], sizeof(*c));
  g_rb.rd_ptr++;                                     /* advance pointer */
  g_rb.count--;                                      /* update count    */

  nxmutex_unlock(&g_rb_lock);
  return 0;
}

/* ------------------------------------------------------------------ */
int rb_count(void)
{
  nxmutex_lock(&g_rb_lock);
  int n = (int)g_rb.count;
  nxmutex_unlock(&g_rb_lock);
  return n;
}

int rb_full(void)
{
  nxmutex_lock(&g_rb_lock);
  int f = (g_rb.count >= RB_DEPTH) ? 1 : 0;
  nxmutex_unlock(&g_rb_lock);
  return f;
}

int rb_empty(void)
{
  nxmutex_lock(&g_rb_lock);
  int e = (g_rb.count == 0) ? 1 : 0;
  nxmutex_unlock(&g_rb_lock);
  return e;
}