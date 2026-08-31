#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

/*
 * Matches Verilog CircularBuffer:
 *   - Separate wr_ptr / rd_ptr (ever-increasing, mod DEPTH for slot)
 *   - count tracks occupancy  (full = count==DEPTH, empty = count==0)
 *   - No data corruption — is_meta is a dedicated field, NOT a marker
 *     stuffed into the payload bytes
 */

#define RB_DEPTH    64          /* must be power-of-2 — matches Verilog DEPTH  */
#define CHUNK_SIZE  256         /* bytes per chunk payload                      */

/* One slot — mirrors the packed Verilog word fields */
struct image_chunk_s
{
  uint8_t  data[CHUNK_SIZE];   /* JPEG payload or metadata string              */
  uint16_t len;                /* valid bytes in data[]                        */
  uint32_t frame_id;           /* which frame this chunk belongs to            */
  uint8_t  is_last;            /* 1 = final JPEG chunk for this frame          */
  uint8_t  is_meta;            /* 1 = metadata string chunk, 0 = JPEG data     */
};

/*
 * Ring buffer state — mirrors Verilog registers:
 *   wr_ptr  advances on every successful write  (wraps with % RB_DEPTH)
 *   rd_ptr  advances on every successful read   (wraps with % RB_DEPTH)
 *   count   occupancy 0..RB_DEPTH
 *           full  when count == RB_DEPTH
 *           empty when count == 0
 */
struct ring_buffer_s
{
  struct image_chunk_s slots[RB_DEPTH];
  volatile uint32_t    wr_ptr;   /* ever-increasing write pointer */
  volatile uint32_t    rd_ptr;   /* ever-increasing read pointer  */
  volatile uint32_t    count;    /* current number of chunks held */
};

/* API */
void rb_init  (void);
int  rb_write (const struct image_chunk_s *c);   /* 0=ok  -1=full  */
int  rb_read  (struct image_chunk_s *c);          /* 0=ok  -1=empty */
int  rb_count (void);                             /* current occupancy */
int  rb_full  (void);                             /* 1 if full         */
int  rb_empty (void);                             /* 1 if empty        */

#endif /* RING_BUFFER_H */