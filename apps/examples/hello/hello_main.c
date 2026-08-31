/*
 * hello.c  —  prints ONLY raw hex JPEG bytes, nothing else
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "ring_buffer.h"

int hello_main(int argc, char *argv[])
{
  struct image_chunk_s chunk;
  int done = 0;

  while (!done)
    {
      if (rb_read(&chunk) != 0)
        {
          usleep(1000);
          continue;
        }

      /* Skip metadata chunks silently */
      if (chunk.is_meta)
        continue;

      /* Print raw hex bytes — no address, no label, no newline between chunks */
      for (uint16_t i = 0; i < chunk.len; i++)
        printf("%02X", chunk.data[i]);

      if (chunk.is_last)
        {
          printf("\n");   /* single newline at end of full JPEG */
          done = 1;
        }
    }

  return 0;
}