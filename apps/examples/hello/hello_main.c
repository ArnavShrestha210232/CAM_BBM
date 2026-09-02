// /*
//  * hello.c  —  prints ONLY raw hex JPEG bytes, nothing else
//  */

// #include <nuttx/config.h>
// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>
// #include <unistd.h>

// #include "ring_buffer.h"

// int hello_main(int argc, char *argv[])
// {
//   struct image_chunk_s chunk;
//   // /* Loop */
//   // for (int img_count = 0; img_count<5; img_count++)
//   // {
//   int target_cam=1; /* Default to camera1 if not specified */
//   if(argc>1)
//   {
//     target_cam=atoi(argv[1]);
//   }
//   int done = 0;

//   while (!done)
//     {
//       if (rb_read(0,&chunk) != 0)
//         {
//           usleep(1000);
//           continue;
//         }
// /* Filter by camera ID*/
//       if (chunk.frame_id != target_cam)
//           continue;
//       /* Skip metadata chunks silently */
//       if (chunk.is_meta)
//         continue;

//       /* Print raw hex bytes — no address, no label, no newline between chunks */
//       for (uint16_t i = 0; i < chunk.len; i++)
//         printf("%02X", chunk.data[i]);

//       if (chunk.is_last)
//         {
//           printf("\n");   /* single newline at end of full JPEG */
//           done = 1;
//         }
//     }
//   // }

//   return 0;
// }
#include <stdio.h>
#include <unistd.h>
#include "ring_buffer.h"

int hello_main(int argc, char *argv[])
{
    struct image_chunk_s chunk;
    printf("[SUBSCRIBER] Starting hex reader...\n");

    while (1)
    {
        for (int cam = 0; cam < NUM_CAMERAS; cam++)
        {
            if (rb_read(cam, &chunk) == 0)
            {
                if (chunk.is_meta)
                {
                    printf("\n[CAM %d META] %.*s\n", cam + 1, chunk.len, chunk.data);
                }
                else
                {
                    printf("[CAM %d DATA] Chunk len %u B | Hex: ", cam + 1, chunk.len);
                    for (int i = 0; i < chunk.len; i++)
                    {
                        printf("%02X ", chunk.data[i]);
                    }
                    printf("\n");
                }
            }
        }
        usleep(10000); /* 10 ms delay */
    }

    return 0;
}