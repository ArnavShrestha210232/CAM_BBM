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

//Chunks of cam1 and cam2 and cam1 and cam2
// #include <stdio.h>
// #include <unistd.h>
// #include "ring_buffer.h"

// int hello_main(int argc, char *argv[])
// {
//     struct image_chunk_s chunk;
//     printf("[SUBSCRIBER] Starting hex reader...\n");

//     while (1)
//     {
//         for (int cam = 0; cam < NUM_CAMERAS; cam++)
//         {
//             if (rb_read(cam, &chunk) == 0)
//             {
//                 if (chunk.is_meta)
//                 {
//                     printf("\n[CAM %d META] %.*s\n", cam + 1, chunk.len, chunk.data);
//                 }
//                 else
//                 {
//                     printf("[CAM %d DATA] Chunk len %u B | Hex: ", cam + 1, chunk.len);
//                     for (int i = 0; i < chunk.len; i++)
//                     {
//                         printf("%02X ", chunk.data[i]);
//                     }
//                     printf("\n");
//                 }
//             }
//         }
//         usleep(10000); /* 10 ms delay */
//     }

//     return 0;
// }
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "ring_buffer.h"

#define MAX_FRAME_SIZE 16000

static uint8_t  g_frame_buf[NUM_CAMERAS][MAX_FRAME_SIZE];
static uint32_t g_frame_len[NUM_CAMERAS];

int hello_main(int argc, char *argv[])
{
    struct image_chunk_s chunk;
    memset(g_frame_len, 0, sizeof(g_frame_len));

    printf("[SUBSCRIBER] Starting full frame hex reader...\n");

    while (1)
    {
        for (int cam = 0; cam < NUM_CAMERAS; cam++)
        {
            while (rb_read(cam, &chunk) == 0)
            {
                int cam_id = cam + 1;

                if (chunk.is_meta)
                {
                    g_frame_len[cam] = 0;
                    printf("\n[CAM %d META] %.*s\n", cam_id, chunk.len, chunk.data);
                    continue;
                }

                /* Append chunk to frame buffer */
                if (g_frame_len[cam] + chunk.len <= MAX_FRAME_SIZE)
                {
                    memcpy(&g_frame_buf[cam][g_frame_len[cam]], chunk.data, chunk.len);
                    g_frame_len[cam] += chunk.len;
                }

                /* Print the complete image hex sequence in one single block */
                if (chunk.is_last)
                {
                    printf("\n[CAM %d FULL JPEG HEX - %u Bytes]:\n", cam_id, g_frame_len[cam]);
                    for (uint32_t i = 0; i < g_frame_len[cam]; i++)
                    {
                        printf("%02X ", g_frame_buf[cam][i]);
                    }
                    printf("\n\n");

                    g_frame_len[cam] = 0;
                }
            }
        }
        usleep(10000); /* 10 ms polling delay */
    }

    return 0;
}
// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>
// #include <unistd.h>
// #include "ring_buffer.h"

// #define MAX_FRAME_SIZE 40000

// static uint8_t  g_frame_buf[NUM_CAMERAS][MAX_FRAME_SIZE];
// static uint32_t g_frame_len[NUM_CAMERAS];

// static void print_clean_hex(int camera_id, const uint8_t *data, uint32_t len)
// {
//     printf("\n==================== [CAM %d] COMPLETE FRAME (%lu B) ====================\n", 
//            camera_id, (unsigned long)len);

//     for (uint32_t i = 0; i < len; i++)
//     {
//         if (i % 16 == 0)
//         {
//             printf("\n%04X: ", i);
//         }
//         printf("%02X ", data[i]);
//     }

//     printf("\n=======================================================================\n\n");
// }

// int hello_main(int argc, char *argv[])
// {
//     struct image_chunk_s chunk;

//     memset(g_frame_len, 0, sizeof(g_frame_len));
//     printf("[SUBSCRIBER] Listening for full image frames...\n");

//     while (1)
//     {
//         for (int cam = 0; cam < NUM_CAMERAS; cam++)
//         {
//             if (rb_read(cam, &chunk) == 0)
//             {
//                 int cam_id = cam + 1;

//                 if (chunk.is_meta)
//                 {
//                     /* Reset frame buffer for incoming image */
//                     g_frame_len[cam] = 0;
//                     printf("[CAM %d] Metadata: %.*s\n", cam_id, chunk.len, chunk.data);
//                     continue;
//                 }

//                 /* Append chunk data to camera frame buffer */
//                 if (g_frame_len[cam] + chunk.len <= MAX_FRAME_SIZE)
//                 {
//                     memcpy(&g_frame_buf[cam][g_frame_len[cam]], chunk.data, chunk.len);
//                     g_frame_len[cam] += chunk.len;
//                 }

//                 /* Print complete image when final chunk arrives */
//                 if (chunk.is_last)
//                 {
//                     print_clean_hex(cam_id, g_frame_buf[cam], g_frame_len[cam]);
//                     g_frame_len[cam] = 0;
//                 }
//             }
//         }

//         usleep(5000);
//     }

//     return 0;
// }