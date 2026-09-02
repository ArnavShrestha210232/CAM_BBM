#include "ring_buffer.h" 
#include <string.h> 

static struct image_chunk_s g_ring_buffer[NUM_CAMERAS][RING_BUF_LEN]; 
static int g_head[NUM_CAMERAS]; 
static int g_tail[NUM_CAMERAS]; 

void rb_init(void) 
{ 
  memset(g_head, 0, sizeof(g_head)); 
  memset(g_tail, 0, sizeof(g_tail)); 
  memset(g_ring_buffer, 0, sizeof(g_ring_buffer)); 
} 

int rb_write(int cam_index, const struct image_chunk_s *c) 
{ 
  if (cam_index < 0 || cam_index >= NUM_CAMERAS) 
  { 
    return -1; 
  } 

  int next = (g_head[cam_index] + 1) % RING_BUF_LEN; 
  
  if (next == g_tail[cam_index]) 
  { 
    return -1; /* Buffer full for this camera */ 
  } 
  memcpy(&g_ring_buffer[cam_index][g_head[cam_index]], c, sizeof(struct image_chunk_s)); 
  g_head[cam_index] = next; 
  return 0; 
} 

int rb_read(int cam_index, struct image_chunk_s *c) 
{ 
  if (cam_index < 0 || cam_index >= NUM_CAMERAS)
  { 
    return -1; 
  } 
  if (g_head[cam_index] == g_tail[cam_index]) 
  { 
    return -1; /* Buffer empty for this camera */ 
  } 
  memcpy(c, &g_ring_buffer[cam_index][g_tail[cam_index]], sizeof(struct image_chunk_s)); 
  g_tail[cam_index] = (g_tail[cam_index] + 1) % RING_BUF_LEN; 
  
  return 0; 
}