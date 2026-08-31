#pragma once

#include <stdint.h>
#include <uORB/uORB.h>

enum camera_state_e
{
  CAMERA_IDLE = 0,
  CAMERA_CAPTURE_STARTED,
  CAMERA_FRAME_DONE
};

struct camera_status_s
{
  uint64_t timestamp;
  uint8_t  state;
  uint32_t frame_id;
  uint32_t image_size;
  uint32_t total_chunks;
};

ORB_DECLARE(camera_status);