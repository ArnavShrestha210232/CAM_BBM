#include <nuttx/config.h>

#include <uORB/uORB.h>
#include "camera_status.h"

ORB_DEFINE(camera_status,
           struct camera_status_s,
           NULL);