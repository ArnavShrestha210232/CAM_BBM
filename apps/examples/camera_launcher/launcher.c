#include <nuttx/config.h>
#include <stdio.h>
#include <sched.h>

extern int camera_MSN2_main(int argc, char *argv[]);
extern int hello_main(int argc, char *argv[]);

int launcher_main(int argc, char *argv[])
{
  printf("Starting camera + subscriber tasks...\n");

  char *camera_argv[] = { NULL };
  char *hello_argv[]  = { NULL };

  int cam_pid = task_create("camera", 100, 8192, camera_MSN2_main, camera_argv);
  if (cam_pid < 0)
    {
      printf("ERROR: camera task failed\n");
    }

  int hello_pid = task_create("hello", 100, 4096, hello_main, hello_argv);
  if (hello_pid < 0)
    {
      printf("ERROR: hello task failed\n");
    }

  return 0;
}