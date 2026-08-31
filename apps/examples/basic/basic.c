/****************************************************************************
 * apps/examples/basic/basic_main.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h> 
#include <unistd.h>
#include <fcntl.h>

#include "gpio.h"

int main(int argc, FAR char *argv[]) 
{
  printf("Starting......");
   while(1)
 {
  stm32_configgpio(GPIO_MY_PA2);

    stm32_gpiowrite(GPIO_MY_PA2, true);
    usleep(1);
    stm32_gpiowrite(GPIO_MY_PA2, false);
    usleep(1);
  stm32_configgpio(GPIO_MY_PA3);
    stm32_gpiowrite(GPIO_MY_PA3, true);
    usleep(1);
    stm32_gpiowrite(GPIO_MY_PA3, false);
    usleep(1);
 }
return 0;
}
