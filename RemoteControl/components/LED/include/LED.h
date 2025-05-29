
#ifndef LED_H_
#define LED_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"
#include "driver/gpio.h"

#define LED1  GPIO_NUM_2
#define LED2  GPIO_NUM_1

void LED_init(void);

#endif /* LED_H_ */
