
#include "Data_declaration.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"

bool init_ok = false;
uint32_t VBAT;
uint32_t UAV_VBAT;
uint32_t alt;
