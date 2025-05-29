
#ifndef COMPONENTS_FLIGHT_CONTROL_PID_INCLUDE_PID_H_
#define COMPONENTS_FLIGHT_CONTROL_PID_INCLUDE_PID_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "Data_declaration.h"

void PID_Postion_Cal(PID_TYPE*PID,float target,float measure);
void PidParameter_init(void);


#endif /* COMPONENTS_FLIGHT_CONTROL_PID_INCLUDE_PID_H_ */
