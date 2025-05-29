

idf_component_register(
SRCS
"IMU/IMU.c" "PID/PID.c" "PWM/PWM.c" "control/control.c"   	
			
INCLUDE_DIRS
"IMU/include" "PID/include" "PWM/include" "control/include"

REQUIRES
sensor
Data_declaration

PRIV_REQUIRES

)