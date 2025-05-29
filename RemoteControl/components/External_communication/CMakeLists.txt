
idf_component_register(
SRCS
"remote_control/remote_control.c" "WIFI/WIFI.c" "UDP_TCP/UDP_TCP.c"
INCLUDE_DIRS
 "remote_control/include" "WIFI/include" "UDP_TCP/include"

REQUIRES
inside_communication 
nvs_flash
Data_declaration
PRIV_REQUIRES

)