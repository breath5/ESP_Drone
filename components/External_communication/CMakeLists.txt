

idf_component_register(
SRCS
"anotc_client_v4.34/anotc.c" "remote_control/remote_control.c"
 "WIFI/WIFI.c" "UDP_TCP/UDP_TCP.c"  "mqtt_lot/mqtt_lot.c"
INCLUDE_DIRS
"anotc_client_v4.34/include" "remote_control/include"
 "WIFI/include" "UDP_TCP/include" "mqtt_lot/include"

REQUIRES
inside_communication 
nvs_flash
Data_declaration
flight_control
        mqtt
        json
PRIV_REQUIRES

)