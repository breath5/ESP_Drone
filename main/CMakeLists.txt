

idf_component_register(
SRCS
main.c 
INCLUDE_DIRS

PRIV_INCLUDE_DIRS

REQUIRES
inside_communication  #内部通信
sensor                #传感器
flight_control		  #飞行控制
External_communication#外部通信
LED                   #灯
Data_declaration      #数据结构体存储
VBAT                  #电池电量
PRIV_REQUIRES
)
