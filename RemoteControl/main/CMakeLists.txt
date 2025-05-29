

idf_component_register(
SRCS                # 这里标记出目录下的c文件 list the source files of this component
main.c 
INCLUDE_DIRS        # 这里标记出目录中存放h文件的文件夹 optional, add here public include directories
"."
PRIV_INCLUDE_DIRS   # optional, add here private include directories

REQUIRES            # 标记出需要的资源 optional, list the public requirements (component names)
RC_control_signal
Data_declaration
inside_communication
External_communication
lvgl_porting
LED
PRIV_REQUIRES       # 标记出需要的资源 optional, list the private requirements
)