

#include "IIC.h"

#include "driver/i2c.h"//IIC API库
#include <stdio.h>
#include "esp_system.h"
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_check.h"
#include "malloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_pm.h"
#include "soc/soc_memory_layout.h"
#include "hal/i2c_hal.h"
#include "hal/gpio_hal.h"
#include "soc/i2c_periph.h"
#include "driver/i2c.h"
#include "driver/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include <sys/param.h>

#define I2C_TRANS_BUF_MINIMUM_SIZE     (sizeof(i2c_cmd_desc_t) + \
                                        sizeof(i2c_cmd_link_t) * 8) /* It is required to have allocate one i2c_cmd_desc_t per command:
                                                                     * start + write (device address) + write buffer +
                                                                     * start + write (device address) + read buffer + read buffer for NACK +
                                                                     * stop */

typedef struct {
    i2c_hw_cmd_t hw_cmd;
    union {
        uint8_t* data;      // When total_bytes > 1
        uint8_t data_byte;  //when total_byte == 1
    };
    size_t bytes_used;
    size_t total_bytes;
} i2c_cmd_t;

typedef struct i2c_cmd_link {
    i2c_cmd_t cmd;              /*!< command in current cmd link */
    struct i2c_cmd_link *next;  /*!< next cmd link */
} i2c_cmd_link_t;

typedef struct {
    i2c_cmd_link_t *head;     /*!< head of the command link */
    i2c_cmd_link_t *cur;      /*!< last node of the command link */
    i2c_cmd_link_t *free;     /*!< the first node to free of the command link */

    void     *free_buffer;    /*!< pointer to the next free data in user's buffer */
    uint32_t  free_size;      /*!< remaining size of the user's buffer */
} i2c_cmd_desc_t;


void Error_print(esp_err_t err,int type){ 									//IIC队列错误信息汉化
	if(err == ESP_OK) 					printf("成功\n");

	if(err == ESP_FAIL){
		if(type == 0) 					printf("安装错误\n");						//初始化部分
		if(type == 1) 					printf("堆上不再剩下内存\n");					//队列部分
		if(type == 2) 					printf("发送命令错误，从属服务器尚未确认传输\n");	//发送部分
	}
	if(err == ESP_ERR_INVALID_ARG) 		printf("参数错误\n");
	if(err == ESP_ERR_NO_MEM) 			printf("用于创建的静态缓冲区太小cmd_handler\n");
	if(err == ESP_ERR_INVALID_STATE) 	printf("驱动程序未安装或未处于主模式\n");
	if(err == ESP_ERR_TIMEOUT) 			printf("操作超时，因为总线繁忙\n");


}

//开头这个INT是表示返回值是INT类型
void IIC_init(void) 							// IIC初始化
{
	printf("********************IIC初始化开始********************\n");
	i2c_config_t conf = {							//创建IIC配置结构体
			.mode = I2C_MODE_MASTER,  				//IIC设置为主模式
	        .sda_io_num = I2C_MASTER_SDA_IO, 		//设置SDA数据线GPIO
	        .scl_io_num = I2C_MASTER_SCL_IO,  		//设置SCL时钟线GPIO
	        .sda_pullup_en = GPIO_PULLUP_ENABLE, 	//设置数据线模式为上拉
	        .scl_pullup_en = GPIO_PULLUP_ENABLE,	//设置时钟线模式为上拉
	        .master.clk_speed = I2C_MASTER_FREQ_HZ, //设置时钟速率
	    };

	printf("IIC模式为%s\n",conf.mode == I2C_MODE_MASTER ? "主机模式" : "从机模式" );
	printf("IIC数据线SDA引脚: %d\n",conf.sda_io_num );
	printf("IIC时钟线SCL引脚: %d\n",conf.scl_io_num);
	printf("IIC数据线上拉%s\n",conf.sda_pullup_en == GPIO_PULLUP_ENABLE ? "启用" : "禁止" );
	printf("IIC时钟线上拉%s\n",conf.scl_pullup_en == GPIO_PULLUP_ENABLE ? "启用" : "禁止" );
	printf("IIC时钟速率: %d\n",conf.master.clk_speed);

	 //配置I2C总线
	 if(i2c_param_config(I2C_MASTER_NUM, &conf) == ESP_OK)printf("IIC总线配置成功\n");
	 else printf("IIC总线配置失败\n");

	 //安装IIC驱动
	 if(i2c_driver_install(I2C_MASTER_NUM,				//IIC端口
	     				   conf.mode,					//IIC模式
						   I2C_MASTER_RX_BUF_SIZE,		//接收缓冲区大小。只有从模式会使用此值，在主模式下会忽略它
						   I2C_MASTER_TX_BUF_SIZE,		//发送缓冲区大小。只有从模式会使用此值，在主模式下会忽略它
						   0 							//用于分配中断的标志。一个或多个（O是读取）
	 	 	 	 	 	   ) == ESP_OK) printf("IIC驱动程序安装成功\n");
	 else printf("IIC驱动程序安装失败\n");
	 printf("********************IIC初始化完成********************\n\n");
}

/*
 * 在指定端口读取指定设备的指定寄存器并返回数据，
 * i2c_port_t i2c_num      IIC设备端口
 * uint8_t device_address  要读取的IIC设备地址
 * uint8_t reg_address     要读取的寄存器地址
 * uint8_t* read_buffer    读取完成返回的数据
 * size_t read_size        读取的大小
 */
int iic_read(i2c_port_t i2c_num, uint8_t device_address,uint8_t reg_address,uint8_t* read_buffer, size_t read_size)
{
	//[起始信号S][从机地址|0][从机应答][寄存器地址][从机应答][起始信号S][从机地址|0][从机应答][寄存器中的的数据][从机非应答][停止信号P]
	//创建命令队列(句柄)
	uint8_t buffer[I2C_TRANS_BUF_MINIMUM_SIZE] = { 0 };
	i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
	assert (handle != NULL);

    //开始信号S
    if(i2c_master_start(handle)!=ESP_OK) goto esp_fail;
    //发送从机地址和写标志位，启用ACK信号
    if(i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE,true)!=ESP_OK) goto esp_fail;
    //从机寄存器地址，启用ACK信号
    if(i2c_master_write_byte(handle, reg_address,true)!=ESP_OK) goto esp_fail;
    //开始信号S
    if(i2c_master_start(handle)!=ESP_OK) goto esp_fail;
    //从机地址和读标志位，启用ACK信号
    if(i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_READ,true)!=ESP_OK) goto esp_fail;
    //从reg_address寄存器地址开始，往后的read_size位读取到read_buffer，最后一个字节检查从机NACK非应答
    if(i2c_master_read(handle, read_buffer, read_size, I2C_MASTER_LAST_NACK)!=ESP_OK) goto esp_fail;
    //停止信号P
    if(i2c_master_stop(handle)!=ESP_OK) goto esp_fail;
    //发送队列
    if(i2c_master_cmd_begin(i2c_num, handle, I2C_MASTER_TIMEOUT_MS)!=ESP_OK) goto esp_fail;
    //删除命令队列(句柄)
    i2c_cmd_link_delete_static(handle);
    return ESP_OK;

    esp_fail:
    //删除命令队列(句柄)
    i2c_cmd_link_delete_static(handle);
    return ESP_FAIL;


}

int iic_write(i2c_port_t i2c_num, uint8_t device_address,uint8_t reg_address,uint8_t write_buffer)
{
	//创建命令队列(句柄)
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    //开始信号S
    if(i2c_master_start(handle)!=ESP_OK) return ESP_FAIL;
    //发送从机地址和写标志位，启用ACK信号
    if(i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE,true)!=ESP_OK) return ESP_FAIL;
    //从机寄存器地址，启用ACK信号
    if(i2c_master_write_byte(handle, reg_address,true)!=ESP_OK) return ESP_FAIL;
    //写入到reg_address寄存器的数据，启用ACK信号
    if(i2c_master_write_byte(handle, write_buffer,true)!=ESP_OK) return ESP_FAIL;
    //停止信号P
    if(i2c_master_stop(handle)!=ESP_OK) return ESP_FAIL;
    //发送队列
    if(i2c_master_cmd_begin(i2c_num, handle, I2C_MASTER_TIMEOUT_MS)!=ESP_OK) return ESP_FAIL;
    //删除命令队列(句柄)
    i2c_cmd_link_delete_static(handle);
    return ESP_OK;
}




