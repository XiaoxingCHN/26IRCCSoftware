//
// Created by LIMBO on 2026/4/21.
//
#include "grayscale.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "main.h"

static USARTInstance *grayscale_usart_instance;
static GrayscaleData_t grayscale_data;

uint8_t tx_flag = 0;

// 协议解析相关变量
#define PACKAGE_SIZE 128  // 根据实际协议调整
static uint8_t ir_start = 0;
static uint8_t ir_step = 0;
static uint8_t ir_rx_buff[PACKAGE_SIZE];
static uint8_t new_package[PACKAGE_SIZE];

static DaemonInstance *grayscale_daemon_instance;

/**
 * @brief 字符串分割函数
 */
static void splitString(char* mystrArray[], char *str, const char *delimiter)
{
    char *token = strtok(str, delimiter);
    int i = 0;
    while (token != NULL && i < 10)
    {
        mystrArray[i++] = token;
        token = strtok(NULL, delimiter);
    }
}

/**
 * @brief 灰度传感器数据解析回调函数
 * 协议格式: $A,IR1:值1,IR2:值2,...#
 */
static void GrayscaleRxCallback()
{
    DaemonReload(grayscale_daemon_instance);  // 喂狗

    uint8_t *buff = grayscale_usart_instance->recv_buff;
    uint16_t size = grayscale_usart_instance->recv_buff_size;

    tx_flag = 1;

    // H7必须刷Cache (如果使用STM32H7)
    #ifdef STM32H723xx
    SCB_InvalidateDCache_by_Addr((uint32_t *)buff, size);
    #endif

    // 把DMA收到的这一段数据，逐个字节喂进状态机
    for (uint16_t i = 0; i < size; i++)
    {
        uint8_t rxtemp = buff[i];


        // 协议解析逻辑
        if (rxtemp == '$')
        {
            ir_start = 1;
            ir_step = 0;
            ir_rx_buff[ir_step++] = rxtemp;
        }
        else if (ir_start)
        {
            if (ir_step < PACKAGE_SIZE)
            {
                ir_rx_buff[ir_step++] = rxtemp;

                if (rxtemp == '#') // 收齐一包
                {
                    memcpy(new_package, ir_rx_buff, ir_step);
                    new_package[ir_step] = '\0'; // 补\0方便打印

                    // 处理A模式数据 (模拟值)
                    if (new_package[0] == '$' && new_package[1] == 'A')
                    {
                        char* strArray[10] = {NULL};
                        char str_temp[PACKAGE_SIZE] = {'\0'};

                        strcpy(str_temp, (char*)new_package);
                        char *p_hash = strchr(str_temp, '#');
                        if(p_hash) *p_hash = '\0';

                        splitString(strArray, str_temp, ",");

                        // 解析8个传感器的模拟值
                        for(int i = 0; i < 8 && strArray[i+1] != NULL; i++)
                        {
                            char *colon = strchr(strArray[i+1], ':');
                            if(colon)
                            {
                                grayscale_data.sensor_values[i] = atoi(colon + 1);
                            }
                        }

                        grayscale_data.valid = 1;
                        grayscale_data.new_package_flag = 1;

                        LOGINFO("[grayscale] A-mode data received: %d %d %d %d %d %d %d %d",
                               grayscale_data.sensor_values[0], grayscale_data.sensor_values[1],
                               grayscale_data.sensor_values[2], grayscale_data.sensor_values[3],
                               grayscale_data.sensor_values[4], grayscale_data.sensor_values[5],
                               grayscale_data.sensor_values[6], grayscale_data.sensor_values[7]);
                    }
                    else
                    {
                        LOGWARNING("[grayscale] Invalid package header: %c%c", new_package[0], new_package[1]);
                    }

                    ir_start = 0;
                    ir_step = 0;
                    memset(ir_rx_buff, 0, PACKAGE_SIZE);
                }
            }
            else // 超长异常
            {
                ir_start = 0;
                ir_step = 0;
                memset(ir_rx_buff, 0, PACKAGE_SIZE);
                LOGWARNING("[grayscale] Package too long");
            }
        }
    }

    // 注意：USARTServiceInit会在USARTRegister时自动调用，这里不需要手动重启DMA
    // 因为bsp_usart库会在回调函数执行完毕后自动重启接收
}

/**
 * @brief 灰度传感器离线回调函数
 */
static void GrayscaleLostCallback(void *id)
{
    memset(&grayscale_data, 0, sizeof(GrayscaleData_t));
    USARTServiceInit(grayscale_usart_instance);  // 重新启动接收
    LOGWARNING("[grayscale] Grayscale sensor lost");
}

/**
 * @brief 灰度传感器初始化
 * @param usart_handle 串口硬件句柄
 * @return 灰度传感器数据指针
 */
GrayscaleData_t *GrayscaleInit(UART_HandleTypeDef *usart_handle)
{
    // 配置串口初始化参数
    USART_Init_Config_s conf = {
        .recv_buff_size = PACKAGE_SIZE,  // 根据协议设置缓冲区大小
        .usart_handle = usart_handle,
        .module_callback = GrayscaleRxCallback
    };

    // 注册串口实例
    grayscale_usart_instance = USARTRegister(&conf);

    // 注册守护进程
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 50,  // 500ms未收到数据视为离线
        .callback = GrayscaleLostCallback,
        .owner_id = NULL
    };
    grayscale_daemon_instance = DaemonRegister(&daemon_conf);

    // 初始化数据
    memset(&grayscale_data, 0, sizeof(GrayscaleData_t));
    memset(ir_rx_buff, 0, PACKAGE_SIZE);
    memset(new_package, 0, PACKAGE_SIZE);

    LOGINFO("[grayscale] Grayscale sensor initialized");
    return &grayscale_data;
}

/**
 * @brief 发送控制命令给灰度传感器
 * @param calib 校准模式 (0/1)
 * @param a_mode 模拟模式 (0/1)
 * @param d_mode 数字模式 (0/1)
 */
void GrayscaleSendControl(uint8_t calib, uint8_t a_mode, uint8_t d_mode)
{
    uint8_t cmd[8] = "$0,0,0#";

    cmd[1] = '0' ;   // calib
    cmd[3] = '1' ;  // a_mode
    cmd[5] = '0' ;  // d_mode

    USARTSend(grayscale_usart_instance, cmd, sizeof(cmd), USART_TRANSFER_BLOCKING);
}

/**
 * @brief 发送查询命令给灰度传感器 (发送控制命令，默认模拟模式)
 */
void GrayscaleSendQuery()
{
    GrayscaleSendControl(0, 1, 0);  // calib=0, a_mode=1, d_mode=0
}

/**
 * @brief 获取灰度传感器数据
 * @return 灰度传感器数据指针
 */
GrayscaleData_t *GrayscaleGetData()
{
    return &grayscale_data;
}

/**
 * @brief 检查灰度传感器是否在线
 * @return 1-在线 0-离线
 */
uint8_t GrayscaleIsOnline()
{
    return DaemonIsOnline(grayscale_daemon_instance);
}

/**
 * @brief 清除新数据包标志
 */
void GrayscaleClearNewPackageFlag()
{
    grayscale_data.new_package_flag = 0;
}

