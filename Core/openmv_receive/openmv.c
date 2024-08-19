#include "openmv.h"
#include "stm32f4xx_hal.h"  // 确保包含HAL库头文件
#include <string.h>         // 包含memset函数头文件
#include <ctype.h>          // 包含isalpha函数头文件
#include <stdlib.h>         // 包含strtof函数头文件
#include <stdio.h>          // 包含printf函数头文件
#include "usart.h"

uint8_t rx_buffer[50];
// 定义一个静态的接收索引，用于记录当前接收数据的位置
uint8_t rx_index = 0;
// 定义一个静态的数据准备标志，用于判断一次接收是否结束
uint8_t data_ready = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    // 判断是否是期望的串口
    if (huart->Instance == OPENMV_UART)
    {
        // 定义一个静态的接收缓冲区
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

        // 不使用阻塞方式接收数据，而依赖中断
        if (rx_buffer[rx_index] == ';')
        {
            data_ready = 1;
        }
        else
        {
            rx_index++;
            if (rx_index >= sizeof(rx_buffer))
            {
                // 缓冲区溢出处理
                rx_index = 0;
            }
        }

        // 数据解析
        if (data_ready)
        {
            // 定义浮点数数组，用于存储解析后的浮点数值
            float values[6];
            char* ptr = (char*)rx_buffer;
            char* end;
            int i = 0;

            // 跳过开头的 '#'
            char* temp_ptr = strchr(ptr, '#');
            if (temp_ptr != NULL)
            {
                ptr = temp_ptr + 1;
            }
            else
            {
                // 未找到 '#' 情况下的处理
                rx_index = 0;
                data_ready = 0;
                memset(rx_buffer, 0, sizeof(rx_buffer));
                HAL_UART_Receive_IT(huart, rx_buffer + rx_index, 1);  // 确保再次启用接收中断
                return;
            }

            // 解析浮点数
            while (*ptr && i < 6)
            {
                while (isalpha(*ptr)) ptr++;  // 跳过字母

                values[i] = strtof(ptr, &end);
                if (ptr == end) break;

                ptr = end;
                i++;

                // 跳过逗号
                while (*ptr == ',') ptr++;
            }

            // 处理解析后的数据
            for (int j = 0; j < i; j++)
            {
                printf("Value %c: %f\n", 'A' + j, values[j]);
            }

            // 重置状态
            rx_index = 0;
            data_ready = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }

        // 及时清除中断标志并重新启用中断接收
        HAL_UART_Receive_IT(huart, rx_buffer + rx_index, 1);
    }
}


