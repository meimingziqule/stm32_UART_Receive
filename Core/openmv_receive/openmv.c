#include "openmv.h"
#include "stm32f4xx_hal.h"  // ȷ������HAL��ͷ�ļ�
#include <string.h>         // ����memset����ͷ�ļ�
#include <ctype.h>          // ����isalpha����ͷ�ļ�
#include <stdlib.h>         // ����strtof����ͷ�ļ�
#include <stdio.h>          // ����printf����ͷ�ļ�
#include "usart.h"

uint8_t rx_buffer[50];
// ����һ����̬�Ľ������������ڼ�¼��ǰ�������ݵ�λ��
uint8_t rx_index = 0;
// ����һ����̬������׼����־�������ж�һ�ν����Ƿ����
uint8_t data_ready = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    // �ж��Ƿ��������Ĵ���
    if (huart->Instance == OPENMV_UART)
    {
        // ����һ����̬�Ľ��ջ�����
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

        // ��ʹ��������ʽ�������ݣ��������ж�
        if (rx_buffer[rx_index] == ';')
        {
            data_ready = 1;
        }
        else
        {
            rx_index++;
            if (rx_index >= sizeof(rx_buffer))
            {
                // �������������
                rx_index = 0;
            }
        }

        // ���ݽ���
        if (data_ready)
        {
            // ���帡�������飬���ڴ洢������ĸ�����ֵ
            float values[6];
            char* ptr = (char*)rx_buffer;
            char* end;
            int i = 0;

            // ������ͷ�� '#'
            char* temp_ptr = strchr(ptr, '#');
            if (temp_ptr != NULL)
            {
                ptr = temp_ptr + 1;
            }
            else
            {
                // δ�ҵ� '#' ����µĴ���
                rx_index = 0;
                data_ready = 0;
                memset(rx_buffer, 0, sizeof(rx_buffer));
                HAL_UART_Receive_IT(huart, rx_buffer + rx_index, 1);  // ȷ���ٴ����ý����ж�
                return;
            }

            // ����������
            while (*ptr && i < 6)
            {
                while (isalpha(*ptr)) ptr++;  // ������ĸ

                values[i] = strtof(ptr, &end);
                if (ptr == end) break;

                ptr = end;
                i++;

                // ��������
                while (*ptr == ',') ptr++;
            }

            // ��������������
            for (int j = 0; j < i; j++)
            {
                printf("Value %c: %f\n", 'A' + j, values[j]);
            }

            // ����״̬
            rx_index = 0;
            data_ready = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }

        // ��ʱ����жϱ�־�����������жϽ���
        HAL_UART_Receive_IT(huart, rx_buffer + rx_index, 1);
    }
}


