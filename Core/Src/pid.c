#include "pid.h"
 
/**
 * @brief  PID������ʼ��
 *   @note   ��
 * @retval ��
 */
void PID_param_init(pid *pid)
 {
     /* ��ʼ������ */
	 pid->target_val=0;
	 pid->actual_val=0.0;
	 pid->err=0.0;
	 pid->err_last=0.0;
	 pid->integral=0.0;
	 pid->Kp = 1.3;
	 pid->Ki = 0.00;
	 pid->Kd = 0.05;

 }
 
 /**
  * @brief  PID�㷨ʵ��
  * @param  val              ʵ��ֵ
  * @note    ��
  * @retval ͨ��PID���������
  */

float PID_realize(float temp_val, pid *pid)
{
     
    pid->err=pid->target_val-temp_val;

    pid->integral+=pid->err;
  
    pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    
    pid->err_last=pid->err;
     
    return pid->actual_val;
}


