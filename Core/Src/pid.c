#include "pid.h"
 
/**
 * @brief  PID参数初始化
 *   @note   无
 * @retval 无
 */
void PID_param_init(pid *pid)
 {
     /* 初始化参数 */
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
  * @brief  PID算法实现
  * @param  val              实际值
  * @note    无
  * @retval 通过PID计算后的输出
  */

float PID_realize(float temp_val, pid *pid)
{
     
    pid->err=pid->target_val-temp_val;

    pid->integral+=pid->err;
  
    pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    
    pid->err_last=pid->err;
     
    return pid->actual_val;
}


