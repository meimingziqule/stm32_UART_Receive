#ifndef __PID_H__
#define __PID_H__

typedef struct
 {
     float target_val;
     float actual_val;
     float err;
     float err_last;
     float Kp,Ki,Kd;
     float integral;
 }pid;



void PID_param_init(pid *pid);
 
 
float PID_realize(float temp_val, pid *pid);
 
#endif


 
