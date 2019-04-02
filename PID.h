#ifndef __PID_H__
#define __PID_H__

float PID (float kp, float ki, float kd, float kat, float predkosc, float poprzedni, float dt);

#define integral_max    (100)

#endif //__PID_H__