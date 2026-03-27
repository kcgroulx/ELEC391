#ifndef PIANOBOT_PID_H
#define PIANOBOT_PID_H

float pid_step(float targetPosition);
void pid_reset(void);
void pid_set_gains(float kp, float ki, float kd);
float pid_get_kp(void);
float pid_get_ki(void);
float pid_get_kd(void);
float pid_get_last_target_angle_deg(void);
float pid_get_last_angle_deg(void);
float pid_get_last_error_deg(void);
float pid_get_last_command(void);

#endif
