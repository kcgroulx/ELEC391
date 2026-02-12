/**
 * @file pid.h
 * @brief Public interface for PID-based motor angle control.
 */
#ifndef SRC_PID_H_
#define SRC_PID_H_

/* Typedefs */
typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float out_min, out_max;
    float deadband_angle;
} pid_config_t;

/* Public Function Declarations */
/**
 * @brief Initialize PID configuration and internal state.
 * @param config Pointer to PID configuration/state struct.
 */
void pid_init(pid_config_t* config);

/**
 * @brief Execute one PID control step for target angle tracking.
 * @param target_angle_deg Desired output angle in degrees.
 * @param dt Loop period in seconds.
 * @return Signed motor command.
 */
float pid_stepAndGetCommand(float target_angle_deg, float dt);

#endif /* SRC_PID_H_ */
