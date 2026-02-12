    %% Cascaded Position–Velocity Control with Homing

clear; clc;

%% Simulation parameters
dt = 0.001;              % 1 kHz inner loop
steps = 2000;            % 4 seconds
time = (0:steps-1)*dt;

%% MBK parameters
mass = 3;
damping = 0.5;
stiffness = 0;

%% State variables
position = zeros(1,steps);
velocity = zeros(1,steps);

%% Encoder parameters
encoder_noise_std = 0.00000;   % meters
measured_position = zeros(1,steps);
measured_velocity = zeros(1,steps);

%% -------- Position loop (outer) --------
Kp_pos = 5;            % position → velocity
max_velocity = 0.4;      % m/s limit

%% -------- Velocity loop (inner PID) --------
Kp_vel = 50;
Ki_vel = 4;
Kd_vel = 1;

vel_integrator = 0;
vel_integrator_limit = 1.5;
output_limit = 0.9;

prev_vel_error = 0;
prev_velocity = 0;

%% Homing parameters
limit_switch_position = 0.0;
homed = true;
homing_speed = -0.25;
target_position = 0.02;

%% Logs
control = zeros(1,steps);
desired_velocity_log = zeros(1,steps);
homed_log = zeros(1,steps);

%% Simulation loop
for i = 1:steps

    %% Encoder measurement
    measured_position(i) = position(i) + ...
        encoder_noise_std * randn();

    if i > 1
        measured_velocity(i) = ...
            (measured_position(i) - measured_position(i-1)) / dt;
    end

    %% Homing logic
    if ~homed
        control(i) = homing_speed;

        if position(i) <= limit_switch_position
            homed = true;
            position(i) = 0;
            velocity(i) = 0;
            vel_integrator = 0;
        end

    else
        %% -------- Position loop --------
        pos_error = target_position - measured_position(i);
        desired_velocity = Kp_pos * pos_error;
        desired_velocity = max(min(desired_velocity, max_velocity), ...
                               -max_velocity);

        %% -------- Velocity loop --------
        vel_error = desired_velocity - measured_velocity(i);

        vel_integrator = vel_integrator + ...
            0.5*(vel_error + prev_vel_error)*dt;
        vel_integrator = max(min(vel_integrator, ...
                                 vel_integrator_limit), ...
                                 -vel_integrator_limit);

        d_vel = (measured_velocity(i) - prev_velocity)/dt;

        u = Kp_vel*vel_error + ...
            Ki_vel*vel_integrator - ...
            Kd_vel*d_vel;

        u = max(min(u, output_limit), -output_limit);

        control(i) = u;

        prev_vel_error = vel_error;
        prev_velocity = measured_velocity(i);
    end

    homed_log(i) = homed;
    desired_velocity_log(i) = desired_velocity;

    %% MBK dynamics
    if i < steps
        acc = (control(i) ...
              - damping*velocity(i) ...
              - stiffness*position(i)) / mass;

        velocity(i+1) = velocity(i) + acc*dt;
        position(i+1) = position(i) + velocity(i+1)*dt;
    end
end

%% Plots
figure;

subplot(3,1,1);
plot(time, position, 'b');
ylabel('Position (m)');
grid on;

subplot(3,1,2);
plot(time, desired_velocity_log);
ylabel('Desired Vel (m/s)');
grid on;

subplot(3,1,3);
plot(time, control);
ylabel('Control');
grid on;

%subplot(4,1,4);
%plot(time, homed_log);
%ylabel('Homed');
%xlabel('Time (s)');
%grid on;
%