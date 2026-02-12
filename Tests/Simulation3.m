%% PID + MBK Rail with Noise, Filtering, and Homing

clear; clc;

%% Simulation parameters
dt = 0.001;              % 1 kHz loop
steps = 4000;            % 4 seconds
time = (0:steps-1)*dt;

%% MBK parameters   
mass = 3; %1kg
damping = 0.5; 
stiffness = 0;

%% State variables
position = zeros(1,steps);
velocity = zeros(1,steps);
position(1) = 0; 

%% Encoder parameters
encoder_noise_std = 0;   % 0.5 mm noise
measured_position = zeros(1,steps);

%% PID parameters
Kp = 600;
Ki = 80;
Kd = 66;

integrator = 0;
integrator_limit = 10.5;
output_limit = 0.9;

prev_measurement = 0;
prev_d_filtered = 0;
prev_error = 0;

% Derivative low-pass filter
derivative_cutoff = 20;           % Hz
alpha = exp(-2*pi*derivative_cutoff*dt);

%% Homing parameters
limit_switch_position = 0.0;      % physical rail end
homed = true;
homing_speed = -0.3;              % constant motor effort
target_position = 0.002;            % after homing

%% Control logging
control = zeros(1,steps);
homed_log = zeros(1,steps);  

%% Simulation loop
for i = 1:steps

    %% Encoder measurement with noise
    measured_position(i) = position(i) + ...
        encoder_noise_std * randn();

    %% Limit switch logic
    if ~homed
        % Move toward rail end until switch hit
        control(i) = max(min(homing_speed, 0.3), -0.3);

        if position(i) <= limit_switch_position
            homed = true;
            position(i) = 0;
            velocity(i) = 0;
            integrator = 0;
            prev_measurement = 0;
        end

    else
        %% PID control
        error = target_position - measured_position(i);

        % Integral
        integrator = integrator + 0.5*(error+prev_error)*dt
        integrator = max(min(integrator, integrator_limit), -integrator_limit)

        % Derivative (measurement-based)
        d_raw = -(measured_position(i) - prev_measurement)/dt;
        %d_filtered = alpha*prev_d_filtered + (1-alpha)*d_raw;
        d_filtered = d_raw;
        
        % PID output
        u = Kp*error + Ki*integrator + Kd*d_filtered;
        u = max(min(u, output_limit), -output_limit);

        control(i) = u;
        
        prev_error = error;
        prev_measurement = measured_position(i);
        prev_d_filtered = d_filtered;
    end

    homed_log(i) = homed; 

    %% MBK dynamics
    if i < steps
        acceleration = (control(i) ...
                        - damping*velocity(i) ...
                        - stiffness*position(i)) / mass;
        velocity(i+1) = velocity(i) + acceleration*dt;
        position(i+1) = position(i) + velocity(i+1)*dt;
    end
end

%% Save CSV
csvwrite('log.csv', [time', position', control']);

%% Plots
figure;

subplot(3,1,1);
plot(time, position, 'b', time, measured_position, 'r:');
ylabel('Position (m)');
legend('True','Measured');
grid on;

subplot(3,1,2);
plot(time, control);
ylabel('Control');
grid on;

subplot(3,1,3);
plot(time, homed_log);
ylabel('Homed');
xlabel('Time (s)');
grid on;

