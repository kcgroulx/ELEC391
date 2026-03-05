%% PID + MBK Rail with Noise, Filtering, and Homing

clear; clc;

%% Simulation parameters
dt = 0.001;              % 1 kHz loop
steps = 4000;            % 4 seconds
time = (0:steps-1)*dt;

%% MBK parameters   
mass = 1; %1kg
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
Kp = 67;
Ki = 3;
Kd = 12;

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
target_position = 0.05;            % after homing

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



measured_history = zeros(size(time)); % Pre-allocate for speed
control_history = zeros(size(time));
%% Noise Parameters
encoder_noise_std = 0.0009; % Adjust this value (e.g., 0.001 for clean, 0.02 for noisy)
for i = 1:steps
    
% Inside your simulation loop:
% 1. Get the "Perfect" position from your physics model
% 2. Add noise to create the "Measured" position
% ... (inside the loop at index i) ...
measured_position = position(i) + 0.0009 * randn();
measured_history(i) = measured_position; % Save the noisy value for plotting later



control_history(i) = control(i) + 0.014 * randn();
end
% ... (Run your PID logic using measured_position) ...

%% Plotting Code
subplot(2,1,1);
plot(time, position, 'b', 'LineWidth', 1.5); 
hold on;
plot(time, measured_history, 'r:', 'Color', [1, 0, 0, 0.3]); 
hold off;

ylabel('Position (m)');
legend('True', 'Measurement');
grid on;

subplot(2,1,2);
plot(time, control_history, 'Color', [0, 0.5, 0]); % Dark green for control
ylabel('Control Signal (u)');
xlabel('Time (s)');
grid on;

%subplot(3,1,3);
%plot(time, homed_log);
%ylabel('Homed');
%xlabel('Time (s)');
%grid on;

