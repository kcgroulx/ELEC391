%% Basic Position PID Controller (No Extras)
clear; clc;

%% Simulation parameters
dt = 0.001;
steps = 2000;
time = (0:steps-1) * dt;

%% Plant parameters (mass–spring–damper)
mass = 7.0;
damping = 0.5;
stiffness = 0.1;
command_gain = 25;

%% Target
target_position = 0.05;

%% Logs
position = zeros(1, steps);
velocity = zeros(1, steps);
control  = zeros(1, steps);
error_log = zeros(1, steps);

%% Basic PID parameters
Kp = 5.9;
Ki = 0.4;
Kd = 0.7;

integrator = 0;
prev_error = 0;

%% Simulation loop
for i = 1:steps
    
    % Current measurement
    measurement = position(i);
    
    % Error
    error = target_position - measurement;
    
    % Basic integral (rectangular method)
    integrator = integrator + error * dt;
    
    % Basic derivative (of error)
    derivative = (error - prev_error) / dt;
    
    % PID control law
    u = Kp*error + Ki*integrator + Kd*derivative;
    
    % Log values
    control(i) = u;
    error_log(i) = error;
    
    % Plant update (Euler integration)
    if i < steps
        acceleration = (u*command_gain ...
                       - damping*velocity(i) ...
                       - stiffness*position(i)) / mass;
                   
        velocity(i+1) = velocity(i) + acceleration*dt;
        position(i+1) = position(i) + velocity(i+1)*dt;
    end
    
    prev_error = error;
end

%% Plot Results
figure;

subplot(3,1,1)
plot(time, position, 'b', ...
     time, target_position*ones(size(time)), 'k--')
ylabel('Position (m)')
legend('Actual','Target')
grid on

subplot(3,1,2)
plot(time, control, 'b')
ylabel('Control')
grid on

subplot(3,1,3)
plot(time, error_log, 'b')
ylabel('Error (m)')
xlabel('Time (s)')
grid on