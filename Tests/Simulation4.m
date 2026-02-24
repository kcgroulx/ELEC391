%% Basic Position PID controller simulation
clear; clc;

%% Simulation parameters
dt = 0.001;
steps = 3000;
time = (0:steps-1) * dt;

%% Plant parameters (m, damping, stiffness)
mass = 3.0;
damping = 0.5;
stiffness = 0.1;
command_gain = 75;

%% Target
target_position = 0.05;

%% Logs
position = zeros(1, steps);
velocity = zeros(1, steps);
control = zeros(1, steps);
error_log = zeros(1, steps);

%% PID parameters
Kp = 3.2;
Ki = 0.07;
Kd = 0.5;
integrator = 0.0;
integrator_limit = 50.0;
prev_error = 0;
prev_measurement = 0;
derivative_lpf = 0;
alpha = exp(-2*pi*20*dt);

for i = 1:steps
    measurement = position(i);
    error = target_position - measurement;
    integrator = integrator + 0.5 * (error + prev_error) * dt;
    integrator = max(min(integrator, integrator_limit), -integrator_limit);
    derivative_raw = (measurement - prev_measurement) / dt;
    derivative_lpf = alpha * derivative_lpf + (1-alpha) * derivative_raw;

    u = Kp * error + Ki * integrator - Kd * derivative_lpf;
    u = max(min(u, 1.0), -1.0);
    control(i) = u;
    error_log(i) = error;

    if i < steps
        acceleration = (u * command_gain - damping * velocity(i) - stiffness * position(i)) / mass;
        velocity(i + 1) = velocity(i) + acceleration * dt;
        position(i + 1) = position(i) + velocity(i + 1) * dt;
    end

    prev_error = error;
    prev_measurement = measurement;
end

%% Plot
figure;
subplot(3,1,1);
plot(time, position, 'b', time, target_position * ones(size(time)), 'r--');
ylabel('Position (m)');
legend('Actual','Target');
grid on;

subplot(3,1,2);
plot(time, control);
ylabel('Command');
grid on;

subplot(3,1,3);
plot(time, error_log);
ylabel('Error (m)');
xlabel('Time (s)');
grid on;
