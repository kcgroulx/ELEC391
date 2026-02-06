%% PID + MBK Rail Simulation in MATLAB

% Simulation parameters
dt = 0.001;          % time step (1 kHz)
steps = 10000;        % number of steps 
time = (0:steps-1)*dt;

% MBK parameters
mass = 1.0;           % kg
damping = 1.0;        % N·s/m
stiffness = 1.0;      % N/m

% Initialize state variables
position = zeros(1, steps);
velocity = zeros(1, steps);

% PID parameters
Kp = 6.0;
Ki = 1.0;
Kd = 0.1;
integrator = 0.0;
prev_measurement = 0.0;
integrator_limit = 5.0;
output_limit = 5.0;

% Target position (setpoint)
target_position = 0.1;  % meters - .5 meter long rail

% Control storage for plotting
control = zeros(1, steps);

%% Simulation loop
for i = 1:steps
    measurement = position(i);
    error = target_position - measurement;

    % PID computation
    integrator = integrator + error*dt;
    % Limit integrator
    if integrator > integrator_limit, integrator = integrator_limit; end
    if integrator < -integrator_limit, integrator = -integrator_limit; end
    P = Kp * error;
    I = Ki * integrator;
    D = -Kd * (measurement - prev_measurement)/dt;

    output = P + I + D;
    % Limit output
    if output > output_limit, output = output_limit; end
    if output < -output_limit, output = -output_limit; end

    prev_measurement = measurement;

    % Save control signal
    control(i) = output;

    % MBK dynamics
    if i < steps
        acceleration = (output - damping*velocity(i) - stiffness*position(i))/mass;
        velocity(i+1) = velocity(i) + acceleration*dt;
        position(i+1) = position(i) + velocity(i+1)*dt;
    end
end

%% Save CSV
csvwrite('log.csv', [time', position', control']);

%% Plot
figure;
subplot(2,1,1);
plot(time, position);
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

subplot(2,1,2);
plot(time, control);
xlabel('Time (s)');
ylabel('Control Effort');
grid on;
