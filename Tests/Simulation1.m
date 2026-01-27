data = readmatrix('log.csv');
time = data(:,1);
position = data(:,2);
control = data(:,3);

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
