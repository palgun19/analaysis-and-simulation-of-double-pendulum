clear;clc;

theta1      = pi/3;
dtheta1     = 0;
theta2      = pi/4;
dtheta2     = 0;
g           = 9.81; 
m1          = 2; 
m2          = 1; 
l1          = 1; 
l2          = 1.5;
duration    = 10;

interval = [0, duration];
ivp = [theta1; dtheta1; theta2; dtheta2; g; m1; m2; l1; l2];
[t, y] = ode45(@double_pendulum_ODE, interval, ivp);

theta1 = y(:, 1);
dtheta1 = y(:, 2);
theta2 = y(:, 3);
dtheta2 = y(:, 4);

% Animation
figure;
subplot(2,2,1);
plot(t, theta1, t, theta2)
xlabel('Time (s)')
ylabel('angle (\theta (rad)')
legend('\theta_1','\theta_2')

subplot(2,2,2);
plot(t, dtheta1, t, dtheta2)
xlabel('Time (s)')
ylabel('angular velocities (d\theta/dt (rad/sec)')
legend('\theta_1','\theta_2')

subplot(2,2,3);
plot(theta1, dtheta1)
xlabel('\theta_1 (rad)')
ylabel('d\theta_1/dt (rad/sec)')

subplot(2,2,4);
plot(theta2, dtheta2)
xlabel('\theta_2 (rad)')
ylabel('d\theta_2/dt (rad/sec)')

% Animation
figure;
for i = 1:length(t)
    x1 = l1 * sin(theta1(i));
    y1 = -l1 * cos(theta1(i));
    
    x2 = x1 + l2 * sin(theta2(i));
    y2 = y1 - l2 * cos(theta2(i));
    
    plot([0, x1, x2], [0, y1, y2], '-o', 'LineWidth', 2);
    axis equal;
    title(['Time: ', num2str(t(i))]);
    xlabel('X-axis');
    ylabel('Y-axis');
    xlim([-2 2]);
    ylim([-2 2]);
    drawnow;
end

function xdot = double_pendulum_ODE(t, x)
    g = x(5); m1 = x(6); m2 = x(7); l1 = x(8); l2 = x(9);
    xdot = zeros(9, 1);
    xdot(1) = x(2);
    xdot(2) = -((g * (2 * m1 + m2) * sin(x(1)) + m2 * (g * sin(x(1) - 2 * x(3)) + 2 * (l2 * x(4)^2 + ...
        l1 * x(2)^2 * cos(x(1) - x(3))) * sin(x(1) - x(3)))) / ...
        (2 * l1 * (m1 + m2 - m2 * cos(x(1) - x(3))^2)));
    xdot(3) = x(4);
    xdot(4) = (((m1 + m2) * (l1 * x(2)^2 + g * cos(x(1))) + l2 * m2 * x(4)^2 * cos(x(1) - x(3))) * ...
        sin(x(1) - x(3))) / (l2 * (m1 + m2 - m2 * cos(x(1) - x(3))^2));
end
