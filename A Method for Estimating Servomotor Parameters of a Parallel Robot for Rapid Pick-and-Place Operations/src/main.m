clc;clear;close all;
%% parameters
global l1 l2 e
l1 = 0.245; l2 = 0.510; e = 0.06; H = 0.46;
m = 1.12; IA = 0.0573; mArA = 0.25;
b = 0.6; h = 0.15;  % 路径尺寸
a_max = 60;
g = 9.81;
T = [sqrt(5.7735*h/a_max); sqrt(5.7735*b/a_max); sqrt(5.7735*h/a_max)]; % P1-P2,P2-P3,P3-P4
tau = 0:0.01:1;
t = [tau*T(1), T(1) + tau*T(2), T(1) + T(2) + tau*T(3)];
n = length(tau);
Q = [0, 1; -1, 0];   % 叉乘反对称矩阵
%% inline function
s = @(num, tau) a_max*T(num)^2/5.7735*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
v = @(num, tau) a_max*T(num)/5.7735*(30*tau.^2 - 60*tau.^3 + 30*tau.^4);
a = @(num, tau) a_max/5.7735*(60*tau - 180*tau.^2 + 120*tau.^3);
%% ikine
r = [-b/2*ones(1, n), s(2, tau) - b/2, b/2*ones(1, n); s(1, tau) - h/2 - H, h/2*ones(1, n) - H, -s(3, tau) + h/2 - H];
dr = [zeros(1, n), v(2, tau), zeros(1, n); v(1, tau), zeros(1, n), -v(3, tau)];
ddr = [zeros(1, n), a(2, tau), zeros(1, n); a(1, tau), zeros(1, n), -a(3, tau)];
[theta1, theta2] = ikine(r);
u1 = [cos(theta1(1, :)); sin(theta1(1, :))];
u2 = [cos(theta1(2, :)); sin(theta1(2, :))];
w1 = [cos(theta2(1, :)); sin(theta2(1, :))];
w2 = [cos(theta2(2, :)); sin(theta2(2, :))];
J = zeros(2, 2, 3*n);
for num = 1:3*n
    J(:, :, num) = [w1(:, num)/(u1(:, num).'*Q*w1(:, num)), w2(:, num)/(u2(:, num).'*Q*w2(:, num))].'/l1;
end
%% angular velocity
dtheta = zeros(2, 3*n);
for num = 1:3*n
    dtheta(:, num) = J(:, :, num)*dr(:, num);
end
figure(1);
plot(t, dtheta(1, :), t, dtheta(2, :), "LineWidth", 1.5);
xlabel("t(s)");ylabel("angular velocity(rad/s)");
legend("1", "2");
ax = gca; ax.FontName = 'Times New Roman';
set(gca, 'FontSize',13);
%% angular accelaration
ddtheta = zeros(2, 3*n);
fv = zeros(2, 3*n);
for num = 1:3*n
    fv(1, num) = dr(:, num).'*(w1(:, num).'*u1(:, num)*w1(:, num)*w1(:, num).'/l1 + u1(:, num)*u1(:, num).'/l2)*dr(:, num)/l1/(u1(:, num).'*Q*w1(:, num))^3;
    fv(2, num) = dr(:, num).'*(w2(:, num).'*u2(:, num)*w2(:, num)*w2(:, num).'/l1 + u2(:, num)*u2(:, num).'/l2)*dr(:, num)/l1/(u2(:, num).'*Q*w2(:, num))^3;
    ddtheta(:, num) = J(:, :, num)*ddr(:, num);
end
ddtheta = ddtheta + fv;
figure(2);
plot(t, ddtheta(1, :), t, ddtheta(2, :), "LineWidth", 1.5);
xlabel("t(s)");ylabel("angular accelaration(rad/s^2)");
legend("1", "2");
ax = gca; ax.FontName = 'Times New Roman';
set(gca, 'FontSize',13);
%% torque
torq = zeros(2, 3*n);
for num = 1:3*n
    torq(:, num) = (IA*J(:, :, num) + m*inv(J(:, :, num).'))*ddr(:, num) + mArA*g*[cos(theta1(1, num)); cos(theta1(2, num))] + m*g*J(:, :, num).'\[0; 1];
end
torq = torq + IA*fv;
figure(3);
plot(t, torq(1, :), t, torq(2, :), "LineWidth", 1.5);
xlabel("t(s)");ylabel("torque(N·m)");
legend("1", "2");grid on;
ax = gca; ax.FontName = 'Times New Roman';
set(gca, 'FontSize',13);
%% power
power = torq.*dtheta;
figure(4);
plot(t, power(1, :), t, power(2, :), "LineWidth", 1.5);
xlabel("t(s)");ylabel("power(N·m/s)");
legend("1", "2");grid on;
ax = gca; ax.FontName = 'Times New Roman';
set(gca, 'FontSize',13);
%% distribution
x = -b/2:0.01:b/2; xList = length(x);
y = -h/2:0.01:h/2 - H; yList = length(y);
[X, Y] = meshgrid(x, y);
J = cell(xList, yList);
for row = 1:xList
    for col = 1:yList
        [theta1, theta2] = ikine2([x(row); y(col)]);
        u1 = [cos(theta1(1, 1)); sin(theta1(1, 1))];
        u2 = [cos(theta1(2, 1)); sin(theta1(2, 1))];
        w1 = [cos(theta2(1, 1)); sin(theta2(1, 1))];
        w2 = [cos(theta2(2, 1)); sin(theta2(2, 1))];
        J(row, col) = [w1(:, num)/(u1(:, num).'*Q*w1(:, num)), w2(:, num)/(u2(:, num).'*Q*w2(:, num))].'/l1;
    end
end