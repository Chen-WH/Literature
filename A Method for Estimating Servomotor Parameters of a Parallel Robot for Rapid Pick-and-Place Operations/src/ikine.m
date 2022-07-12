function [theta1, theta2] = ikine(r)
%% inverse kinematics
% parameter
n = size(r, 2);
x = r(1, :); y = r(2, :);
global l1 l2 e
theta = zeros(2, 2);   % temporary
theta1 = zeros(2, n);
theta2 = zeros(2, n);
phi = [0; pi];
% ikine
for count = 1:n
    for num = 1:2
        A = 2*(-(-1)^num*e - x(count))*l1;
        B = -2*y(count)*l1;
        C = l2^2 - e^2 - l1^2 - x(count)^2 - y(count)^2;
        tmp = sqrt(B^2 + A^2 - C^2);
        theta(num, :) = [2*atan((B + tmp)/(A + C)), 2*atan((B - tmp)/(A + C))];
        if angleDelta(phi(num), theta(num, 2)) < angleDelta(phi(num), theta(num, 1))
            theta(num, :) = theta(num, end:-1:1);
        end
        theta2(num, count) = atan2(y(count) - l1*sin(theta(num, 1)), x(count) + (-1)^num*e - l1*cos(theta(num, 1)));
    end
    theta1(:, count) = theta(:, 1);
end
end