function [theta1, theta2] = ikine2(r)
%% inverse kinematics
% parameter
x = r(1); y = r(2);
global l1 l2 e
theta1 = zeros(2);
theta2 = zeros(2);
% ikine
for num = 1:2
    A = 2*(-(-1)^num*e - x)*l1;
    B = -2*y*l1;
    C = l2^2 - e^2 - l1^2 - x^2 - y^2;
    tmp = sqrt(B^2 + A^2 - C^2);
    theta1(num, :) = [2*atan((B + tmp)/(A + C)), 2*atan((B - tmp)/(A + C))];
    if num == 1
        phi = 0;
    else
        phi = pi;
    end
    if angleDelta(phi, theta1(num, 2)) < angleDelta(phi, theta1(num, 1))
        theta1(num, :) = theta1(num, end:-1:1);
    end
    theta2(num, :) = atan2(y - l1*sin(theta1(num, :)), x + (-1)^num*e - l1*cos(theta1(num, :)));
end
end