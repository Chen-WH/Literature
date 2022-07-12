function del = angleDelta(theta1, theta2)
% This source code is written to calculate the angle between two vectors
del = abs(acos(dot([cos(theta1);sin(theta1)],[cos(theta2);sin(theta2)])));
end