% https://math.stackexchange.com/users/6622/joriki

close all
clear;clc

theta = repmat(deg2rad(80),3,1);

z = rand(3,1) .* (1-cos(theta)) + cos(theta);
t = rand(3,1) .* 2*pi;
tilt = [z,sqrt(1-z.^2).*cos(t), sqrt(1-z.^2).*sin(t)];

x = [1;0;0];
Rr = aa_getRotnMat(cross(x,tilt(1,:)'),acos(tilt(1,:)*x));
Rv = aa_getRotnMat(cross(x,tilt(2,:)'),acos(tilt(2,:)*x));
Ra = aa_getRotnMat(cross(x,tilt(3,:)'),acos(tilt(3,:)*x));

quiver3(zeros(3,1),zeros(3,1),zeros(3,1),tilt(:,1),tilt(:,2),tilt(:,3))
pbaspect([1 1 1])
axis equal