function R = aa_getRotnMat(vec,t)
%AA_GETROTNMAT Summary of this function goes here
%   Detailed explanation goes here

x = vec(1);
y = vec(2);
z = vec(3);
R = [ ...
 cos(t)+x^2*(1-cos(t))  , x*y*(1-cos(t))-z*sin(t) , x*z*(1-cos(t))+y*sin(t) ...
 y*x*(1-cos(t))+z*sin(t), cos(t)+y^2*(1-cos(t))   , y*z*(1-cos(t))-x*sin(t) ...
 z*x*(1-cos(t))-y*sin(t), z*y*(1-cos(t))+x*sin(t) , cos(t)+z^2*(1-cos(t)) ];

end

