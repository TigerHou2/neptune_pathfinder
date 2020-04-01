%%
clear;clc

syms y(t)
syms k q y0 real

% ode = diff(y,t) == k*y^2*exp(-(h0+int(y,t))/H);
% ode = diff(y,t) == y^2*(q-k*int(y,t));
% ode = diff(y,t,2) == k*y^3 + 2/y * diff(y,t)^2;
ode = diff(y,t,2) == k*y^3;
cond = y(0) == y0;

ySol(t) = dsolve(ode,cond)

%%
clear;clc

m = 3300;
Cl = 0.4;
LoD = 0.25;
Cd = Cl / LoD;
A = pi*(4.5/2)^2;
h_0 = 500e3;
rho_0 = 1.4e-5;

g = 12.01;

t0 = 0;
t1 = 1;

qq = -1/2/m*Cl*A*h_0*rho_0;
kk = 1/2/m*Cl*A*rho_0;

tf = 200;
dt = tf-t0;

vy0 = -4e3;
vy1 = -4.01e3;

a = 0.9;
b = 1-a;

syms C1 C2 real
eqn1 = vy0/a == 1 / (kk*t0^2 + C1*t0 + C2) - g*t0;
eqn2 = vy1/a == 1 / (kk*t1^2 + C1*t1 + C2) - g*t1;
soln = solve([eqn1,eqn2],[C1,C2]);
c1 = double(soln.C1)
c2 = double(soln.C2)

syms C3 C4 real
eqn3 = vy0/b == C3 / (C4 + t0) - g*t0;
eqn4 = vy1/b == C4 / (C4 + t1) - g*t1;
soln = solve([eqn3,eqn4],[C3,C4]);
c3 = double(soln.C3)
c4 = double(soln.C4)

tf = linspace(100,300,500);

yf = a ./ (kk.*tf.^2 + c1.*tf + c2) + b .* c3 ./ (c4 + tf) - g.*tf + 24000e3 .* sin(pi.*tf/1000);
plot(tf,yf)