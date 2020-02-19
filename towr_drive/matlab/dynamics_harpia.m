% Harpia Dynamics

clear; clc; close all;

% considering straight motion in an inclined terrain with slope alpha
syms ax Ftt Ftd Nt Nd alpha_y F N real

% terrain
theta = 10*pi/180;
y = [0; 1; 0];
n = [-sin(theta); 0; cos(theta)];
t = cross(y,n);
t = t/norm(t);

xyz = [0 -theta 0];
w_R_b = GetRotationMatrixBaseToWorld(xyz);
b_R_w = inv(w_R_b);

% robot parameters
Ixx =  1.35484745362;
Iyy =  0.51559401168;
Izz =  1.81519958621;
Ixy = -0.00300508233; 
Ixz = -0.00238036472;
Iyz = -0.00425512338;

m = 27.24;
J = [Ixx Ixy Ixz; 
     Ixy Iyy Iyz;
     Ixz Iyz Izz];  % inertia
n_ee = 4;
g = 9.81; % gravity acceleration

x_b =  0.0849;
y_b =  0.0; %0.2975;
z_b =  0.0836;
r_wh = 0.0762;

% static condition (2D)
% Ft = [Ftt; 0; Nt]
% Fd = [Ftd; 0; Nd]
Ft = [F; 0; Nt]
Fd = [F; 0; Nd]

eq_lin = Ft + Fd - b_R_w*m*[0; 0; g] - m*[ax; 0; 0]


pd = [x_b; 0; -z_b];
pt = [-x_b; 0; -z_b];

eq_rot = cross(Fd,pd) + cross(Ft,pt) - J*[0; 0; 0]

a_max = g*(x_b*cos(theta)-z_b*sin(theta))/z_b
a_max = 5.0;
sys = [eq_lin(1), eq_lin(3), eq_rot(2)]
sys = subs(sys, ax, a_max)

sol = solve(sys)
F_sol = double(sol.F)
Nd_sol = double(sol.Nd)
Nt_sol = double(sol.Nt)

mu = 0.5;
mu*Nd_sol
mu*Nt_sol
