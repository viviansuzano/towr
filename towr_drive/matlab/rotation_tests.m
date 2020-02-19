% Tests with the wheels coordinate frame

clc; clear; close all;

% v_c = [0.5 0 0]';
v_w = [0.5 0 0]';

xyz = [0 0 20]*pi/180;
w_R_b = GetRotationMatrixBaseToWorld(xyz);
b_R_w = inv(w_R_b);

% v_b = b_R_w * v_w

% n = [-2 0 1]';
% n = [0 0 1]';
% n = [1 0 1]';
[n, tx, ty] = GetTerrainBasis(1.05, "Flat"); %"Step")
n = n/norm(n)
heading = w_R_b * [1 0 0]';
ey = cross(n,heading); %ey = ey / norm(ey);
ex = cross(ey,n);      %ex = ex / norm(ex);
ez = cross(ex,ey);     %ez = ez / norm(ez);
w_R_c = [ex/norm(ex) ey/norm(ey) ez/norm(ez)]
c_R_w = w_R_c';
quat = rotm2quat(w_R_c);
% v_w = w_R_c * v_c
v_c = c_R_w * v_w

% xyz = [0 -atan(2) 0];
% w_R_c = GetRotationMatrixBaseToWorld(xyz)
% v_w = w_R_c * v_c
% w_R_c' * v_w;

%% wheels motion constraint

ang = 20*pi/180;
xyz = [0 -ang 0];
w_R_b = GetRotationMatrixBaseToWorld(xyz);

x_nominal_b =  0.0849;
y_nominal_b =  0.2975;
z_nominal_b =  -0.0074; %0.03175;

r_wh = 0.0762;

wheels_center_B = [ x_nominal_b, -y_nominal_b, z_nominal_b;
                   -x_nominal_b, -y_nominal_b, z_nominal_b];
               
n = w_R_b*[0; 0; 1];
% base_pos_W = [0.4, 0.0, 0.19004];
% base_pos_W = [0.58794, 0.0, 0.25844];
%base_pos_W = [0.0, 0.0, 0.04445];
base_pos_W = [0.0, 0.0, 0.0836];
wheels_center_W = base_pos_W' + w_R_b*wheels_center_B';
wheels_contact_W = wheels_center_W - r_wh*n;

figure();
t = -0.5:0.01:0.5; plot(t, tand(20)*t); hold on;
plot(base_pos_W(1),base_pos_W(3),'r*'); hold on;
plot(wheels_center_W(1,1),wheels_center_W(3,1),'g*'); hold on;
plot(wheels_center_W(1,2),wheels_center_W(3,2),'g*'); hold on;
t = 0:0.01:2*pi;
plot(r_wh*cos(t)+wheels_center_W(1,1), r_wh*sin(t)+wheels_center_W(3,1), 'r'); hold on;
plot(r_wh*cos(t)+wheels_center_W(1,2), r_wh*sin(t)+wheels_center_W(3,2), 'r'); hold on;
body = robot_body_pos([base_pos_W(1) base_pos_W(3)],ang,0.4,0.0782);
plot(body(1,:),body(2,:),'b'); hold on;
plot(wheels_contact_W(1,1),wheels_contact_W(3,1),'k*'); hold on;
plot(wheels_contact_W(1,2),wheels_contact_W(3,2),'k*'); hold on;
axis equal
