%% Load towr rosbags in matlab
% Author: Vivian S. Medeiros
% 25/11/2019

% clc;
% clear;
% close all;

%% Extract the desired 3D vectors from the bag
% filePath = '~/Dropbox/ETH/JFR_2019/videos/27_08_2019/up_45/anymal_wheels_matlab.bag';
% filePath = '~/Dropbox/ETH/JFR_2019/simulations/five_steps/large/anymal_wheels_matlab.bag';
filePath = '../bags/anymal_wheels_matlab.bag';
% filePath = '../bags/step_bear_v1_matlab.bag';
% filePath = '/home/vivian/.ros/towr_trajectory.bag';
bag_all = rosbag(filePath);

t_init = bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t_init T] };
bag = select(bag_all, selectOptions{:});

% base motion
bag_base_pose = select(bag, 'Topic', 'base_pose');
ts_base_pos = timeseries(bag_base_pose);

bag_base_vel  = select(bag, 'Topic', 'base_vel_lin');
ts_base_vel_lin = timeseries(bag_base_vel);

bag_base_vel  = select(bag, 'Topic', 'base_vel_ang');
ts_base_vel_ang = timeseries(bag_base_vel);

bag_base_acc  = select(bag, 'Topic', 'base_acc_lin');
ts_base_acc_lin = timeseries(bag_base_acc);

bag_base_acc  = select(bag, 'Topic', 'base_acc_ang');
ts_base_acc_ang = timeseries(bag_base_acc);

% endeffector motion
bag_foot = select(bag, 'Topic', 'foot_pos_0');
ts_foot_LF = timeseries(bag_foot);
bag_foot_vel = select(bag, 'Topic', 'foot_vel_0');
ts_vel_LF = timeseries(bag_foot_vel);
bag_foot_acc = select(bag, 'Topic', 'foot_acc_0');
ts_acc_LF = timeseries(bag_foot_acc);

bag_foot = select(bag, 'Topic', 'foot_pos_1');
ts_foot_RF = timeseries(bag_foot);
bag_foot_vel = select(bag, 'Topic', 'foot_vel_1');
ts_vel_RF = timeseries(bag_foot_vel);
bag_foot_acc = select(bag, 'Topic', 'foot_acc_1');
ts_acc_RF = timeseries(bag_foot_acc);

bag_foot = select(bag, 'Topic', 'foot_pos_2');
ts_foot_LH = timeseries(bag_foot);
bag_foot_vel = select(bag, 'Topic', 'foot_vel_2');
ts_vel_LH = timeseries(bag_foot_vel);
bag_foot_acc = select(bag, 'Topic', 'foot_acc_2');
ts_acc_LH = timeseries(bag_foot_acc);

bag_foot = select(bag, 'Topic', 'foot_pos_3');
ts_foot_RH = timeseries(bag_foot);
bag_foot_vel = select(bag, 'Topic', 'foot_vel_3');
ts_vel_RH = timeseries(bag_foot_vel);
bag_foot_acc = select(bag, 'Topic', 'foot_acc_3');
ts_acc_RH = timeseries(bag_foot_acc);

% endeffector forces
bag_force = select(bag, 'Topic', 'foot_force_0');
ts_force_LF = timeseries(bag_force);

bag_force = select(bag, 'Topic', 'foot_force_1');
ts_force_RF  = timeseries(bag_force);

bag_force = select(bag, 'Topic', 'foot_force_2');
ts_force_LH  = timeseries(bag_force);

bag_force = select(bag, 'Topic', 'foot_force_3');
ts_force_RH  = timeseries(bag_force);

%% define the plotting range and other additional quantities
t = ts_base_pos.Time; 
dyn_con_idx = rem(t,0.1) < 1e-5;
kyn_con_idx = rem(t,0.08) < 1e-5;

% base motion
base_pos  = [ts_base_pos.Data(:,1), ts_base_pos.Data(:,2), ts_base_pos.Data(:,3)];
base_quat = [ts_base_pos.Data(:,4), ts_base_pos.Data(:,5), ...
             ts_base_pos.Data(:,6), ts_base_pos.Data(:,7)];
[base_yaw, base_pitch, base_roll] = quat2angle(base_quat,'XYZ');

% base velocity
base_vel_lin = [ts_base_vel_lin.Data(:,1), ts_base_vel_lin.Data(:,2), ts_base_vel_lin.Data(:,3)];
base_vel_ang = [ts_base_vel_ang.Data(:,1), ts_base_vel_ang.Data(:,2), ts_base_vel_ang.Data(:,3)];

% base acceleration
base_acc_lin = [ts_base_acc_lin.Data(:,1), ts_base_acc_lin.Data(:,2), ts_base_acc_lin.Data(:,3)];
base_acc_ang = [ts_base_acc_ang.Data(:,1), ts_base_acc_ang.Data(:,2), ts_base_acc_ang.Data(:,3)];

% foot motion
pos_LF = [ts_foot_LF.Data(:,1), ts_foot_LF.Data(:,2), ts_foot_LF.Data(:,3)];
pos_RF = [ts_foot_RF.Data(:,1), ts_foot_RF.Data(:,2), ts_foot_RF.Data(:,3)];
pos_LH = [ts_foot_LH.Data(:,1), ts_foot_LH.Data(:,2), ts_foot_LH.Data(:,3)];
pos_RH = [ts_foot_RH.Data(:,1), ts_foot_RH.Data(:,2), ts_foot_RH.Data(:,3)];

% foot velocity
vel_LF = [ts_vel_LF.Data(:,1), ts_vel_LF.Data(:,2), ts_vel_LF.Data(:,3)];
vel_RF = [ts_vel_RF.Data(:,1), ts_vel_RF.Data(:,2), ts_vel_RF.Data(:,3)];
vel_LH = [ts_vel_LH.Data(:,1), ts_vel_LH.Data(:,2), ts_vel_LH.Data(:,3)];
vel_RH = [ts_vel_RH.Data(:,1), ts_vel_RH.Data(:,2), ts_vel_RH.Data(:,3)];

% foot acceleration
acc_LF = [ts_acc_LF.Data(:,1), ts_acc_LF.Data(:,2), ts_acc_LF.Data(:,3)];
acc_RF = [ts_acc_RF.Data(:,1), ts_acc_RF.Data(:,2), ts_acc_RF.Data(:,3)];
acc_LH = [ts_acc_LH.Data(:,1), ts_acc_LH.Data(:,2), ts_acc_LH.Data(:,3)];
acc_RH = [ts_acc_RH.Data(:,1), ts_acc_RH.Data(:,2), ts_acc_RH.Data(:,3)];

% foot force
force_LF = [ts_force_LF.Data(:,1), ts_force_LF.Data(:,2), ts_force_LF.Data(:,3)];
force_RF = [ts_force_RF.Data(:,1), ts_force_RF.Data(:,2), ts_force_RF.Data(:,3)];
force_LH = [ts_force_LH.Data(:,1), ts_force_LH.Data(:,2), ts_force_LH.Data(:,3)];
force_RH = [ts_force_RH.Data(:,1), ts_force_RH.Data(:,2), ts_force_RH.Data(:,3)];

%% wheels' torque

forces_ee = zeros([size(force_LF),4]);
forces_ee(:,:,1) = force_LF;
forces_ee(:,:,2) = force_RF;
forces_ee(:,:,3) = force_LH;
forces_ee(:,:,4) = force_RH;
pos_ee = zeros([size(pos_LF),4]);
pos_ee(:,:,1) = pos_LF;
pos_ee(:,:,2) = pos_RF;
pos_ee(:,:,3) = pos_LH;
pos_ee(:,:,4) = pos_RH;

n = size(forces_ee,1);
n_ee = 4;
wh_radius = 0.05; % [m]
wh_torque = zeros(size(forces_ee,1),1,4);
for i = 1:n
    for j = 1:n_ee
        x = pos_ee(i,1,j);
        [normal, tx, ty] = GetTerrainBasis(x, "Step45");
        wh_torque(i,:,j) = (forces_ee(i,:,j) * tx) * wh_radius * 1.2;    
    end
end

% wh_torque(:,:,1) -> LF
% wh_torque(:,:,2) -> RF
% wh_torque(:,:,3) -> LH
% wh_torque(:,:,4) -> RH
