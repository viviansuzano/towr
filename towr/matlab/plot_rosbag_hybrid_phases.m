%% Plot towr rosbags in matlab
% Author: Vivian S. Medeiros
% 11/04/2019

clc;
clear;
close all;

%% Extract the desired 3D vectors from the bag
filePath = '../../towr_ros/bags/anymal_wheels_matlab.bag';
bag_all = rosbag(filePath);

t0 = bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
bag = select(bag_all, selectOptions{:});

% endeffector motion
bag_foot = select(bag, 'Topic', 'foot_pos_0');
ts_foot_LF = timeseries(bag_foot);
% bag_foot_vel = select(bag, 'Topic', 'foot_vel_0');
% ts_vel_LF = timeseries(bag_foot_vel);
% bag_foot_acc = select(bag, 'Topic', 'foot_acc_0');
% ts_acc_LF = timeseries(bag_foot_acc);

bag_foot = select(bag, 'Topic', 'foot_pos_1');
ts_foot_RF = timeseries(bag_foot);
% bag_foot_vel = select(bag, 'Topic', 'foot_vel_1');
% ts_vel_RF = timeseries(bag_foot_vel);
% bag_foot_acc = select(bag, 'Topic', 'foot_acc_1');
% ts_acc_RF = timeseries(bag_foot_acc);

bag_foot = select(bag, 'Topic', 'foot_pos_2');
ts_foot_LH = timeseries(bag_foot);
% bag_foot_vel = select(bag, 'Topic', 'foot_vel_2');
% ts_vel_LH = timeseries(bag_foot_vel);
% bag_foot_acc = select(bag, 'Topic', 'foot_acc_2');
% ts_acc_LH = timeseries(bag_foot_acc);

bag_foot = select(bag, 'Topic', 'foot_pos_3');
ts_foot_RH = timeseries(bag_foot);
% bag_foot_vel = select(bag, 'Topic', 'foot_vel_3');
% ts_vel_RH = timeseries(bag_foot_vel);
% bag_foot_acc = select(bag, 'Topic', 'foot_acc_3');
% ts_acc_RH = timeseries(bag_foot_acc);

% endeffector forces
bag_force = select(bag, 'Topic', 'foot_force_0');
ts_force_LF = timeseries(bag_force);

bag_force = select(bag, 'Topic', 'foot_force_1');
ts_force_RF  = timeseries(bag_force);

bag_force = select(bag, 'Topic', 'foot_force_2');
ts_force_LH  = timeseries(bag_force);

bag_force = select(bag, 'Topic', 'foot_force_3');
ts_force_RH  = timeseries(bag_force);

% endeffector contact state
bag_contact = select(bag, 'Topic', 'foot_contact_0');
ts_contact_LF = timeseries(bag_contact);

bag_contact = select(bag, 'Topic', 'foot_contact_1');
ts_contact_RF  = timeseries(bag_contact);

bag_contact = select(bag, 'Topic', 'foot_contact_2');
ts_contact_LH  = timeseries(bag_contact);

bag_contact = select(bag, 'Topic', 'foot_contact_3');
ts_contact_RH  = timeseries(bag_contact);

% ee polynomials durations
bag_dur_pos_LF = select(bag, 'Topic', 'poly_dur_pos_0');
ts_dur_pos_LF = readMessages(bag_dur_pos_LF);
bag_dur_pos_RF = select(bag, 'Topic', 'poly_dur_pos_1');
ts_dur_pos_RF = readMessages(bag_dur_pos_RF);
bag_dur_pos_LH = select(bag, 'Topic', 'poly_dur_pos_2');
ts_dur_pos_LH = readMessages(bag_dur_pos_LH);
bag_dur_pos_RH = select(bag, 'Topic', 'poly_dur_pos_3');
ts_dur_pos_RH = readMessages(bag_dur_pos_RH);

bag_dur_force_LF = select(bag, 'Topic', 'poly_dur_force_0');
ts_dur_force_LF = readMessages(bag_dur_force_LF);
bag_dur_force_RF = select(bag, 'Topic', 'poly_dur_force_1');
ts_dur_force_RF = readMessages(bag_dur_force_RF);
bag_dur_force_LH = select(bag, 'Topic', 'poly_dur_force_2');
ts_dur_force_LH = readMessages(bag_dur_force_LH);
bag_dur_force_RH = select(bag, 'Topic', 'poly_dur_force_3');
ts_dur_force_RH = readMessages(bag_dur_force_RH);

%% define the plotting range and other additional quantities
t = ts_force_LF.Time; 
dyn_con_idx = rem(t,0.1) < 1e-5;
kyn_con_idx = rem(t,0.08) < 1e-5;

% foot motion
pos_LF = [ts_foot_LF.Data(:,1), ts_foot_LF.Data(:,2), ts_foot_LF.Data(:,3)];
pos_RF = [ts_foot_RF.Data(:,1), ts_foot_RF.Data(:,2), ts_foot_RF.Data(:,3)];
pos_LH = [ts_foot_LH.Data(:,1), ts_foot_LH.Data(:,2), ts_foot_LH.Data(:,3)];
pos_RH = [ts_foot_RH.Data(:,1), ts_foot_RH.Data(:,2), ts_foot_RH.Data(:,3)];

% % foot velocity
% vel_LF = [ts_vel_LF.Data(:,1), ts_vel_LF.Data(:,2), ts_vel_LF.Data(:,3)];
% vel_RF = [ts_vel_RF.Data(:,1), ts_vel_RF.Data(:,2), ts_vel_RF.Data(:,3)];
% vel_LH = [ts_vel_LH.Data(:,1), ts_vel_LH.Data(:,2), ts_vel_LH.Data(:,3)];
% vel_RH = [ts_vel_RH.Data(:,1), ts_vel_RH.Data(:,2), ts_vel_RH.Data(:,3)];
% 
% % foot acceleration
% acc_LF = [ts_acc_LF.Data(:,1), ts_acc_LF.Data(:,2), ts_acc_LF.Data(:,3)];
% acc_RF = [ts_acc_RF.Data(:,1), ts_acc_RF.Data(:,2), ts_acc_RF.Data(:,3)];
% acc_LH = [ts_acc_LH.Data(:,1), ts_acc_LH.Data(:,2), ts_acc_LH.Data(:,3)];
% acc_RH = [ts_acc_RH.Data(:,1), ts_acc_RH.Data(:,2), ts_acc_RH.Data(:,3)];

% foot force
force_LF = [ts_force_LF.Data(:,1), ts_force_LF.Data(:,2), ts_force_LF.Data(:,3)];
force_RF = [ts_force_RF.Data(:,1), ts_force_RF.Data(:,2), ts_force_RF.Data(:,3)];
force_LH = [ts_force_LH.Data(:,1), ts_force_LH.Data(:,2), ts_force_LH.Data(:,3)];
force_RH = [ts_force_RH.Data(:,1), ts_force_RH.Data(:,2), ts_force_RH.Data(:,3)];

% foot contact
contact_LF = ts_contact_LF.Data(:,1);
contact_RF = ts_contact_RF.Data(:,1);
contact_LH = ts_contact_LH.Data(:,1);
contact_RH = ts_contact_RH.Data(:,1);

% pos phase durations
[dur_pos_LF, idx_pos_LF] = build_time_vec(ts_dur_pos_LF{1,1}.Data(:,1),t);
[dur_pos_RF, idx_pos_RF] = build_time_vec(ts_dur_pos_RF{1,1}.Data(:,1),t);
[dur_pos_LH, idx_pos_LH] = build_time_vec(ts_dur_pos_LH{1,1}.Data(:,1),t);
[dur_pos_RH, idx_pos_RH] = build_time_vec(ts_dur_pos_RH{1,1}.Data(:,1),t); 

% force phase durations
[dur_force_LF, idx_force_LF] = build_time_vec(ts_dur_force_LF{1,1}.Data(:,1),t);
[dur_force_RF, idx_force_RF] = build_time_vec(ts_dur_force_RF{1,1}.Data(:,1),t);
[dur_force_LH, idx_force_LH] = build_time_vec(ts_dur_force_LH{1,1}.Data(:,1),t);
[dur_force_RH, idx_force_RH] = build_time_vec(ts_dur_force_RH{1,1}.Data(:,1),t); 

%% plot the values

% foot motion
h = figure();
set(h, 'Name', 'Wheels position');
subplot(3,4,1); plot(t,pos_LF(:,1)); grid on;
hold on; plot(t, contact_LF*max(pos_LF(:,1)));
hold on; plot(t(idx_pos_LF), pos_LF(idx_pos_LF,1),'r*');
xlabel('t [s]'); ylabel('p_x [m]'); title('LF')
subplot(3,4,5); plot(t,pos_LF(:,2)); grid on;
hold on; plot(t, contact_LF*max(pos_LF(:,2)));
hold on; plot(t(idx_pos_LF), pos_LF(idx_pos_LF,2),'r*');
xlabel('t [s]'); ylabel('p_y [m]'); %ylim([0.18 0.2]);
subplot(3,4,9); plot(t,pos_LF(:,3)); grid on;
hold on; plot(t, contact_LF*max(pos_LF(:,3)));
hold on; plot(t(idx_pos_LF), pos_LF(idx_pos_LF,3),'r*');
xlabel('t [s]'); ylabel('p_z [m]')

subplot(3,4,2); plot(t,pos_RF(:,1)); grid on;
hold on; plot(t, contact_RF*max(pos_RF(:,1)));
hold on; plot(t(idx_pos_RF), pos_RF(idx_pos_RF,1),'r*');
xlabel('t [s]'); ylabel('p_x [m]'); title('RF')
subplot(3,4,6); plot(t,pos_RF(:,2)); grid on;
hold on; plot(t, contact_RF*max(pos_RF(:,2)));
hold on; plot(t(idx_pos_RF), pos_RF(idx_pos_RF,2),'r*');
xlabel('t [s]'); ylabel('p_y [m]'); %ylim([-0.2 -0.18]);
subplot(3,4,10); plot(t,pos_RF(:,3)); grid on;
hold on; plot(t, contact_RF*max(pos_RF(:,3)));
hold on; plot(t(idx_pos_RF), pos_RF(idx_pos_RF,3),'r*');
xlabel('t [s]'); ylabel('p_z [m]');

subplot(3,4,3); plot(t,pos_LH(:,1)); grid on;
hold on; plot(t, contact_LH*max(pos_LH(:,1)));
hold on; plot(t(idx_pos_LH), pos_LH(idx_pos_LH,1),'r*');
xlabel('t [s]'); ylabel('p_x [m]'); title('LH')
subplot(3,4,7); plot(t,pos_LH(:,2)); grid on;
hold on; plot(t, contact_LH*max(pos_LH(:,2)));
hold on; plot(t(idx_pos_LH), pos_LH(idx_pos_LH,2),'r*');
xlabel('t [s]'); ylabel('p_y [m]'); %ylim([0.18 0.2]);
subplot(3,4,11); plot(t,pos_LH(:,3)); grid on;
hold on; plot(t, contact_LH*max(pos_LH(:,3)));
hold on; plot(t(idx_pos_LH), pos_LH(idx_pos_LH,3),'r*');
xlabel('t [s]'); ylabel('p_z [m]')

subplot(3,4,4); plot(t,pos_RH(:,1)); grid on;
hold on; plot(t, contact_RH*max(pos_RH(:,1)));
hold on; plot(t(idx_pos_RH), pos_RH(idx_pos_RH,1),'r*');
xlabel('t [s]'); ylabel('p_x [m]'); title('RH')
subplot(3,4,8); plot(t,pos_RH(:,2)); grid on;
hold on; plot(t, contact_RH*max(pos_RH(:,2)));
hold on; plot(t(idx_pos_RH), pos_RH(idx_pos_RH,2),'r*');
xlabel('t [s]'); ylabel('p_y [m]'); %ylim([-0.2 -0.18]);
subplot(3,4,12); plot(t,pos_RH(:,3)); grid on;
hold on; plot(t, contact_RH*max(pos_RH(:,3)));
hold on; plot(t(idx_pos_RH), pos_RH(idx_pos_RH,3),'r*');
xlabel('t [s]'); ylabel('p_z [m]')

h = figure();
terrain = "Block"; % "Block"; %"Gap"; %"Flat"; %"Step45";
set(h, 'Name', 'Wheels position (X x Z)');
subplot(4,1,1); plot(pos_LF(:,1),pos_LF(:,3),pos_LF(:,1),GetTerrainHeight(pos_LF(:,1), terrain)); grid on; %axis equal;
% hold on; plot(pos_LF(idx_pos_LF,1),pos_LF(idx_pos_LF,3),'r*');
xlabel('p_x [m]'); ylabel('p_z [m]'); title('LF')
subplot(4,1,2); plot(pos_RF(:,1),pos_RF(:,3),pos_RF(:,1),GetTerrainHeight(pos_RF(:,1), terrain)); grid on; %axis equal;
% hold on; plot(pos_RF(idx_pos_RF,1),pos_RF(idx_pos_RF,3),'r*');
xlabel('p_x [m]'); ylabel('p_z [m]'); title('RF')
subplot(4,1,3); plot(pos_LH(:,1),pos_LH(:,3),pos_LH(:,1),GetTerrainHeight(pos_LH(:,1), terrain)); grid on; %axis equal;
% hold on; plot(pos_LH(idx_pos_LH,1),pos_LH(idx_pos_LH,3),'r*');
xlabel('p_x [m]'); ylabel('p_z [m]'); title('LH')
subplot(4,1,4); plot(pos_RH(:,1),pos_RH(:,3),pos_RH(:,1),GetTerrainHeight(pos_RH(:,1), terrain)); grid on; %axis equal;
% hold on; plot(pos_RH(idx_pos_RH,1),pos_RH(idx_pos_RH,3),'r*');
xlabel('p_x [m]'); ylabel('p_z [m]'); title('RH')

% foot forces
lineWidth = 1.5;
h = figure();
set(h, 'Name', 'Wheels force');
subplot(3,4,1); plot(t,force_LF(:,1),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_LF*max(force_LF(:,1)));
xlabel('t [s]'); ylabel('f_x [N]'); title('LF')
subplot(3,4,5); plot(t,force_LF(:,2),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_LF*max(force_LF(:,2)));
xlabel('t [s]'); ylabel('f_y [N]')
subplot(3,4,9); %plot(t,force_LF(:,3),'LineWidth',lineWidth); grid on;
hold on; plot(t, GetTerrainHeight(pos_LF(:,1), terrain)); 
hold on; plot(t, contact_LF*0.2); grid on; %max(force_LF(:,3)));
% hold on; plot(t(idx_force_LF), force_LF(idx_force_LF,3),'r*');
xlabel('t [s]'); ylabel('f_z [N]')

subplot(3,4,2); plot(t,force_RF(:,1),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_RF*max(force_RF(:,1)));
xlabel('t [s]'); ylabel('f_x [N]'); title('RF')
subplot(3,4,6); plot(t,force_RF(:,2),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_RF*max(force_RF(:,2)));
xlabel('t [s]'); ylabel('f_y [N]')
subplot(3,4,10); %plot(t,force_RF(:,3),'LineWidth',lineWidth); grid on;
hold on; plot(t, GetTerrainHeight(pos_RF(:,1), terrain)); 
hold on; plot(t, contact_RF*0.2); grid on; %max(force_RF(:,3)));
% hold on; plot(t(idx_force_RF), force_RF(idx_force_RF,3),'r*');
xlabel('t [s]'); ylabel('f_z [N]')

subplot(3,4,3); plot(t,force_LH(:,1),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_LH*max(force_LH(:,1)));
xlabel('t [s]'); ylabel('f_x [N]'); title('LH')
subplot(3,4,7); plot(t,force_LH(:,2),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_LH*max(force_LH(:,2)));
xlabel('t [s]'); ylabel('f_y [N]')
subplot(3,4,11); %plot(t,force_LH(:,3),'LineWidth',lineWidth); grid on;
hold on; plot(t, GetTerrainHeight(pos_LH(:,1), terrain)); 
hold on; plot(t, contact_LH*0.2); grid on; %max(force_LH(:,3)));
% hold on; plot(t(idx_force_LH), force_LH(idx_force_LH,3),'r*');
xlabel('t [s]'); ylabel('f_z [N]')

subplot(3,4,4); plot(t,force_RH(:,1),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_RH*max(force_RH(:,1)));
xlabel('t [s]'); ylabel('f_x [N]'); title('RH')
subplot(3,4,8); plot(t,force_RH(:,2),'LineWidth',lineWidth); grid on;
hold on; plot(t, contact_RH*max(force_RH(:,2)));
xlabel('t [s]'); ylabel('f_y [N]')
subplot(3,4,12); %plot(t,force_RH(:,3),'LineWidth',lineWidth); grid on;
hold on; plot(t, GetTerrainHeight(pos_RH(:,1), terrain)); 
hold on; plot(t, contact_RH*0.2); grid on; %max(force_RH(:,3)));
% hold on; plot(t(idx_force_RH), force_RH(idx_force_RH,3),'r*');
xlabel('t [s]'); ylabel('f_z [N]')



