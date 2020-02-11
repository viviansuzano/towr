%% Plots for thesis
% Author: Vivian S. Medeiros
% 11/04/2019

clc;
clear;
% close all;

%% Extract the desired 3D vectors from the bag
filePath = '../../towr_ros/bags/anymal_wheels_matlab.bag';
bag_all = rosbag(filePath);

t0 = bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
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
t = ts_base_pos.Time;
% end_idx = length(t);
end_idx = find(abs(t-1.65) < 0.001);
idx = 1:end_idx;
t = ts_base_pos.Time(idx); 
dyn_con_idx = rem(t,0.1) < 1e-5;
kyn_con_idx = rem(t,0.08) < 1e-5;

% base motion
base_pos  = [ts_base_pos.Data(idx,1), ts_base_pos.Data(idx,2), ts_base_pos.Data(idx,3)];
base_quat = [ts_base_pos.Data(idx,4), ts_base_pos.Data(idx,5), ...
             ts_base_pos.Data(idx,6), ts_base_pos.Data(idx,7)];
[base_yaw, base_pitch, base_roll] = quat2angle(base_quat,'XYZ');
base_roll = abs(base_roll);

% base velocity
base_vel_lin = [ts_base_vel_lin.Data(idx,1), ts_base_vel_lin.Data(idx,2), ts_base_vel_lin.Data(idx,3)];
base_vel_ang = [ts_base_vel_ang.Data(idx,1), ts_base_vel_ang.Data(idx,2), ts_base_vel_ang.Data(idx,3)];

% base acceleration
base_acc_lin = [ts_base_acc_lin.Data(idx,1), ts_base_acc_lin.Data(idx,2), ts_base_acc_lin.Data(idx,3)];
base_acc_ang = [ts_base_acc_ang.Data(idx,1), ts_base_acc_ang.Data(idx,2), ts_base_acc_ang.Data(idx,3)];

% foot motion
pos_LF = [ts_foot_LF.Data(idx,1), ts_foot_LF.Data(idx,2), ts_foot_LF.Data(idx,3)];
pos_RF = [ts_foot_RF.Data(idx,1), ts_foot_RF.Data(idx,2), ts_foot_RF.Data(idx,3)];
pos_RF(t<=0.076,3) = zeros(31,1);  % fix small values that look strange on the plot
pos_LH = [ts_foot_LH.Data(idx,1), ts_foot_LH.Data(idx,2), ts_foot_LH.Data(idx,3)];
pos_RH = [ts_foot_RH.Data(idx,1), ts_foot_RH.Data(idx,2), ts_foot_RH.Data(idx,3)];

% foot velocity
vel_LF = [ts_vel_LF.Data(idx,1), ts_vel_LF.Data(idx,2), ts_vel_LF.Data(idx,3)];
vel_RF = [ts_vel_RF.Data(idx,1), ts_vel_RF.Data(idx,2), ts_vel_RF.Data(idx,3)];
vel_LH = [ts_vel_LH.Data(idx,1), ts_vel_LH.Data(idx,2), ts_vel_LH.Data(idx,3)];
vel_RH = [ts_vel_RH.Data(idx,1), ts_vel_RH.Data(idx,2), ts_vel_RH.Data(idx,3)];

% foot acceleration
acc_LF = [ts_acc_LF.Data(idx,1), ts_acc_LF.Data(idx,2), ts_acc_LF.Data(idx,3)];
acc_RF = [ts_acc_RF.Data(idx,1), ts_acc_RF.Data(idx,2), ts_acc_RF.Data(idx,3)];
acc_LH = [ts_acc_LH.Data(idx,1), ts_acc_LH.Data(idx,2), ts_acc_LH.Data(idx,3)];
acc_RH = [ts_acc_RH.Data(idx,1), ts_acc_RH.Data(idx,2), ts_acc_RH.Data(idx,3)];

% foot force
force_LF = [ts_force_LF.Data(idx,1), ts_force_LF.Data(idx,2), ts_force_LF.Data(idx,3)];
force_RF = [ts_force_RF.Data(idx,1), ts_force_RF.Data(idx,2), ts_force_RF.Data(idx,3)];
force_LH = [ts_force_LH.Data(idx,1), ts_force_LH.Data(idx,2), ts_force_LH.Data(idx,3)];
force_RH = [ts_force_RH.Data(idx,1), ts_force_RH.Data(idx,2), ts_force_RH.Data(idx,3)];

% foot contact
contact_LF = ts_contact_LF.Data(idx,1);
contact_RF = ts_contact_RF.Data(idx,1);
contact_LH = ts_contact_LH.Data(idx,1);
contact_RH = ts_contact_RH.Data(idx,1);

% pos phase durations
[dur_pos_LF, idx_pos_LF] = build_time_vec(ts_dur_pos_LF{1,1}.Data(:,1),t);
[dur_pos_RF, idx_pos_RF] = build_time_vec(ts_dur_pos_RF{1,1}.Data(:,1),t);
[dur_pos_LH, idx_pos_LH] = build_time_vec(ts_dur_pos_LH{1,1}.Data(:,1),t);
[dur_pos_RH, idx_pos_RH] = build_time_vec(ts_dur_pos_RH{1,1}.Data(:,1),t); 
idx_pos_LF = unique(idx_pos_LF);
idx_pos_RF = unique(idx_pos_RF);
idx_pos_LH = unique(idx_pos_LH);
idx_pos_RH = unique(idx_pos_RH);

% force phase durations
[dur_force_LF, idx_force_LF] = build_time_vec(ts_dur_force_LF{1,1}.Data(:,1),t);
[dur_force_RF, idx_force_RF] = build_time_vec(ts_dur_force_RF{1,1}.Data(:,1),t);
[dur_force_LH, idx_force_LH] = build_time_vec(ts_dur_force_LH{1,1}.Data(:,1),t);
[dur_force_RH, idx_force_RH] = build_time_vec(ts_dur_force_RH{1,1}.Data(:,1),t); 
idx_force_LF = unique(idx_force_LF);
idx_force_RF = unique(idx_force_RF);
idx_force_LH = unique(idx_force_LH);
idx_force_RH = unique(idx_force_RH);

%% plot the values

lineWidth = 1.5;
labelFont = 25;
markerSize = 6;

% base
figure()
subplot(3,1,1)
plot(t,base_pos(:,1),'LineWidth',lineWidth); hold on; grid on; 
plot(t(dyn_con_idx),base_pos(dyn_con_idx,1),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
ax = gca;
ax.FontSize = 14;
% xticks(t(dyn_con_idx))
title('Base Linear Position');
xlabel('t [s]'); 
ylabel('x [m]')
ylabel('$$ x [m]$$','Interpreter','Latex','FontSize',labelFont);

subplot(3,1,2)
plot(t,base_pos(:,2),'LineWidth',lineWidth); hold on; grid on;
plot(t(dyn_con_idx),base_pos(dyn_con_idx,2),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
ax = gca;
ax.FontSize = 14;
% xticks(t(dyn_con_idx))
xlabel('t [s]'); 
ylabel('$$ y [m]$$','Interpreter','Latex','FontSize',labelFont);

subplot(3,1,3)
plot(t,base_pos(:,3),'LineWidth',lineWidth); hold on; grid on;
plot(t(dyn_con_idx),base_pos(dyn_con_idx,3),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
ax = gca;
ax.FontSize = 14;
% xticks(t(dyn_con_idx))
xlabel('t [s]'); 
ylabel('$$ z [m]$$','Interpreter','Latex','FontSize',labelFont);

% foot motion
lineWidth = 1.5;
h = figure();
set(h, 'Name', 'Wheels position');
subplot(2,1,1); 
plot(t,pos_RF(:,1),'LineWidth',lineWidth); grid on;
hold on; plot(t(idx_pos_RF), pos_RF(idx_pos_RF,1),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
title('RF Wheel position');
ax = gca;
ax.FontSize = 14;
xticks(t(idx_pos_RF))
set(gca,'YTick',[]);
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
xlabel('t [s]');
ylabel('$$ p_x [m]$$','Interpreter','Latex','FontSize',labelFont);

subplot(2,1,2); 
plot(t,pos_RF(:,3),'LineWidth',lineWidth); grid on;
hold on; plot(t(idx_pos_RF), pos_RF(idx_pos_RF,3),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
ax = gca;
ax.FontSize = 14;
xticks(t(idx_pos_RF))
set(gca,'YTick',[]);
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
xlabel('t [s]');
ylabel('$$ p_z [m]$$','Interpreter','Latex','FontSize',labelFont);

% foot forces
lineWidth = 1.5;
h = figure();
set(h, 'Name', 'Wheels force');
subplot(2,1,1);  
plot(t,force_RF(:,1),'LineWidth',lineWidth); grid on;
hold on; plot(t(idx_force_RF), force_RF(idx_force_RF,1),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
title('RF Wheel force');
ax = gca;
ax.FontSize = 14;
xticks(t(idx_force_RF))
set(gca,'YTick',[]);
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
xlabel('t [s]');
ylabel('$$ f_x [N]$$','Interpreter','Latex','FontSize',labelFont);

subplot(2,1,2); 
plot(t,force_RF(:,3),'LineWidth',lineWidth); grid on;
hold on; plot(t(idx_force_RF), force_RF(idx_force_RF,3),'o','MarkerSize',markerSize,'Marker','o','MarkerFaceColor','red');
ax = gca;
ax.FontSize = 14;
xticks(t(idx_force_RF))
set(gca,'YTick',[]);
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
xlabel('t [s]');
ylabel('$$ f_z [N]$$','Interpreter','Latex','FontSize',labelFont);



