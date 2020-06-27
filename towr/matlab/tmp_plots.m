% wheel LF
figure()
set(gcf, 'Position', [20, 300, 1200, 450]) % [x, y, width, height]
subplot(2,1,1)
plot(t,vel_LF(:,1),'LineWidth',2); hold on;
plot(t(idx_pos_LF),vel_LF(idx_pos_LF,1),'r*','LineWidth',2);
amp = max(vel_LF(:,1)) - min(vel_LF(:,1));
contact = contact_LF*amp-abs(min(vel_LF(:,1)));
% pl = plot(t, contact,'Color', [0.15, 0.15, 0.15]);
% pl.Color (4) = 0.15;
ylim([min(contact) max(contact)]);
xlim([0 t(idx_pos_LF(end))]);
xticks(t(idx_pos_LF))
xtickformat('%.2f')
grid on; xlabel('t [s]'); ylabel('v_x [m/s^2]')
title('Wheel LF Linear Velocity')
ax = gca;
ax.FontSize = 14;
labels = string(ax.XAxis.TickLabels); % extract
labels(2:2:end) = nan; % remove every other one
ax.XAxis.TickLabels = labels; % set

subplot(2,1,2)
plot(t,acc_LF(:,1),'LineWidth',2); hold on;
plot(t(idx_pos_LF),acc_LF(idx_pos_LF,1),'r*','LineWidth',2);
amp = max(acc_LF(:,1)) - min(acc_LF(:,1));
contact = contact_LF*amp-abs(min(acc_LF(:,1))); hold on; 
% pl = plot(t, contact,'Color', [0.15, 0.15, 0.15]);
% pl.Color (4) = 0.15;
ylim([min(contact) max(contact)]);
xlim([0 t(idx_pos_LF(end))]);
xticks(t(idx_pos_LF))
xtickformat('%.2f')
grid on; xlabel('t [s]'); ylabel('a_x [m/s^2]')
title('Wheel LF Linear Acceleration')
ax = gca;
ax.FontSize = 14;
labels = string(ax.XAxis.TickLabels); % extract
labels(2:2:end) = nan; % remove every other one
ax.XAxis.TickLabels = labels; % set

% ticks_original = ax.XAxis.TickValues;
% ticks = [ax.XAxis.TickValues t(idx_pos_LF)'];
% [ticks_sorted, I] = sort(ticks);
% ax.XAxis.TickValues = ticks_sorted;
% labels = string(ax.XAxis.TickLabels); % extract
% labels(I>size(ticks_original,2)) = nan; % remove every other one
% ax.XAxis.TickLabels = labels; % set

% wheel LF (z-direction)
figure()
set(gcf, 'Position', [20, 300, 1200, 675]) % [x, y, width, height]

subplot(3,1,1)
plot(t,pos_LF(:,3),'LineWidth',2); hold on;
plot(t(idx_pos_LF),pos_LF(idx_pos_LF,3),'r*','LineWidth',2);
amp = max(pos_LF(:,3)) - min(pos_LF(:,3));
contact = contact_LF*amp-abs(min(pos_LF(:,3)));
% pl = plot(t, contact,'Color', [0.15, 0.15, 0.15]);
% pl.Color (4) = 0.15;
ylim([min(contact) max(contact)]);
xlim([0 t(idx_pos_LF(end))]);
xticks(t(idx_pos_LF))
xtickformat('%.2f')
grid on; xlabel('t [s]'); ylabel('p_z [m/s^2]')
title('Wheel LF Linear Position in Z')
ax = gca;
ax.FontSize = 14;
labels = string(ax.XAxis.TickLabels); % extract
labels(2:2:end) = nan; % remove every other one
ax.XAxis.TickLabels = labels; % set

subplot(3,1,2)
plot(t,vel_LF(:,3),'LineWidth',2); hold on;
plot(t(idx_pos_LF),vel_LF(idx_pos_LF,3),'r*','LineWidth',2);
amp = max(vel_LF(:,3)) - min(vel_LF(:,3));
contact = contact_LF*amp-abs(min(vel_LF(:,3)));
% pl = plot(t, contact,'Color', [0.15, 0.15, 0.15]);
% pl.Color (4) = 0.15;
ylim([min(contact) max(contact)]);
xlim([0 t(idx_pos_LF(end))]);
xticks(t(idx_pos_LF))
xtickformat('%.2f')
grid on; xlabel('t [s]'); ylabel('v_z [m/s^2]')
title('Wheel LF Linear Velocity in Z')
ax = gca;
ax.FontSize = 14;
labels = string(ax.XAxis.TickLabels); % extract
labels(2:2:end) = nan; % remove every other one
ax.XAxis.TickLabels = labels; % set

subplot(3,1,3)
plot(t,acc_LF(:,3),'LineWidth',2); hold on;
plot(t(idx_pos_LF),acc_LF(idx_pos_LF,3),'r*','LineWidth',2);
amp = max(acc_LF(:,3)) - min(acc_LF(:,3));
contact = contact_LF*amp-abs(min(acc_LF(:,3))); hold on; 
% pl = plot(t, contact,'Color', [0.15, 0.15, 0.15]);
% pl.Color (4) = 0.15;
ylim([min(contact) max(contact)]);
xlim([0 t(idx_pos_LF(end))]);
xticks(t(idx_pos_LF))
xtickformat('%.2f')
grid on; xlabel('t [s]'); ylabel('a_z [m/s^2]')
title('Wheel LF Linear Acceleration in Z')
ax = gca;
ax.FontSize = 14;
labels = string(ax.XAxis.TickLabels); % extract
labels(2:2:end) = nan; % remove every other one
ax.XAxis.TickLabels = labels; % set