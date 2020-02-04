clc

three    = 0.30+0.05;
lateral  = 0.15+0.05;
diagonal = 0.15+0.05;
walk3e = (three+lateral+three+diagonal+three+lateral+three);
walk3 = (three+lateral+three+diagonal+three+lateral+three+diagonal);
stand_flight = 0.2+0.2;
drive = 0.6;
stand_flight+walk3e+drive;
stand_flight+walk3e+walk3e+drive;

% gap_phase = [0.5, 0.2, 0.2, 0.1, 0.3, 0.1, 0.1]
% total_time = sum(gap_phase)
% 3*gap_phase/total_time


% gap_dist = [0.2, 0.15, 0.25, 0.15, 0.2, 0.1, 0.25, 0.1, 0.1]
gap_dist = [0.2, 0.15, 0.25, 0.15, 0.2, 0.1, 0.25, 0.1, 0.1]
sum(gap_dist)
gap_cont = zeros(size(gap_dist));
for i = 1:length(gap_cont)
    gap_cont(i) = sum(gap_dist(1:i));
end
gap_cont+0.35
gap_cont-0.35
% gap_phase = gap_dist*2
% total_time = sum(gap_phase)
% 3*gap_phase/total_time

% teste do problema em towr
v = [1.54894 0.66383 0.442553 0.66383 0.442553 0.66383 0.774468]
for i = 1:length(v)
    v_cont(i) = sum(v(1:i));
end
v_cont

