% Stability Analysis for Harpia

% run after bag from TOWR is loaded!

% parameters for Harpia

x_nominal_b =  0.0849;
y_nominal_b =  0.2975;
z_nominal_b =  0.03175;

r_wh = 0.0762;

% stability measure
ee_pos_cw = zeros(size(pos_ee,1),size(pos_ee,2),size(pos_ee,3)+1);
ee_pos_cw(:,:,1) = pos_ee(:,:,1);
ee_pos_cw(:,:,2) = pos_ee(:,:,2);
ee_pos_cw(:,:,3) = pos_ee(:,:,4);
ee_pos_cw(:,:,4) = pos_ee(:,:,3);
ee_pos_cw(:,:,5) = pos_ee(:,:,1);

I_b = J;

base_acc = [19 0 0]; % m/s^2

w_R_b = GetRotationMatrixBaseToWorld(base_ang(1,:));
I_w = w_R_b * I_b * w_R_b';
for ee = 1:size(pos_ee,3)
    a = (ee_pos_cw(1,:,ee+1) - ee_pos_cw(1,:,ee))';
    a_n = a/norm(a);
    proj_a = eye(3) - a_n*a_n';
    l = proj_a * (ee_pos_cw(1,:,ee+1) - base_pos(1,:))';
    l_n = l/norm(l);
    f = proj_a * (m*base_acc(1,:)' + m*g);
    n = (eye(3) - proj_a) * (I_w*base_acc_ang(1,:)' + cross(base_vel_ang(1,:)', I_w*base_vel_ang(1,:)'));
    ft = f + cross(l_n,n)/norm(l);
    ft_n = ft/norm(ft);
    d = -l + (l'*ft_n)*ft_n;
    tmp = cross(ft_n,l_n)' * a_n;
    theta(1,ee,1) = sign(tmp) * acos(ft_n'*l_n);  % with angular loads

    f_n = f/norm(f);
    tmp = cross(f_n,l_n)' * a_n;
    theta(1,ee,2) = sign(tmp) * acos(f_n'*l_n);  % without angular loads
    
    (m*base_acc_lin(1,:)' + m*g)'*l;
end 

stab = [theta(:,1,1)' ; theta(:,2,1)'; theta(:,3,1)'; theta(:,4,1)']*180/pi

% kinematics test

kin = base_vel_lin - (vel_LF + vel_RF)*(0.5)

