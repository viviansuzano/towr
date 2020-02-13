function vel_ee_C = RotationWorldToContactFrame (base_ang, vel_ee_W)

% It assumes flat terrain for now
n = size(base_quat,1);
vel_ee_C = zeros(size(vel_ee_W));

for i = 1:n
 w_R_b = eul2rotm(base_ang(i,:));
 n = [0; 0; 1];
 heading = w_R_b * [1; 0; 0];
 ey = cross(n, heading);
 ex = cross(ey, n);
 ez = n;
 
 c_R_w = [ex, ey, ez];
 
 vel_ee_C(i,:) = (c_R_w * vel_ee_W(i,:)')';
 
end

end