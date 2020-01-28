function body = robot_body_pos(r,theta,x_dim,y_dim)

    R = [cos(theta) -sin(theta);  sin(theta) cos(theta)];
    body0 = [-x_dim/2 x_dim/2 x_dim/2 -x_dim/2 -x_dim/2; 
             0 0 y_dim y_dim 0];
    body = R*body0 + [r(1); r(2)]; 
    
end