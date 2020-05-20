%% Two-impluse Maneuver Based on Matricies
% Ryo Takatori

function [delta_v,delta_v_1,delta_v_2] = twoimpulse(r_i,v_i,n,t)
% Matrices
phi_rr = [4-3*cos(n*t) 0 0;
    6*(sin(n*t)-n*t) 1 0;
    0 0 cos(n*t)];

phi_rv = [(1/n)*sin(n*t) (2/n)*(1-cos(n*t)) 0;
    (2/n)*(cos(n*t)-1) (1/n)*(4*sin(n*t)-3*(n*t)) 0;
    0 0 (1/n)*sin(n*t)];

phi_vr = [3*n*sin(n*t) 0 0;
    6*n*(cos(n*t)-1) 0 0;
    0 0 -n*sin(n*t)];

phi_vv = [cos(n*t) 2*sin(n*t) 0;
    -2*sin(n*t) 4*cos(n*t)-3 0;
    0 0 cos(n*t)];

% Velocity to get on [km/s]
v_plus_2 = phi_rv^-1*-phi_rr*r_i;
delta_v_1 = v_plus_2-v_i;

% Velocity to get off [km/s]
v_f_minus_2 = phi_vr*r_i+phi_vv*v_plus_2;
delta_v_2 = -v_f_minus_2;

% Delta-V [km/s]
delta_v = norm(delta_v_1)+norm(delta_v_2);
end

