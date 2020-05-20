%% Moving Function
% Ryo Takatori
% Calculates satellite moving from one location to another using two
% impluse maneuver

function [end_R,end_V,delta_v,r] = move(r_i,v_i,t,R,V,n,target)
mu = 398600; % Gravitational constant [km^3/s^2]
options = odeset('RelTol',1e-8,'AbsTol',1e-8); % Ode settings
% Two body propagation
tspan = [0 t]; % Time span [s]
state = [R;V]; % State vector
% Two impulse maneuver
[delta_v,delta_v_1,delta_v_2] = twoimpulse(r_i,v_i,n,t); % Delta-V [km/s]
state = [R;V;r_i;delta_v_1]; % State vector
[t_new,new_state] = ode45(@LEOM,tspan,state,options,mu); % ode45
r = [new_state(:,7) new_state(:,8) new_state(:,9)]; % Position vector [km]
% Offset to fix graph
offsetx = ones(length(r),1)*target(1);
offsety = ones(length(r),1)*target(2);
r = [new_state(:,7)+offsetx new_state(:,8)+offsety];
% New state vectors
end_R = new_state(end,1:3)';
end_V = new_state(end,4:6)';
% Plot
figure
plot(r(:,2),r(:,1));
xlabel('y [km]')
ylabel('x [km]')
grid on
end

