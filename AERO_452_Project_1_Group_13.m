%% Project 1
% AERO 452
% Ryo Takatori Kevin Banuelos
% 10/10/19

clc, clear all, close all
global i
mu = 398600; % Gravitational constant [km^3/s^2]
e_rad = 6378; % Radius of Earth [km]
options = odeset('RelTol',1e-8,'AbsTol',1e-8); % Ode settings

%% Orbit determination
[Me, n, ecc, inc, RAAN, w, epoch, tle, a, E, theta, h, T, reci_i, veci_i] = TLE_Reader('INTELSAT_18_TLE.txt');
ecc = 0; % Eccentricity set to 0 to make sure everything lines up
t = 24*3600;
tspan = [0 t]; % Time span [s]
state = [reci_i;veci_i]; % State vector
[tnew ,new_state] = ode45(@twobody,tspan,state,options,mu); % ode45

% Plot of orbit
figure
plot3(new_state(:,1),new_state(:,2),new_state(:,3))
hold on
plot3(reci_i(1),reci_i(2),reci_i(3),'o','MarkerSize',20)
title('INTELSAT 18 Orbit')
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
legend('Orbit','Starting Point')
grid on

%% 100 [km] => 40 [km]
target_100_40 = [0 40 0]'; % Target vector [km]
r_i_100_40 = [0 60 0]'; % Difference between current and next location [km]
v_i_100_40 = [0 0 0]'; % Relative velocity [km/s]
t_100_40=24*60*60; % Transfer time [s]
% Moving function
[end_R_100_40,end_V_100_40,delta_v_100_40,r_100_40] = move(r_i_100_40,v_i_100_40,t_100_40,reci_i,veci_i,n,target_100_40);
title('100~40 [km] Two Impulse Maneuver')

%% 20~40 [km] hold (football)
v_football = (n*40)/2; % Delta-V [km/s]
delta_v_40_hold = v_football*2; % Actual delta-v for getting on and off [km/s]
r_i_40_hold = [0 0 0]'; % Difference between current and next location [km]
v_i_40_hold = [v_football 0 0]'; % Relative velocity [km/s]
t_40_hold=24*60*60; % Hold time [s]
tspan_40_hold = [0 t_40_hold]; % Time span [s]
state_40_hold = [end_R_100_40;end_V_100_40;r_i_40_hold;v_i_40_hold]; % State vector
[t_new_40_hold,new_state_40_hold] = ode45(@LEOM,tspan_40_hold,state_40_hold,options,mu); % ode45
offsety_40_hold = ones(length(new_state_40_hold),1)*40; % Offset to correct graph
r_40_hold = [new_state_40_hold(:,7) new_state_40_hold(:,8)+offsety_40_hold new_state_40_hold(:,9)]; % Position vector [km]

% Plot
figure
plot(r_40_hold(:,2),r_40_hold(:,1))
title('Football Hold for 20~40 [km]')
xlabel('y [km]')
ylabel('x [km]')
ylim([-40 40])
grid on

%% 40 [km] => 1 [km]
target_40_1 = [0 1 0]';  % Target vector [km]
r_i_40_1 = [0 39 0]'; % Difference between current and next location [km]
v_i_40_1 = [0 0 0]'; % Relative velocity [km/s]
t_40_1 = (24*60*60)/2; % Transfer time [s]
% Moving function
[end_R_40_1,end_V_40_1,delta_v_40_1,r_40_1] = move(r_i_40_1,v_i_40_1,t_40_1,new_state_40_hold(end,1:3)',new_state_40_hold(end,4:6)',n,target_40_1);
title('40~1 [km] Two Impulse Maneuver')

%% 1 [km] hold (following on same orbit)
% Two body propagation
t_1_hold = 24*60*60; % Hold time [s]
tspan_1_hold = [0 t_1_hold]; % Time span [s]
state_1_hold = [end_R_40_1;end_V_40_1]; % State vector
[tnew_1_hold,new_state_1_hold] = ode45(@twobody,tspan_1_hold,state_1_hold,options,mu); % ode45

%% 1 [km] => 300 [m]
target_1_300 = [0 300/1000 0]';  % Target vector [km]
r_i_1_300 = [0 700/1000 0]'; % Difference between current and next location [km]
v_i_1_300 = [0 0 0]'; % Relative velocity [km/s]
t_1_300 = (24*60*60)/4; % Transfer time [s]
% Moving function
[end_R_1_300,end_V_1_300,delta_v_1_300,r_1_300] = move(r_i_1_300,v_i_1_300,t_1_300,new_state_1_hold(end,1:3)',new_state_1_hold(end,4:6)',n,target_1_300);
title('1000~300 [m] Two Impulse Maneuver')

%% 300 [m] hold (following on same orbit)
% Two body propagation
t_300_hold = 24*60*60; % Hold time [s]
tspan_300_hold = [0 t_300_hold]; % Time span [s]
state_300_hold = [end_R_1_300;end_V_1_300]; % State vector
[tnew_300_hold ,new_state_300_hold] = ode45(@twobody,tspan_300_hold,state_300_hold,options,mu); % ode45

%% 300 [m] => 20 [m] (Added motion to go 20 [m] above to prepare for R-Bar)
target_300_20 = [-20/1000 0 0]';  % Target vector [km]
r_i_300_20 = [20/1000 300/1000 0]'; % Difference between current and next location [km]
v_i_300_20 = [0 0 0]'; % Relative velocity [km/s]
t_300_20 = (24*60*60)/4; % Transfer time [s]
% Moving function
[end_R_300_20,end_V_300_20,delta_v_300_20,r_300_20] = move(r_i_300_20,v_i_300_20,t_300_20,new_state_300_hold(end,1:3)',new_state_300_hold(end,4:6)',n,target_300_20);
title('300~20 [m] Two Impulse Maneuver')

%% 20 [m] hold (R-Bar)
t_20_hold = 24*60*60; % Hold time [s]
r_i_20_hold = [20/1000 0 0]';  % Difference between current and next location [km]
v_i_20_hold = [0;0;0]; % Relative velocity [km/s]
tspan_20_hold = [0 t_20_hold]; % Time span [s]
state_20_hold = [end_R_300_20;end_V_300_20;r_i_20_hold;v_i_20_hold]; % State vector
[tnew_20_hold_1,new_state_20_hold_1] = ode45(@LEOM,tspan_20_hold,state_20_hold,options,mu); % ode45
r_20_hold_1 = [new_state_20_hold_1(:,7) new_state_20_hold_1(:,8) new_state_20_hold_1(:,9)]; % Position vector [km]
v_f_20_hold = [new_state_20_hold_1(:,10) new_state_20_hold_1(:,11) new_state_20_hold_1(:,12)]'; % Final velocity [km/s]
delta_v_20_hold = norm(v_f_20_hold-v_i_20_hold);  % Delta-V [km/s]
[tnew_20_hold_2,new_state_20_hold_2] = ode45(@LEOM_rbar,tspan_20_hold,state_20_hold,options,mu); % ode45
r_20_hold_2 = [new_state_20_hold_2(:,7) new_state_20_hold_2(:,8) new_state_20_hold_2(:,9)]; % Position vector [km]

% Plot
figure
plot(r_20_hold_2(:,2),r_20_hold_2(:,1),'.','Markersize',30)
title('R-Bar Hold for 20 [m]')
xlabel('y [km]')
ylabel('x [km]')
grid on
 ylim([-0.1 0.1])
 xlim([-0.1 0.1])

%% 20 [m] => 0 [m] (V-Bar)
% Move back to 20 [m] away
target_20_20 = [0 20/1000 0]'; % Target vector [km]
r_i_20_20 = [-20/1000 -20/1000 0]'; % Difference between current and next location [km]
v_i_20_20 = [0 0 0]'; % Relative velocity [km/s]
t_20_20 = (24*60*60)/2; % Transfer time [s]
% Moving function
[end_R_20_20,end_V_20_20,delta_v_20_20,r_20_20] = move(r_i_20_20,v_i_20_20,t_20_20,new_state_20_hold_2(end,1:3)',new_state_20_hold_2(end,4:6)',n,target_20_20);
title('20~20 [m] Two Impulse Maneuver')

%% Hops
%% 20 [m] => 10 [m]
target_20_10 = [0 10/1000 0]'; % Target vector [km]
r_i_20_10 = [0 10/1000 0]'; % Difference between current and next location [km]
v_i_20_10 = [0 0 0]'; % Relative velocity [km/s]
t_20_10 = (24*60*60)/4; % Transfer time [s]
[end_R_20_10,end_V_20_10,delta_v_20_10,r_20_10] = move(r_i_20_10,v_i_20_10,t_20_10,end_R_20_20,end_V_20_20,n,target_20_10);
title('20~10 [m] Hop')

%% 10 [m] hold (following on same orbit)
% Two body propagation
t_10_hold = (24*60*60)/2; % Hold time [s]
tspan_10_hold = [0 t_10_hold]; % Time span [s]
state_10_hold = [end_R_20_10;end_V_20_10]; % State vector
[tnew_10_hold ,new_state_10_hold] = ode45(@twobody,tspan_10_hold,state_10_hold,options,mu); % ode45

%% 10 [m] => 5 [m]
target_10_5 = [0 5/1000 0]'; % Target vector [km]
r_i_10_5 = [ 0 5/1000 0]'; % Difference between current and next location [km]
v_i_10_5 = [0 0 0]'; % Relative velocity [km/s]
t_10_5 = (24*60*60)/4; % Transfer time [s]
[end_R_10_5,end_V_10_5,delta_v_10_5,r_10_5] = move(r_i_10_5,v_i_10_5,t_10_5,new_state_10_hold(end,1:3)',new_state_10_hold(end,4:6)',n,target_10_5);
title('10~5 [m] Hop')

%% 5 [m] hold (following on same orbit)
% Two body propagation
t_5_hold = (24*60*60)/2; % Hold time [s]
tspan_5_hold = [0 t_5_hold]; % Time span [s]
state_5_hold = [end_R_20_10;end_V_20_10]; % State vector
[tnew_5_hold ,new_state_5_hold] = ode45(@twobody,tspan_5_hold,state_5_hold,options,mu); % ode45

%% Final Approach (V-Bar)
t_5_0 = (24*60*60); % Final approach time [s]
v_c = -5/1000/t_5_0; % Required velocity [km/s]
r_i_5_0 = [0 5/1000 0]'; % Difference between current and next location [km]
v_i_5_0_1 = [0;v_c;0]; % Relative velocity [km/s]
tspan_5_0 = [0 t_5_0]; % Time span [s]
state_5_0_1 = [new_state_5_hold(end,1:3)';new_state_5_hold(end,4:6)';r_i_5_0;v_i_5_0_1]; % State vector
[tnew_5_0_1,new_state_5_0_1] = ode45(@LEOM,tspan_5_0,state_5_0_1,options,mu); % ode45
r_5_0_1 = [new_state_5_0_1(:,7) new_state_5_0_1(:,8) new_state_5_0_1(:,9)]; % Position vector [km]
v_5_0 = [new_state_5_0_1(:,10) new_state_5_0_1(:,11) new_state_5_0_1(:,12)]'; %Final velocity [km/s]
delta_v_5_0 = norm(v_5_0 - v_i_5_0_1); % Delta-v [km/s]
state_5_0_2= [end_R_10_5;end_V_10_5;r_i_5_0;v_i_5_0_1]; % State vector
i = 2;
[tnew_5_0,new_state_5_0] = ode45(@LEOM_vbar,tspan_5_0,state_5_0_2,options,mu,v_c); % ode45
r_5_0_2 = [new_state_5_0(:,7) new_state_5_0(:,8) new_state_5_0(:,9)]; % Position vector [km]

% Plot
figure
plot(r_5_0_2(:,2),r_5_0_2(:,1))
title('V-Bar for Final Approach [m]')
xlabel('y [km]')
ylabel('x [km]')
ylim([-0.1 0.1])
grid on

% Entire trajectory plot
figure
plot(r_100_40(:,2),r_100_40(:,1))
hold on
plot(r_40_hold(:,2),r_40_hold(:,1))
plot(r_40_1(:,2),r_40_1(:,1))
plot(r_40_1(end,2),r_40_1(end,1),'.','Markersize',30)
plot(r_1_300(:,2),r_1_300(:,1))
plot(r_1_300(end,2),r_1_300(end,1),'.','Markersize',30)
plot(r_300_20(:,2),r_300_20(:,1))
plot(r_20_hold_2(:,2),-r_20_hold_2(:,1),'.','Markersize',30)
plot(r_20_20(:,2),r_20_20(:,1))
plot(r_20_10(:,2),r_20_10(:,1))
plot(r_20_10(end,2),r_20_10(end,1),'.','Markersize',30)
plot(r_10_5(:,2),r_10_5(:,1))
plot(r_10_5(end,2),r_10_5(end,1),'.','Markersize',30)
plot(r_5_0_2(:,2),r_5_0_2(:,1))
title('Entire Rendezvous Trajectory')
xlabel('y [km]')
ylabel('x [km]')
legend('100~40 [km] Two Impulse Maneuver','40~20 [km] Football Hold',...
    '40~1 [km] Two Impulse Maneuver','1 [km] Hold','1000~300 [m] Two Impulse Maneuver',...
    '300 [m] Hold','300~20 [m] Two Impulse Maneuver','20 [m] R-Bar Hold',...
    '20~20 [m] Two Impulse Maneuver','20~10 [m] Hop','10 [m] Hold','10~5 [m] Hop',...
    '5 [m] Hold','Final V-Bar Approach','Location','southeast')
grid on

%% Total Delta-V [km/s]
delta_v_total = delta_v_100_40+delta_v_40_hold+delta_v_40_1+delta_v_1_300...
    +delta_v_300_20+delta_v_20_20+delta_v_20_hold+delta_v_20_10+delta_v_10_5+delta_v_5_0;
fprintf('Total Delta-V: %f [m/s]\n\n',delta_v_total*1000)

%% Total Time [Days]
t_total = t_100_40+t_40_hold+t_40_1+t_1_hold+t_1_300+t_300_hold+t_300_20+...
    t_20_hold+t_20_20+t_20_10+t_10_hold+t_10_5+t_5_hold+t_5_0;
fprintf('Total Time: %f [Days]',t_total/(60*60*24))

%% Functions

function [Me, n, ecc, inc, RAAN, w, epoch, title, a, E, theta, h, T, reci, veci] = TLE_Reader(TLE)

% TLE READER
%Takes Two-line-element text data and stores into orbital COEs data

%Constants
mu = 398600; %Gravitational parameter for earth

%Load File Data
fid = fopen(TLE, 'rb');
L1 = fscanf(fid,'%24c%*s',1);
L2 = fscanf(fid,'%d%6d%*c%5d%*3c%*2f%f%f%5d%*c%*d%5d%*c%*d%d%5d',[1,9]);
L3 = fscanf(fid,'%d%6d%f%f%f%f%f%f%f',[1,8]);
fclose(fid);

%Assign Data to COE Variables
title = L1;                     % Title of TLE (MUST BE 24 Characters long)
epoch = L2(1,4)*24*3600;        % Epoch Date and Julian Date Fraction
inc   = L3(1,3);                % Inclination [deg]
RAAN  = L3(1,4);                % Right Ascension of the Ascending Node [deg]
ecc   = L3(1,5)/(10^7);         % Eccentricity
w     = L3(1,6);                % Argument of periapsis [deg]
Me    = L3(1,7);                % Mean anomaly [deg]
n     = L3(1,8);                % Mean motion [Revs per day]

%Additional COE Calculations
n = (n*2*pi)/(24*60*60); % Mean motion [rad/s]
a = (mu/n^2)^(1/3);     % Semi-major axis [km]
[E] = Ecc_Anomaly_NewtonsMethod(ecc, Me); % Eccentric Anomaly
theta = rad2deg(2*atan(sqrt(((1+ecc)/(1-ecc)))*tand(E/2))); % True Anomaly [deg]'
h = sqrt(a*mu*(1-ecc^2)); % Angular momentum
T = 2*pi*sqrt((a^3)/mu); % Period [s]
[ rperi, vperi, reci, veci ] = coe2rv( mu, ecc, h, inc, RAAN, w, theta); %rv vectors
    function [E] = Ecc_Anomaly_NewtonsMethod(ecc, M_e)
        if M_e < pi
            E = M_e + (ecc/2);
        end
        if M_e > pi
            E = M_e - (ecc/2);
        end
        
        ii = 1; %max iterations
        tol = 1;
        while tol > 10^-8
            if ii < 1000
                E = E + ((M_e - E + ecc*sin(E))/(1 - ecc*cos(E)));
            end
            %Precision Check
            tol = ((M_e - E + ecc*sin(E))/(1 - ecc*cos(E)));
            ii = ii + 1;
        end
    end
%% COE to RV
% Ryo Takatori

    function [ rperi, vperi, reci, veci ] = coe2rv( mu, e, h, inc, OMEGA, omega, theta)
        % State vectors in perifocal
        rperi = ((h^2)/mu)*(1/(1+e*cosd(theta)))*[cosd(theta) sind(theta) 0];
        vperi = (mu/h)*[-sind(theta) e+cosd(theta) 0];
        
        % Perifocal to ECI
        [ reci, veci ] = peri2eci( rperi, vperi, omega, OMEGA, inc );
        
    end
%% Perifocal to ECI
% Ryo Takatori
    function [ reci, veci ] = peri2eci( rperi, vperi, omega, OMEGA, inc )
        % Rotate to ECI
        R3omega = [cosd(omega) sind(omega) 0;
            -sind(omega) cosd(omega) 0;
            0 0 1];
        
        R1inc = [1 0 0;
            0 cosd(inc) sind(inc);
            0 -sind(inc) cosd(inc)];
        
        R3OMEGA = [cosd(OMEGA) sind(OMEGA) 0;
            -sind(OMEGA) cosd(OMEGA) 0;
            0 0 1];
        
        Q = (R3omega*R1inc*R3OMEGA)';
        
        reci = Q*rperi';
        veci = Q*vperi';
    end

end

%% Two Body Equation of Motion
function [ dstatedt ] = twobody(t, state, mu )
dx = state(4); % dx [km/s]
dy = state(5); % dy [km/s]
dz = state(6); % dz [km/s]

r = norm([state(1) state(2) state(3)]); % Magnitude of position [km]

% Equation of motion
ddx = -mu*state(1)/r^3; % ddx [km/s^2]
ddy = -mu*state(2)/r^3; % ddy [km/s^2]
ddz = -mu*state(3)/r^3; % ddz [km/s^2]

dstatedt = [dx;dy;dz;ddx;ddy;ddz]; % Return state vector for next step
end

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
% Offset to correct graph
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

%% Linearized Equation of Motion
% Ryo Takatori
% Used for delta_r << R

function [dstatedt] = LEOM(t,state,mu)
%% Chief
x = state(1);
y = state(2);
z = state(3);
r = [x y z];
R = norm(r);
dx = state(4);
dy = state(5);
dz = state(6);
v = [dx dy dz];
h = norm(cross(v,r));
ddx = -mu*x/R^3;
ddy = -mu*y/R^3;
ddz = -mu*z/R^3;
a = [ddx ddy ddz];

% Deputy
delta_x = state(7);
delta_y = state(8);
delta_z = state(9);
delta_x_dot = state(10);
delta_y_dot = state(11);
delta_z_dot = state(12);
delta_x_2dot = (((2*mu)/R^3)+(h^2/R^4))*delta_x-((2*(dot(v,r))*h)/R^4)*delta_y+(2*h/R^2)*delta_y_dot;
delta_y_2dot = -(((mu)/R^3)-(h^2/R^4))*delta_y+((2*(dot(v,r))*h)/R^4)*delta_x-(2*h/R^2)*delta_x_dot;
delta_z_2dot = -(mu/R^3)*delta_z;
dstatedt = [v';a';delta_x_dot;delta_y_dot;delta_z_dot;delta_x_2dot;delta_y_2dot;delta_z_2dot];
end

%% Linearized Equation of Motion for R-Bar Hold
% Ryo Takatori
% Used for delta_r << R

function [dstatedt] = LEOM_rbar(t,state,mu)
%% Chief
x = state(1);
y = state(2);
z = state(3);
r = [x y z];
R = norm(r);
dx = state(4);
dy = state(5);
dz = state(6);
v = [dx dy dz];
h = norm(cross(v,r));
ddx = -mu*x/R^3;
ddy = -mu*y/R^3;
ddz = -mu*z/R^3;
a = [ddx ddy ddz];

% Deputy
delta_x = state(7);
delta_y = state(8);
delta_z = state(9);
delta_x_dot = state(10);
delta_y_dot = state(11);
delta_z_dot = state(12);
delta_x_2dot = (((2*mu)/R^3)+(h^2/R^4))*delta_x-((2*(dot(v,r))*h)/R^4)*delta_y+(2*h/R^2)*delta_y_dot -(3*mu*delta_x)/R^3;
delta_y_2dot = -(((mu)/R^3)-(h^2/R^4))*delta_y+((2*(dot(v,r))*h)/R^4)*delta_x-(2*h/R^2)*delta_x_dot;
delta_z_2dot = -(mu/R^3)*delta_z;
dstatedt = [v';a';delta_x_dot;delta_y_dot;delta_z_dot;delta_x_2dot;delta_y_2dot;delta_z_2dot];
end

%% Linearized Equation of Motion for V-Bar Approach
% Ryo Takatori
% Used for delta_r << R

function [dstatedt] = LEOM_vbar(t,state,mu,v_c)
global i
%% Chief
x = state(1);
y = state(2);
z = state(3);
r = [x y z];
R = norm(r);
dx = state(4);
dy = state(5);
dz = state(6);
v = [dx dy dz];
h = norm(cross(v,r));
ddx = -mu*x/R^3;
ddy = -mu*y/R^3;
ddz = -mu*z/R^3;
a = [ddx ddy ddz];

% Deputy
delta_x = state(7);
delta_y = state(8);
delta_z = state(9);
delta_x_dot = state(10);
delta_y_dot = state(11);
delta_z_dot = state(12);
delta_x_2dot = (((2*mu)/R^3)+(h^2/R^4))*delta_x-((2*(dot(v,r))*h)/R^4)*delta_y+(2*h/R^2)*delta_y_dot-2*sqrt(mu/R^3)*v_c;
if i ==1
    delta_y_2dot = -(((mu)/R^3)-(h^2/R^4))*delta_y+((2*(dot(v,r))*h)/R^4)*delta_x-(2*h/R^2)*delta_x_dot+v_c*t;
else
    delta_y_2dot = -(((mu)/R^3)-(h^2/R^4))*delta_y+((2*(dot(v,r))*h)/R^4)*delta_x-(2*h/R^2)*delta_x_dot;
end
i = i+1;
delta_z_2dot = -(mu/R^3)*delta_z;
dstatedt = [v';a';delta_x_dot;delta_y_dot;delta_z_dot;delta_x_2dot;delta_y_2dot;delta_z_2dot];
end

