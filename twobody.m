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