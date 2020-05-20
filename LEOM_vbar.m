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

