%% Kepler's Equation by Newton's Method
% Ryo Takatori

function [ E ] = keplerE( e, M )
if M < pi
    E = M + e/2;
else
    E = M - e/2;
end
ratio = 2;
error = 10^-8;
i = 0;
while error < abs(ratio)
    ratio = (M - E + e*sin(E))/(1 - e*cos(E));
    E = E + ratio;
    i = i + 1;
    if i == 10000
        break
    end
end
end

