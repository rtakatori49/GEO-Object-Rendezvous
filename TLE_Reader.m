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
% theta = 0;
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

