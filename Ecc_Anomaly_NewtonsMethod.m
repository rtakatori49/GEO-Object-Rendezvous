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

