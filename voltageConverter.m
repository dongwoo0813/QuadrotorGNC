function [eak] = voltageConverter(Fk,NBk,P)
% voltageConverter : Generates output voltages appropriate for desired
%                    torque and thrust.
%
%
% INPUTS
%
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% eak -------- Commanded 4x1 voltage vector to be applied at time tk, in
%              volts. eak(i) is the voltage for the ith motor.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Edward Jung
%+==============================================================================+  

beta = 0.9;
% To be controlled
alpha = 0.95;

% Parameter
cm = P.quadParams.cm(1);
kF = P.quadParams.kF;
kN = P.quadParams.kN;

IU = P.quadParams.rotor_loc;
kT = kN./kF;
G = [1 1 1 1; IU(2,1) IU(2,2) IU(2,3) IU(2,4); -IU(1,1) -IU(1,2) -IU(1,3) -IU(1,4); -kT(1) kT(2) -kT(3) kT(4)];

Ginv = G\eye(4);

F = sum(Fk);
Fmax = kF(1)*(cm*P.quadParams.eamax)^2;
Fik = Ginv*[min(F, 4*beta*Fmax); alpha*NBk]; % 4x1 vector

for IZONE = 1:4;
    if Fik(IZONE) < 0
        Fik(IZONE) = 0;
    elseif Fik(IZONE) > Fmax
        error('The force Fi is exceeded the maximum force capacity.')
    else
        Fik(IZONE) = Fik(IZONE);
    end
end

omegaik = zeros(4,1);
eak = zeros(4,1);
for TWICE = 1:4
    omega(TWICE) = sqrt(Fik(TWICE)/kF(1));
    eak(TWICE) = omega(TWICE)/cm;
end



  

