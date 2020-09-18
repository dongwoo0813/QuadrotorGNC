function [Xdot] = quadOdeFunctionHF(t,X,eaVec,distVec,P)
% quadOdeFunctionHF : Ordinary differential equation function that models
%                     quadrotor dynamics -- high-fidelity version.  For use
%                     with one of Matlab's ODE solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),...
%                   omegaB',omegaVec']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%        omegaVec = 4x1 vector of rotor angular rates, in rad/sec.
%                   omegaVec(i) is the angular rate of the ith rotor.
%
%    eaVec --- 4x1 vector of voltages applied to motors, in volts.  eaVec(i)
%              is the constant voltage setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the quad's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+


% Initialize dx/dt
Xdot = zeros(22,1);
Fi = zeros(4,1);
Ni = zeros(4,1);

% Rotation Matrix
R_BI = zeros(3,3);
R_BI(:) = X(7:15); % Rotation Matrix in Matrix Form

% Set parameter values.
kF = P.quadParams.kF; kN = P.quadParams.kN; mass = P.quadParams.m;
Jq = P.quadParams.Jq; ri = P.quadParams.rotor_loc; %ith rotor location (3x4)
g = P.constants.g; rho = P.constants.rho;
C_d = P.quadParams.Cd; A_d = P.quadParams.Ad;
Cm = P.quadParams.cm(1); Taum = P.quadParams.taum(1);

% Calculate Xdot
Xdot(1:3) = X(4:6); % Velocity in Inertia Frame
omegaVec = X(19:22);

for er = 1:4
    Fi(er,1) = kF(er,1)*omegaVec(er)^2;
end

if norm(X(4:6)) < 0.001
    vI_u=[0;0;0];
else
    vI_u = X(4:6)/norm(X(4:6),2);
end
f_d = dot(X(4:6),[0; 0; 1])^2; % Function of z_I and v_I; NOT COMPLETE YET
d_a = (1/2)*C_d*A_d*rho*f_d;

Xdot(4:6) = (-d_a*vI_u-[0; 0; mass*g] + R_BI'*[0; 0; sum(Fi)] + distVec)/mass; % Acceleration

R_BI_dot = -crossProductEquivalent(X(16:18))*R_BI;
Xdot(7:15) = R_BI_dot(:); %R_BI dot

Si = [-1; 1; -1; 1];

for rr = 1:4
    Ni(rr,1) = kN(rr,1)*omegaVec(rr)^2;
end

N_B = zeros(3,1); % Empty N_B, Total Torque due to pushing against air in B
for qq = 1:4
    N_B = N_B + Si(qq)*[0; 0; Ni(qq,1)] + cross(ri(:,qq),[0; 0; Fi(qq,1)]);
end
Jq_inv = Jq\eye(3);
Xdot(16:18) = Jq_inv*(N_B - crossProductEquivalent(X(16:18))*Jq*X(16:18)); % Angular Acceleration

for num = 1:4
   Xdot(18+num) = (Cm*eaVec(num)- X(18+num))/Taum;
end