function [Fk,zIstark] = trajectoryController(R,S,P)
% trajectoryController : Controls quadcopter toward a reference trajectory.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       rIstark = 3x1 vector of desired CM position at time tk in the I frame,
%                 in meters.
%
%       vIstark = 3x1 vector of desired CM velocity at time tk with respect to
%                 the I frame and expressed in the I frame, in meters/sec.
%
%       aIstark = 3x1 vector of desired CM acceleration at time tk with
%                 respect to the I frame and expressed in the I frame, in
%                 meters/sec^2.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
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
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% zIstark ---- Desired 3x1 body z axis expressed in I frame at time tk.    
%                  
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  
% k= diag([10 10 10]); % K matrix of Trajectory Controller
% kd= diag([10 5 5]); % Kd matrix of Trajectory Controller
k= diag([10 10 12]); % K matrix of Trajectory Controller
kd= diag([10 5 3]); % Kd matrix of Trajectory Controller


mass = P.quadParams.m;
g = P.constants.g;

% eP=R.rIstark-S.statek.rI
% eD=R.vIstark-S.statek.vI

FIstark = k*(R.rIstark - S.statek.rI) + kd*(R.vIstark - S.statek.vI) + mass*g*[0;0;1] + mass*R.aIstark;
zIstark = FIstark/norm(FIstark);
Fk = (FIstark)'*S.statek.RBI'*[0; 0; 1];

  

