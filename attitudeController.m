function [NBk] = attitudeController(R,S,P)
% attitudeController : Controls quadcopter toward a reference attitude
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       zIstark = 3x1 desired body z-axis direction at time tk, expressed as a
%                 unit vector in the I frame.
%
%       xIstark = 3x1 desired body x-axis direction, expressed as a
%                 unit vector in the I frame.
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
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Edward Jung
%+==============================================================================+  
b_Vec = cross(R.zIstark,R.xIstark)/norm(cross(R.zIstark,R.xIstark));
a_Vec = cross(b_Vec,R.zIstark);
RBIstark = [a_Vec, b_Vec, R.zIstark]';

% K = diag([1 1 0.5]);
K = diag([1 1 0.2]);
% K matrix of Attitude Controller
% Kd = diag([0.28 0.2 0.12]); % Kd matrix of Attitude Controller
Kd = diag([0.28 0.2 0.04]); 
RE = RBIstark*S.statek.RBI';

% eE = dcm2euler(RE);

eE = [RE(2,3)-RE(3,2); RE(3,1)-RE(1,3); RE(1,2)-RE(2,1)];

Jq = P.quadParams.Jq;
omegaB = S.statek.omegaB;
NBk = K*eE - Kd*omegaB + crossProductEquivalent(omegaB)*Jq*omegaB;




  

