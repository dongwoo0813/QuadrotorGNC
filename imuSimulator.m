function [fB,omegaBtilde] = imuSimulator(S,P)
% imuSimulator : Simulates IMU measurements of specific force and
%                body-referenced angular rate.
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
%
%                  vI = 3x1 velocity of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%
%                  aI = 3x1 acceleration of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second^2.
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%           omegaBdot = 3x1 time derivative of omegaB, in radians per
%                       second^2.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% fB --------- 3x1 specific force measured by the IMU's 3-axis accelerometer
%
% omegaBtilde  3x1 angular rate measured by the IMU's 3-axis rate gyro
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  
RBI = S.statek.RBI;
aI = S.statek.aI;
omegaB = S.statek.omegaB;
g = P.constants.g;

alphaa = P.sensorParams.alphaa;
alphag = P.sensorParams.alphag;
Qa = P.sensorParams.Qa;
Qa2 = P.sensorParams.Qa2;
Qg = P.sensorParams.Qg;
Qg2 = P.sensorParams.Qg2;

va = mvnrnd(zeros(3,1), Qa)';
va2 = mvnrnd(zeros(3,1), Qa2)';
vg = mvnrnd(zeros(3,1), Qg)';
vg2 = mvnrnd(zeros(3,1), Qg2)';

sensorParams = P.sensorParams;

persistent ba bg
if(isempty(ba))
% Set ba's initial value
QbaSteadyState = sensorParams.Qa2/(1 - sensorParams.alphaa^2);
ba = mvnrnd(zeros(3,1), QbaSteadyState)';
end

if(isempty(bg))
% Set bg's initial value
QbgSteadyState = sensorParams.Qg2/(1 - sensorParams.alphag^2);
bg = mvnrnd(zeros(3,1), QbgSteadyState)';
end

ba = alphaa*ba + va2;
bg = alphag*bg + vg2;

fB = RBI*(aI + g*[0;0;1])+ ba + va;

omegaBtilde = omegaB + bg + vg;