function [rPtilde,rStilde,rCtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
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
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%
% OUTPUTS
%
% rPtilde----- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rStilde ---- 3x1 GNSS-measured position of the quad's secondary GNSS
%              antenna, in ECEF coordinates relative to the reference antenna,
%              in meters.
%
% rCtilde ---- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rCtilde is constrained to satisfy norm(rCtilde) = b, where b is
%              the known distance between the two antennas.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

% sensorParamsScript
R_LG = Recef2enu(P.sensorParams.r0G);

%rR_G = P.sensorParams.r0G; % Location of the reference antenna in the Global Frame G

%% Primary Antenna
rA_1 = P.sensorParams.rA(:,1); % Position of the primary antenna in the Body Frame relative to I
rP_I = S.statek.rI + S.statek.RBI'*rA_1; % Position of the Primary antenna in the I frame

R_IG = R_LG; % Can be assumed same.

R_LG_inv = R_LG\eye(3);
R_LG_trans_inv = (R_LG')\eye(3);

R_PL = P.sensorParams.RpL;
R_P = R_LG_inv*R_PL*R_LG_trans_inv; % R_P

w_P = mvnrnd(zeros(3,1), R_P)';

rP_G = R_IG'*rP_I; % Position of the primary antenna in the Global Frame
rP = rP_G;

rPtilde = rP + w_P;

%% Secondary Antenna
rA_2 = P.sensorParams.rA(:,2); % Position of the secondary antenna in the Body Frame relative to I

rS_I = S.statek.rI + S.statek.RBI'*rA_2; % Position of the secondary antenna in the I frame

w_S = mvnrnd(zeros(3,1), R_P)';

rS_G = R_IG'*rS_I;
rS = rS_G;

rStilde = rS + w_S;

%% rCtilde
rC = rS_G - rP_G;
norm_rC = norm(rC);
sigmaC = P.sensorParams.sigmaC;
rC_u = rC/norm(rC);
epsilon = 1e-8;
R_C = norm(rC,2)^2*sigmaC^2*(eye(3) - rC_u*(rC_u')) + epsilon*eye(3);

w_C = mvnrnd(zeros(3,1),R_C)';

rCtilde = rC + w_C;
norm_rCtilde = norm(rCtilde);
% Normalizing the rCtilde so norm(rCtilde) = norm(rC);
if norm_rCtilde == norm_rC
    rCtilde = rCtilde;
else
    rCtilde = rCtilde/norm(rCtilde)*norm(rC); 
end