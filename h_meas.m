function [zk] = h_meas(xk,wk,RBIBark,rXIMat,mcVeck,P)
% h_meas : Measurement model for quadcopter.
%
% INPUTS
%
% xk --------- 15x1 state vector at time tk, defined as 
% 
%              xk = [rI', vI', e', ba', bg']'
%
%              where all corresponding quantities are identical to those
%              defined for E.statek in stateEstimatorUKF.m and where e is the
%              3x1 error Euler angle vector defined such that for an estimate
%              RBIBar of the attitude, the true attitude is RBI =
%              dRBI(e)*RBIBar, where dRBI(e) is the DCM formed from the error
%              Euler angle vector e.
%
% wk --------- nz-by-1 measurement noise vector at time tk, defined as
%
%              wk = [wPIk', wCIk', w1C', w2C', ..., wNfkC']'
%
%              where nz = 6 + Nfk*3, with Nfk being the number of features
%              measured by the camera at time tk, and where all 3x1 noise
%              vectors represent additive noise on the corresponding
%              measurements.
%
% RBIBark ---- 3x3 attitude matrix estimate at time tk.
%
% rXIMat ----- Nf-by-3 matrix of coordinates of visual features in the
%              simulation environment, expressed in meters in the I
%              frame. rXIMat(i,:)' is the 3x1 vector of coordinates of the ith
%              feature.
%
% mcVeck ----- Nf-by-1 vector indicating whether the corresponding feature in
%              rXIMat is sensed by the camera: If mcVeck(i) is true (nonzero),
%              then a measurement of the visual feature with coordinates
%              rXIMat(i,:)' is assumed to be made by the camera.  mcVeck
%              should have Nfk nonzero values.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% zk --------- nz-by-1 measurement vector at time tk, defined as
%
%              zk = [rPItilde', rCItildeu', v1Ctildeu', ..., vNfkCtildeu']'
%
%              where rPItilde is the 3x1 measured position of the primary
%              antenna in the I frame, rCItildeu is the 3x1 measured unit
%              vector pointing from the primary to the secondary antenna,
%              expressed in the I frame, and viCtildeu is the 3x1 unit vector,
%              expressed in the camera frame, pointing toward the ith 3D
%              feature, which has coordinates rXIMat(i,:)'.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: 
%+==============================================================================+  
rI = xk(1:3);
e = xk(7:9);
dRBI = euler2dcm(e);
RBI = dRBI*RBIBark;

[nz, columnz] = size(wk);

zk = zeros(nz,1);

rA1 = P.sensorParams.rA(:,1);
rA2 = P.sensorParams.rA(:,2);
r_c = P.sensorParams.rc; % Maybe location of the camera center in B frame
RCB = P.sensorParams.RCB;

zk(1:3) = rI + (RBI')*rA1 + wk(1:3);
rCB_u = (rA2-rA1)/norm(rA2-rA1,2); % direction from primary to secondary antenna
zk(4:6) = RBI'*rCB_u + wk(4:6);

rcI = rI + RBI'*r_c;

[m,n] = size(rXIMat);

if nz > 6
    for IZONE = 1:m
        if mcVeck(IZONE)
            viI = rXIMat(IZONE,:)' - rcI;
            viI_u = viI/norm(viI);
            zk(7+3*(IZONE-1):9+3*(IZONE-1)) = RCB*RBI*viI_u + wk(7+(IZONE-1));    
        end
    end
end