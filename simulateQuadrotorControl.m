function [Q] = simulateQuadrotorControl(R,S,P)
% simulateQuadrotorControl : Simulates closed-loop control of a quadrotor
%                            aircraft.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%        rIstar = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
%        vIstar = Nx3 matrix of desired CM velocities with respect to the I
%                 frame and expressed in the I frame, in meters/sec.
%                 vIstar(k,:)' is the 3x1 velocity at time tk = tVec(k).
%
%        aIstar = Nx3 matrix of desired CM accelerations with respect to the I
%                 frame and expressed in the I frame, in meters/sec^2.
%                 aIstar(k,:)' is the 3x1 acceleration at time tk =
%                 tVec(k).
%
%        xIstar = Nx3 matrix of desired body x-axis direction, expressed as a
%                 unit vector in the I frame. xIstar(k,:)' is the 3x1
%                 direction at time tk = tVec(k).
%  
% S ---------- Structure with the following elements:
%
%  oversampFact = Oversampling factor. Let dtIn = R.tVec(2) - R.tVec(1). Then
%                 the output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.
%
%        state0 = State of the quad at R.tVec(1) = 0, expressed as a structure
%                 with the following elements:
%                   
%                   r = 3x1 position in the world frame, in meters
% 
%                   e = 3x1 vector of Euler angles, in radians, indicating the
%                       attitude
%
%                   v = 3x1 velocity with respect to the world frame and
%                       expressed in the world frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%       distMat = (N-1)x3 matrix of disturbance forces acting on the quad's
%                 center of mass, expressed in Newtons in the world frame.
%                 distMat(k,:)' is the constant (zero-order-hold) 3x1
%                 disturbance vector acting on the quad from R.tVec(k) to
%                 R.tVec(k+1).
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
% Q ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 Q.tVec(1) = R.tVec(1), Q.tVec(M) = R.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%  
%         state = State of the quad at times in tVec, expressed as a
%                 structure with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the I frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the I frame
%                       and expressed in the I frame, in meters per
%                       second.
%                 
%           omegaBMat = Mx3 matrix composed such that omegaBMat(k,:)' is the
%                       3x1 angular rate vector expressed in the body frame in
%                       radians, that applies at tVec(k).
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Edward Jung
%+==============================================================================+  
clc;
close all;

% Setting the parameter
tVecIn = R.tVec;
oversampFact = S.oversampFact;
N = length(tVecIn);
distMat = S.distMat;

Cm = P.quadParams.cm(1);

% Empty X0 to reorganize the structure of initial state.
X0 = zeros(22,1);

% Initial Values
r0 = S.state0.r;
e0 = S.state0.e;
v0 = S.state0.v;
omegaB0 = S.state0.omegaB;
R_BI0 = euler2dcm(e0);
omega_c = 0; % steady-state rotor rate, same for all rotors and constant

% eaMat = omega_c/Cm*ones((N-1),4); %eaMat = [(N-1)x4] matrix

% Reorganizing the initial state structure.
X0(1:6) = [r0;v0];
X0(7:15)= R_BI0(:);
X0(16:18) = omegaB0;
X0(19:22) = omega_c*ones(4,1);

dtIn = tVecIn(2) - tVecIn(1); % dt
dtOut = dtIn/oversampFact; % Output sample interval
if oversampFact < 1
   error('Oversampling Factor must be equal to or greater than 1.')
else
end

tVecOut = [];
XMat = [];
eMat = [];
Xk = X0;
IU.statek.rI = r0;
IU.statek.RBI = R_BI0;
IU.statek.vI = v0;
IU.statek.omegaB = omegaB0;

for k = 1:N-1
  
    % Build the time vector for kth segment.  We oversample by a factor
    % oversampFact relative to the coarse timing of each segment because we
    % may be interested in dynamical behavior that is short compared to the
    % segment length.
    tspan = [tVecIn(k):dtOut:tVecIn(k+1)]';
    
    DTraj.rIstark = R.rIstar(k,:)';
    DTraj.vIstark = R.vIstar(k,:)';
    DTraj.aIstark = R.aIstar(k,:)';
    
    [Fk,zIstark] = trajectoryController(DTraj,IU,P);
    DAttitude.zIstark = zIstark;
    DAttitude.xIstark = R.xIstar(k,:)';  
    
    NBk = attitudeController(DAttitude,IU,P);
    eak = voltageConverter(Fk,NBk,P);
    
    distVec = distMat(k,:)';
    
    % Run ODE solver for segment
    [tVeck,XMatk] = ode45(@(t,X)quadOdeFunctionHF(t,X,eak,distVec,P), tspan, Xk);

    % Add the data from the kth segment to your storage vector
    tVecOut = [tVecOut; tVeck(1:end-1)];
    XMat = [XMat; XMatk(1:end-1,:)];
    % Prepare for the next iteration
    Xk = XMatk(end,:)';
    IU.statek.rI = Xk(1:3);
    IU.statek.RBI=zeros(3,3);
    IU.statek.RBI(:)=Xk(7:15);
    IU.statek.vI = Xk(4:6);
    IU.statek.omegaB = Xk(16:18);
end

% Store the final state of the final segment
XMat = [XMat; XMatk(end,:)];
tVecOut = [tVecOut; tVeck(end,:)];
Q.tVec = tVecOut;       
Q.state.rMat = XMat(:,(1:3));
Q.state.vMat = XMat(:,(4:6));
for Bol4 = 1:length(tVecOut)
    R_BIk=zeros(3,3);
    R_BIk(:)=XMat(Bol4,7:15);
    e_k = dcm2euler(R_BIk)';
    eMat = [eMat; e_k];
end
Q.state.eMat = eMat;
Q.state.omegaBMat = XMat(:,(16:18));