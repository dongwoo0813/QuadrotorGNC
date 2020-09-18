% Top-level script for calling simulateQuadrotorControl or
% simulateQuadrotorEstimationAndControl

% 'clear all' is needed to clear out persistent variables from run to run
clear all; clc;
tic
% Seed Matlab's random number: this allows you to simulate with the same noise
% every time (by setting a nonnegative integer seed as argument to rng) or
% simulate with a different noise realization every time (by setting
% 'shuffle' as argument to rng).
rng('shuffle');
% Assert this flag to call the full estimation and control simulator;
% otherwise, only the control simulator is called
estimationFlag = 1;
% Total simulation time, in seconds
Tsim = 10;
% Update interval, in seconds
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
tVec=[0:N-1]'*delt;
% Angular rate of orbit, in rad/sec
n = 2*pi/10;
% Radius of circle, in meters
r = 2;
% Populate reference trajectory
R.tVec = tVec;
R.rIstar = [r*cos(n*tVec),r*sin(n*tVec),ones(N,1)];
R.vIstar = [-r*n*sin(n*tVec),r*n*cos(n*tVec),zeros(N,1)];
R.aIstar = [-r*n*n*cos(n*tVec),-r*n*n*sin(n*tVec),zeros(N,1)];
% The desired xI points toward the origin. The code below also normalizes
% each row in R.xIstar.
R.xIstar = diag(1./vecnorm(R.rIstar'))*(-R.rIstar);
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = 0*randn(N-1,3);
% Initial position in m
S.state0.r = [r 0 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 pi]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
% Oversampling factor
S.oversampFact = 10;
% Feature locations in the I frame
S.rXIMat = [0,0,0; 0,0,0.7];
% S.rXIMat = []; % To shut off camera

% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
sensorParamsScript;
P.quadParams = quadParams; 
P.constants = constants; 
P.sensorParams = sensorParams;

if(estimationFlag)
  Q = simulateQuadrotorEstimationAndControl(R,S,P);
else
  Q = simulateQuadrotorControl(R,S,P);
end

S2.tVec = Q.tVec;
S2.rMat = Q.state.rMat;
S2.eMat = Q.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=2.5*[-1 1 -1 1 -0.1 1];
visualizeQuad(S2);

figure(2);clf;
plot(Q.tVec,Q.state.rMat(:,3)); grid on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

figure(3);clf;
psiError = unwrap(n*Q.tVec + pi - Q.state.eMat(:,3));
meanPsiErrorInt = round(mean(psiError)/2/pi);
plot(Q.tVec,psiError - meanPsiErrorInt*2*pi);
grid on;
xlabel('Time (sec)');
ylabel('\Delta \psi (rad)');
title('Yaw angle error');

figure(5);clf;
plot(Q.state.rMat(:,1), Q.state.rMat(:,2));
hold on
[row,~] = size(Q.state.rMat(:,1));
xIstar = diag(1./vecnorm(Q.state.rMat'))*(-Q.state.rMat);
u = xIstar(:,1);
v = xIstar(:,2);
[x,y] = meshgrid(min(Q.state.rMat(:,1)):.2:max(Q.state.rMat(:,1)), min(Q.state.rMat(:,2)):.2:max(Q.state.rMat(:,2)));
quiver(Q.state.rMat(:,1), Q.state.rMat(:,2), u,v,'AutoScaleFactor',2,'MaxHeadSize',0.5);
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');

toc
%% PLOT FOR COMPARISON
% figure(5);clf;
% plot(Q.state.rMat(:,1), Q.state.rMat(:,2));
% hold on
% 
% % Setting reference trajectory vector to the size equivalent to true
% % trajectroy.
% [Number,~] = size(Q.state.rMat(:,1));
% tVec_Comp = linspace(0,10,Number)';
% rIstar_Comp = [r*cos(n*tVec_Comp),r*sin(n*tVec_Comp),ones(Number,1)];
% 
% plot(rIstar_Comp(:,1), rIstar_Comp(:,2));
% legend('True Trajectory','Reference Trajectory')
% axis equal; grid on;
% xlabel('X (m)');
% ylabel('Y (m)');
% 
% J = norm(rIstar_Comp - Q.state.rMat);
% title(['Horizontal position of CM, J = ',num2str(J)]);
% 
% figure(6);clf;
% plot(Q.tVec,Q.state.rMat(:,3)); grid on;
% hold on
% plot(R.tVec,R.rIstar(:,3))
% xlabel('Time (sec)');
% ylabel('Vertical (m)');
% title(['Vertical position of CM, J = ',num2str(J)]); 
% legend('True Trajectory','Reference Trajectory')



