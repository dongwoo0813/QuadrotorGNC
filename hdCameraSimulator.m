function [rx] = hdCameraSimulator(rXI,S,P)
% hdCameraSimulator : Simulates feature location measurements from the
%                     quad's high-definition camera. 
%
%
% INPUTS
%
% rXI -------- 3x1 location of a feature point expressed in I in meters.
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
% OUTPUTS
%
% rx --------- 2x1 measured position of the feature point projection on the
%              camera's image plane, in pixels.  If the feature point is
%              not visible to the camera (the ray from the feature to the
%              camera center never intersects the image plane), then rx is
%              an empty matrix.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

f = P.sensorParams.f;
RCB = P.sensorParams.RCB;
RBI = S.statek.RBI;
Rc = zeros(2,2);
Rc = P.sensorParams.Rc;
pc = P.sensorParams.pixelSize;

RCI = RCB*RBI; % Rotation Matrix from Inertial Frame to Camera Frame;
rcI = zeros(3,1);
rcI = P.sensorParams.rc;
t = -(RCI*rcI); % Translational vector

P_1 = [f 0 0 0; 0 f 0 0; 0 0 1 0]; % Camera Projection Matrix 
P_K = P_1*[RCI, t; zeros(1,3), 1]; % P from I to Image Plane

x_c1 = P_K*[rXI;1]; % Image plane coordinate in H coordinates
x_c = [x_c1(1)/x_c1(3); x_c1(2)/x_c1(3)];


w_c = mvnrnd(zeros(2,1),Rc)';

rx = (1/pc)*x_c + w_c;

epsilon = 1e-8;

size_IP = P.sensorParams.imagePlaneSize;
if 	(x_c1(3) > epsilon) && (rx(1) <= size_IP(1)-epsilon) && (rx(2) <= size_IP(2)-epsilon)
% If the feature point is not visible to the camera, in other words,
% the ray from the feature to the camera center never intersects the image plane
    rx = (1/pc)*x_c + w_c; % 2x1 measured position of the feature point projection on the camera's image plane, in pixels.
else    
    rx = []; % Empty Matrix
end

