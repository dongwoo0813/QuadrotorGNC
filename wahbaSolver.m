function [RBI] = wahbaSolver(aVec,vIMat,vBMat)
% wahbaSolver : Solves Wahba's problem via SVD.  In other words, this
%               function finds the rotation matrix RBI that minimizes the
%               cost Jw:
%
%                     N
%    Jw(RBI) = (1/2) sum ai*||viB - RBI*viI||^2
%                    i=1
%
%
% INPUTS
%
% aVec ------- Nx1 vector of least-squares weights.  aVec(i) is the weight
%              corresponding to the ith pair of vectors 
%
% vIMat ------ Nx3 matrix of 3x1 vectors expressed in the I frame.
%              vIMat(i,:)' is the ith 3x1 vector. 
%
% vBMat ------ Nx3 matrix of 3x1 vectors expressed in the B frame. vBMat(i,:)'
%              is the ith 3x1 vector, which corresponds to vIMat(i,:)';
%
% OUTPUTS
% 
% RBI -------- 3x3 direction cosine matrix indicating the attitude of the
%              B frame relative to the I frame.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  
B = zeros(3,3);
[row,~]=size(vBMat);
for IZONE = 1:row
    vB_i = vBMat(IZONE,:)';
    vI_i = vIMat(IZONE,:)';
    vBu_i = vB_i/norm(vB_i);
    vIu_i = vI_i/norm(vI_i);
    B = B + aVec(IZONE)*vBu_i*(vIu_i');
end

[U,S,V] = svd(B);
M = [1 0 0; 0 1 0; 0 0 det(U)*det(V)];
RBI = U*M*(V');