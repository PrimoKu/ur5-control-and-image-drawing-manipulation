%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function minNormIdx(qInvKin, q)
%
% * Purpose:
%   Compute the index for minimum norm b/w 8 joints from InvKin and q
% * Input:
%   *InvKin*: 6x8 joint space variable vector from InvKin
%   *q*: 6x1 joint space variable vector 
%

function idx = minNormIdx(qInvKin, q_current)
    norms = [];
    for i = 1:size(qInvKin,2)
        q_idx = qInvKin(:,i);
        norm_idx = norm(q_idx - q_current);
        norms = [norms; norm_idx];
    end
    [~, idx] = min(norms);
end
