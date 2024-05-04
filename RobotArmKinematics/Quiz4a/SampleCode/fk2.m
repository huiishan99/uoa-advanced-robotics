function [p] = fk2(q, L1, L2)
%fk forward kinematics for 2-link planar arm
%   q: Joint angle vector, l1, l2: Link length
x = L1*cos( q(1) )+ L2*cos( q(1)+q(2) );
y = L1*sin( q(1) )+ L2*sin( q(1)+q(2) );
p = [x; y];
end

