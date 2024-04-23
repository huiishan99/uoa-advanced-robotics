function [p] = fk1(q, L1)
%fk forward kinematics for 1-link planar arm
%   q: Joint angle vector, L1: Link length
x = L1*cos( q(1) );
y = L1*sin( q(1) );
p = [x; y];
end

