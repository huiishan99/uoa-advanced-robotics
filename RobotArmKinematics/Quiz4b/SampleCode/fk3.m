function [p] = fk3(q, L1, L2, L3)
%fk forward kinematics for 3-link planar arm
%   q: Joint angle vector, L1, L2, L3: Link length
x = L1*cos( q(1) ) + L2*cos( q(1)+q(2) ) + L3*cos( q(1)+q(2)+q(3) );
y = L1*sin( q(1) ) + L2*sin( q(1)+q(2) ) + L3*sin( q(1)+q(2)+q(3) );
p = [x; y];
end

