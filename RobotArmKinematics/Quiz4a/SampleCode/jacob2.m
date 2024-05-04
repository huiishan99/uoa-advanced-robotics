function [J] = jacob2(q, L1, L2)
%jaob2 returns jacobian of robot arm
%   q: joint angle vector, L1, L2: Link Length
J = [-L1*sin(q(1))-L2*(sin(q(1)+q(2))), -L2*sin(q(1)+q(2));
      L1*cos(q(1))+L2*(cos(q(1)+q(2))),  L2*cos(q(1)+q(2))];
end

