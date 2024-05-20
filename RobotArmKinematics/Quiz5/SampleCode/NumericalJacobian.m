function [J] = NumericalJacobian(Q)
%NumericalJacobian finds a numerical Jacobain represented in ForwardKinematics 
%   Input: Q = [q1; q2; q3], Joint angle vetor
%   Output: J = [dx/dq1, dx/dq2, dx/dq3], Jacobain
%               [dy/dq1, dy/dq2, dy/dq3]
%               [dz/dq1, dz/dq2, dz/dq3]

% Jacobian
J = zeros(3, 3);

% DeltaQ
DeltaQ = 1E-6;

% Forward kinematics is implemented in the function of ForwardKinematics
% Forward kinematics of Q
P = ForwardKinematics(Q);

% Forward kinematics of Q1 <- Q1+DeltaQ
P1 = ForwardKinematics([Q(1)+DeltaQ; Q(2); Q(3)]);
J(:,1) = (P1(:,4) - P(:,4)) / DeltaQ;

% Forward kinematics of Q2 <- Q2+DeltaQ
P2 = ForwardKinematics([Q(1); Q(2)+DeltaQ; Q(3)]);
J(:,2) = (P2(:,4) - P(:,4)) / DeltaQ;

% Forward kinematics of Q3 <- Q3+DeltaQ
P3 = ForwardKinematics([Q(1); Q(2); Q(3)+DeltaQ]);
J(:,3) = (P3(:,4) - P(:,4)) / DeltaQ;

end

