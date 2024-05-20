function [J] = NumericalJacobian(Q)
    % NumericalJacobian calculates the numerical Jacobian matrix
    J = zeros(3, 3);
    DeltaQ = 1E-6;

    % Forward kinematics
    P = ForwardKinematics(Q);

    % Calculate numerical Jacobian matrix
    for i = 1:3
        Q_temp = Q;
        Q_temp(i) = Q_temp(i) + DeltaQ;
        P_temp = ForwardKinematics(Q_temp);
        J(:, i) = (P_temp(:, end) - P(:, end)) / DeltaQ;
    end
end
