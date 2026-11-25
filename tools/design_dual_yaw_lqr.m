% Replace the example J and b with your plant values, then tune Ts, Q and R.
% State order [big angle, small relative angle, big rate, small relative rate].
% Torque output order [big, small]. Angles and rates are in radians and rad/s.
J_big_identified = 0.032;
big_inertia_includes_small = false; % Set true if the inertia was identified with both stages moving together.
J2 = 0.005505764873850784;
J1 = J_big_identified - big_inertia_includes_small * J2;
b1 = 0.020;
b2 = 0.03659462621840192;
Ts = 0.002;
Q = diag([900, 150, 0.5, 0.08]); % [big angle, small angle, big rate, small rate]
R = diag([1.6, 3.6]);               % [big torque, small torque]
big_angle_only = false;        % Set true to zero K13 and K14, then check stability.

assert(J1 > 0 && J2 > 0, 'Stage inertias must be positive');
M = [J1 + J2, J2; J2, J2];
D = diag([b1, b2]);
A = [zeros(2), eye(2); zeros(2), -(M\D)];
B = [zeros(2); M\eye(2)];
plant = c2d(ss(A, B, eye(4), zeros(4, 2)), Ts, 'zoh');
% K depends on the discrete plant and Q/R. References enter u = K*(xref-x).
K = dlqr(plant.A, plant.B, Q, R);
if big_angle_only
    K(1, 3:4) = 0;
end
pole_radius = max(abs(eig(plant.A - plant.B*K)));
assert(isfinite(pole_radius) && pole_radius < 1, ...
    'The selected gain is unstable in the linear model. Adjust Q/R or restore full feedback.');
fprintf('K (columns: big angle, small relative angle, big rate, small relative rate)\n');
disp(K);
fprintf('Maximum closed-loop pole modulus: %.9f\n\n', pole_radius);
for row = 1:2
    for col = 1:4
        fprintf('          k%d%d: %.9f\n', row, col, K(row, col));
    end
end
% Copy the printed k11 through k24 into DoubleYaw::Param or your app's YAML.
