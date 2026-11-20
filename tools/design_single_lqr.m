% Edit J, b, Q and R here, then copy the printed K into YAML.
% State: [position error; velocity error], u = K*(reference-state).
J = 0.005505764873850784; % Historical IDENTIFIED inertia candidate, not CAD J
b = 0.03659462621840192; % BUFF15-17 energy-fit candidate
Ts = 0.002;
Q = diag([120, 0.08]);
R = 0.12;
A = [0, 1; 0, -b/J];
B = [0; 1/J];
sys_d = c2d(ss(A, B, eye(2), zeros(2,1)), Ts, 'zoh');
[K, P, poles] = dlqr(sys_d.A, sys_d.B, Q, R);
assert(all(abs(poles)<1), 'Nominal closed loop is not stable');
disp(K); disp(poles);
fprintf('      k_theta: %.8f\n      k_omega: %.8f\n', K(1), K(2));
% Copy these gains to SingleLqr::Param; Gimbal selection is not integrated yet.
% An integrating BSP may expose k_theta/k_omega in its YAML configuration.
