function torques = NEshell(theta,thetadot,thetadotdot,Ftip)
% Uses the Newton-Euler inverse dynamics algorithm to calculate the torques
% needed to generate the trajectory given by the inputs, for the robot
% described in the lab assignment.
% theta, thetadot, and thetadotdot are three-element vectors, giving the trajectory.
% Ftip is a six-element vector, giving the wrench applied to the environment by the tip.

% define values of constants:
L1 = 2;
L2 = 1;
L3 = 0.5;
r = 0.1;
rho = 1;
g = 9.8;

% screw axes of joint i expressed in {i}:
A1 = transpose([0 0 1 0 (L1 / 2) 0]);
A2 = transpose([0 -1 0 (L2 / 2) 0 0]);
A3 = transpose([1 0 0 0 0 0]);

% configuration of {i-1} in {i} at home position:
m10 = [(- L1 / 2); 0; 0];
m21 = [(- L1 / 2); 0; (L2 / 2)];
m32 = [(- L3 / 2); 0; (L2 / 2)];
m43 = [(- L3 / 2); 0; 0];
bot = [0 0 0 1];
M10 = [eye(3) m10; bot];
M21 = [eye(3) m21; bot];
M32 = [eye(3) m32; bot];
M43 = [eye(3) m43; bot];

% spatial inertia matrices:
m_over_l = r * r * pi * rho;
m1 = m_over_l * L1;
m2 = m_over_l * L2;
m3 = m_over_l * L3;
i_small = 1/2*r^2;
i_large = 1/12*(3*r^2 + L1^2);
Ib1 = m1*[
    i_small 0 0;
    0 i_large 0;
    0 0 i_large
];
Ib2 = m2*[
    i_large 0 0;
    0 i_small 0;
    0 0 i_large
];
Ib3 = m3*[
    i_small 0 0;
    0 i_large 0;
    0 0 i_large
];
G1 = [Ib1 zeros(3); zeros(3) (eye(3)*m1)];
G2 = [Ib2 zeros(3); zeros(3) (eye(3)*m2)];
G3 = [Ib3 zeros(3); zeros(3) (eye(3)*m3)];

% gravity:
V0dot = [0 0 0 0 0 g]';
% assume the base is not moving:
V0 = zeros(6,1);

% calculate exp(-[A_i]theta_i):
expAtheta1 = expm_se3(A1, - theta(1));
expAtheta2 = expm_se3(A2, - theta(2));
expAtheta3 = expm_se3(A3, - theta(3));

% -- Forward iterations --

% calculate the quantities of Equations 8.50--8.52 for all joints:
[T10, AdT10, V1, adV1, V1dot] = forward_it(A1, expAtheta1, M10, V0, V0dot, thetadot(1), thetadotdot(1));
[T21, AdT21, V2, adV2, V2dot] = forward_it(A2, expAtheta2, M21, V1, V1dot, thetadot(2), thetadotdot(2));
[T32, AdT32, V3, adV3, V3dot] = forward_it(A3, expAtheta3, M32, V2, V2dot, thetadot(3), thetadotdot(3));
AdT21
adV2
% -- Backward iterations --

% first calculate the last adjoint matrix we'll need:
T43 = M43;
AdT43 = t_adjoint(T43);

% now calculate the quantities of Equations 8.53--8.54 for all joints:
[F3, tau3] = backward_it(A3, AdT43, Ftip, G3, V3, V3dot, adV3);
[F2, tau2] = backward_it(A2, AdT32, F3, G2, V2, V2dot, adV2);
[F1, tau1] = backward_it(A1, AdT21, F2, G1, V1, V1dot, adV1);

[tau1 tau2 tau3]
% return the vector of joint torques:
torques = [tau1 tau2 tau3]';
end
function lt = lt_cross(w)
    lt = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
end
function result = bracket_s(s)
    top_left = lt_cross(s(1:3));
    top_right = s(4:6);
    bottom = [0 0 0 0];
    result = [top_left top_right; bottom];
end 
function adj = t_adjoint(t)
    r = t(1:3, 1:3);
    p = t(1:3, 4);
    p_m = lt_cross(p);
    top_right = [0 0 0; 0 0 0; 0 0 0];
    bot_left = p_m * r;
    adj = [r top_right; bot_left r];
end
function ad = v_ad(v)
    w_m = - lt_cross(v(1:3));
    v_m = - lt_cross(v(4:6));
    ad = [w_m zeros(3); v_m w_m];
end
function [T, AdT, V, adV, Vdot] = forward_it(A, expA, m, V_prev, Vdot_prev, theta_d, theta_dd)
    T = expA * m;
    AdT = t_adjoint(T);
    V = AdT * V_prev + A*theta_d;
    adV = v_ad(V);
    Vdot = AdT*Vdot_prev + adV*A*theta_d + A*theta_dd;
end
function [f, tau] = backward_it(A, AdT, f_after, g, V, Vdot, adV)
    f = transpose(AdT) * f_after + g*Vdot - transpose(adV) * g * V;
    tau = transpose(f) * A;
end
function res = expm_so3(w, theta)
    res = eye(3) + sin(theta) * lt_cross(w) + (1 - cos(theta)) * lt_cross(w) ^ 2;
end
function res = expm_se3(screw, theta)
    w = screw(1:3);
    v = screw(4:6);
    top_left = expm_so3(w, theta);
    top_right = (eye(3) * theta + (1 - cos(theta)) * lt_cross(w) + (theta - sin(theta)) * lt_cross(w) ^ 2) * v ;
    bottom = [0 0 0 1];
    res = [top_left top_right; bottom];
end