function theta = IKshell(T,theta0)
% Numerical calculation of UR5e inverse kinematics for desired end-effector position described by the transformation matrix T, starting from initial guess theta0 for the angles.

% Arm geometry from the FK lab (all values in mm):
l1 = 425;
l2 = 392;
w1 = 133;
w2 = 99.6;
h1 = -240;
h2 = 99.7;

b_1 = transpose([0, 1, 0, w1 + w2, 0, l1 + l2]);
b_2 = transpose([0, 0, 1, h2, -(l1 + l2), 0]);
b_3 = transpose([0, 0, 1, h2, -l2, 0]);
b_4 = transpose([0, 0, 1, h2, 0, 0]);
b_5 = transpose([0, -1, 0, -w2, 0, 0]);
b_6 = transpose([0, 0, 1, 0, 0, 0]);
body_screws = [b_1 b_2 b_3 b_4 b_5 b_6];

% Joint screw axes in the world frame (make sure this uses the same convention as we did in the FK lab), vector form:
e_w = 0.01;
e_v = 1;
theta = theta0;
while true
    result = FKShell(theta)^(-1) * T;
    log_result = logm(result);
    v_error = log_result(1:3,4);
    w_error = [log_result(3,2); log_result(1,3); log_result(2,1)];
    twist_error = [w_error; v_error];
    if norm(v_error) < e_v && norm(w_error) < e_w
        break
    else 
        j = jacobian(body_screws, theta);
        theta = theta + transpose(j) * (j * transpose(j)) ^ (-1) * twist_error;
    end
end







% Transformation between the world and body frames when at home position (M):



% Bracketed form of the joint screw axes in the world frame:







% Bracketed form of the screw axes in the body frame (make sure this uses the same convention as we did in the FK lab):







% Vector form of the screw axes in the body frame:







% Here begins the iterative algorithm described in the book:

% (a) Initialization


% (b) "Set [V_b] = log(T^{-1}_{ab}(theta^i)T_{ad}":

% 1. Calculate exp([B]theta) for each of the body screw axes and the current theta

% 2. Calculate the columns of the Jacobian using the above values and the body screw axes

% 3. Calculate the forward kinematics as a function of the current angles, using the body form of the product of exponentials formula

% 4. Calculate T_{bd}, the desired end-effector position in the body frame, using above values

% 5. Calculate the matrix logarithm of T_{bd}, using the algorithm given in the book for doing so

% 6. Set [V_b] using the above result

% 7. Convert [V_b] to V_b

% 8. Update the current guess for the angles, using above values

% Iterate






% Return the angles after the iteration has converged

