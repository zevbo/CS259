syms w1 w2 w3 v1 v2 v3 z1 z2 m1 m2 n1 n2;

twist = [w1;w2;w3;v1;v2;v3];
theta = norm(twist(1:3));
screw = twist / theta;
t_ab = expm_se3(screw, theta);
p_a = [m1*z1;m2*z1;z1;1];
p_b = [n1*z2;n2*z2;z2;1];
t_ab * p_b

function rot = z_rot(rad)
    rot = [cos(rad) -sin(rad) 0; -sin(rad) cos(rad) 0; 0 0 1];
end
function rot = y_rot(rad)
    rot = [cos(rad) 0 sin(rad); 0 1 0; -sin(rad) 0 cos(rad)];
end
function rot = x_rot(rad)
    rot = [1 0 0; 0 cos(rad) -sin(rad); 0 sin(rad) cos(rad)];
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
function adj = t_adjoint(t)
    r = t(1:3, 1:3);
    p = t(1:3, 4);
    p_m = lt_cross(p);
    top_right = [0 0 0; 0 0 0; 0 0 0];
    bot_left = p_m * r;
    adj = [r top_right; bot_left r];
end
function j = space_j(screws, thetas)
    curr_t = eye(4);
    j = [];
    sz = size(screws);
    for i = 1:sz(2)
        curr_transform = t_adjoint(curr_t);
        screw = screws(1:6, i);
        j = [j (curr_transform * screw)];
        curr_t = curr_t * expm_se3(screw, thetas(i));
    end
end
function j = body_j(screws, thetas)
    curr_t = eye(4);
    j = [];
    sz = size(screws);
    for i_ = 1:sz(2)
        i = sz(2) - i_ + 1;
        curr_transform = t_adjoint(curr_t);
        screw = screws(1:6, i);
        j = [(curr_transform * screw) j];
        curr_t = curr_t * expm_se3(screw, - thetas(i));
    end
end