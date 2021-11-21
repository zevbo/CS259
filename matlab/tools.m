syms L th1 th2 th3 th4 th5 th6;

[p511a, p511b, p511c, p511d] = p511();
[p515b, p515c] = p515();
[p513a, p513b] = p513();
[p517a, p517b, p517c, p517test] = p517();
test = test_det();

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
function [a, b, c, d] = p511()
    syms L;
    s1 = transpose([0 0 1 0 0 0]);
    s2 = transpose([0 -1 0 (2*L) 0 0]);
    s3 = transpose([0 -1 0 (2*L) 0 (-1*L)]);
    a = space_j([s1 s2 s3], [0, 0, 0]);
    b1 = transpose([0 0 1 (-L) (2*L) 0]);
    b2 = transpose([0 -1 0 0 0 (2*L)]);
    b3 = transpose([0 -1 0 0 0 L]);
    b_j = body_j([b1 b2 b3], [0, pi / 4, - pi / 4]);
    b = transpose(b_j) * transpose([0,0,0,10,0,0]);
    c = transpose(b_j) * transpose([10,0,0,0,0,0]);
    d = body_j([b1 b2 b3], [0, 0, 0]);
end
function [b, c] = p515()
    syms L t1 t2 t3;
    s1 = transpose([0 0 0 0 1 0]);
    s2 = transpose([0 0 1 0 0 0]);
    s3 = transpose([0 0 1 L L 0]);
    s4 = transpose([0 1 0 0 0 (-L)]);
    s5 = transpose([1 0 0 0 0 (-3 * L)]);
    s6 = transpose([0 0 0 0 1 0]);
    b = space_j([s1 s2 s3], [t1, t2, t3]);
    b1 = transpose([0 0 0 0 1 0]);
    b2 = transpose([0 0 1 (-5*L) (-1*L) 0]);
    b3 = transpose([0 0 1 (-4*L) 0 0]);
    b4 = transpose([0 1 0 0 0 0]);
    b5 = transpose([1 0 0 0 0 (2*L)]);
    b6 = transpose([0 0 0 0 1 0]);
    jb = body_j([b1 b2 b3 b4 b5 b6], [0 0 0 pi/2 0 0]);
    c = transpose(jb) * transpose([0 0 0 10 0 10]);
end
function [a_, b_] = p513()
    syms L a b c d e f;
    s1 = transpose([0 0 1 0 0 0]);
    s2 = transpose([0 1 0 0 0 0]);
    s3 = transpose([-1 0 0 0 0 0]);
    s4 = transpose([-1 0 0 0 0 L]);
    s5 = transpose([-1 0 0 0 0 (2*L)]);
    s6 = transpose([0 1 0 0 0 0]);
    a_ = space_j([s1 s2 s3 s4 s5 s6], [a b c d e f]);
    b1 = transpose([0 0 1 (-3*L) 0 0]);
    b2 = transpose([0 1 0 0 0 0]);
    b3 = transpose([-1 0 0 0 0 (-3*L)]);
    b4 = transpose([-1 0 0 0 0 (-2*L)]);
    b5 = transpose([-1 0 0 0 0 (L)]);
    b6 = transpose([0 1 0 0 0 0]);
    b_ = body_j([b1 b2 b3 b4 b5 b6], [1.2 (pi / 2) 0.2 0.2 -0.6 0.4]);
end
function [a, b, c, test] = p517()
    syms L t1 t2 t3 t4 t5 t6;
    s1 = transpose([0 0 0 0 0 1]);
    s2 = transpose([1 0 0 0 0 0]);
    s3 = transpose([0 0 1 L 0 0]);
    s4 = transpose([1 0 0 0 0 (-L)]);
    s5 = transpose([(sqrt(2)/2) (sqrt(2)/2) 0 0 0 (-sqrt(2)/2*L)]);
    s6 = transpose([0 0 0 0 1 0]);
    a = space_j([s1 s2 s3 s4 s5 s6], [t1 t2 t3 t4 t5 t6]);
    test = space_j([s1 s2 s3 s4 s5 s6], [1 1 1 1 1 1]);
    og_j = space_j([s1 s2 s3 s4 s5 s6], [0 0 0 0 0 0]);
    b = og_j * transpose([1 0 1 -1 2 0]);
    c = det(og_j);
end
function res = test_det()
    syms t1 t2 t3 t4 t5 t6 a1 b1 c1 d1 e1 f1 a2 b2 c2 d2 e2 f2 a3 b3 c3 d3 e3 f3 a4 b4 c4 d4 e4 f4 a5 b5 c5 d5 e5 f5 a6 b6 c6 d6 e6 f6
    s1 = transpose([a1 b1 c1 d1 e1 f1]);
    s2 = transpose([a2 b2 c2 d2 e2 f2]);
    s3 = transpose([a3 b3 c3 d3 e3 f3]);
    s4 = transpose([a4 b4 c4 d4 e4 f4]);
    s5 = transpose([a5 b5 c5 d5 e5 f5]);
    s6 = transpose([a6 b6 c6 d6 e6 f6]);
    res = space_j([s1 s2 s3 s4 s5 s6], [t1 t2 t3 t4 t5 t6]);
end